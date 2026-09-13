#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <helpers/KeyValueStore.h>

/*
 * Two independent reasons to park the LoRa radio:
 *
 *   voltage gate  radio parks at/below `off_mv` and may return at/above
 *                 `on_mv`; the dead band between them absorbs load-induced sag
 *                 and ADC noise. Off by default.
 *   charge gate   radio runs only while the harvester PMIC reports a charging
 *                 current (MainBoard::getChargeCurrentNa). Off by default,
 *                 works with the voltage gate on or off. It has its own,
 *                 longer debounce window because weather is slower than a
 *                 voltage dip.
 *
 * A gate that is switched off does not veto anything; with both off the radio
 * is simply always on. With both on the radio needs both to allow it, and
 * either one can send it back to sleep. Each gate debounces on its own timer,
 * so tuning one does not change the other's reaction time.
 *
 * Weather and noise handling on the charge gate:
 *   - The decision uses the AVERAGE of the last CHG_AVG_SAMPLES readings, not
 *     a single sample. A harvester PMIC delivers a pulsed current, so one
 *     unlucky reading of ~0 must not be read as "the sun went away" - and an
 *     average must not let a single noisy sample starve the gate either.
 *   - The average must stay above the threshold for the whole `chg hold`
 *     window (default 15 min, floored at CHG_MIN_HOLD_MS). A cloud that passes
 *     for less than that can never park the radio, and alternating sun/cloud
 *     never accumulates, because the timer restarts whenever the average dips.
 *   - A failed I2C read is dropped from the average rather than counted as 0.
 *   - The threshold itself is configurable (`set batt.chg min=<mA>`) because a
 *     PMIC's resting reading is board specific; the value snaps to the nearest
 *     entry of CHG_MIN_UA, the way the NEH7100 driver snaps its own codes.
 *
 * The voltage gate keeps its dead band plus `hold_secs`; parking additionally
 * waits for `min_on_mins` ("run for x hours") whichever gate asked for it.
 * Implausible voltage readings hold state instead of driving a transition.
 *
 * Persisted: the count of completed full-recharge cycles (OFF -> ON while the
 * voltage gate is active) and the output state as of the last transition, so
 * cycles are still counted correctly across a reboot.
 *
 * Storage budget: exactly 8 keys. The DynamicConfigSerializer blob these live
 * in (NodePrefs' `custom` store) only parses 8 key/value pairs - a 9th key
 * silently evicts one of the others - so `last_on`, `seen`, `chg_enabled` and
 * the charge threshold's table index share `bg_st`, and the charge gate's hold
 * is a key of its own.
 *
 * Pure logic, no platform dependencies: the caller supplies millis(), the RTC
 * epoch, the battery reading and the charge current. That makes it unit-testable
 * (see test/test_batt_radio_gate).
 */

// Selectable minimum charging current that counts as "harvesting", in milliamps
// (stored as uA). A PMIC's resting reading is board specific, so this is picked
// at runtime and snaps to the nearest entry. 0 = strictly positive.
// Capped at 10 mA because that is what these PMICs can actually report: the
// NEH7100 reads an 8-bit value over four ranges (~18 uA / 122 uA / 1.2 mA /
// 17.2 mA), so a threshold anywhere near its 17 mA ceiling is unreachable and
// would hold the gate shut forever.
static const uint32_t CHG_MIN_UA[] = { 0, 50, 100, 200, 500, 1000, 2000, 5000, 10000 };
#define CHG_MIN_UA_COUNT (sizeof(CHG_MIN_UA) / sizeof(CHG_MIN_UA[0]))
#define CHG_MIN_MA_MAX    10.0f

// uA -> mA text, trailing zeros trimmed, no decimal when whole (0.1mA, 1mA)
static inline void chgFmtMa(char* out, size_t n, uint32_t ua) {
  if (ua % 1000 == 0) { snprintf(out, n, "%lu", (unsigned long)(ua / 1000)); return; }
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu.%03lu", (unsigned long)(ua / 1000), (unsigned long)(ua % 1000));
  char* e = buf + strlen(buf) - 1;
  while (*e == '0' && e[-1] != '.') *e-- = 0;
  snprintf(out, n, "%s", buf);
}

class BattRadioGate {
public:
  enum : uint8_t { STATE_OFF = 0, STATE_ON = 1 };

  // The charge gate never reacts faster than this, however `hold` is set.
  static const uint32_t CHG_MIN_HOLD_MS = 300000UL;   // 5 min
  // Charge readings averaged before the threshold test (one per sample tick).
  static const uint8_t  CHG_AVG_SAMPLES = 4;

  // config (persisted)
  uint8_t  enabled;        // voltage gate on/off
  uint8_t  chg_enabled;    // charge gate on/off (independent of `enabled`)
  uint16_t on_mv;          // voltage gate: radio may return at/above this
  uint16_t off_mv;         // voltage gate: radio parks at/below this
  uint16_t min_on_mins;    // minimum ON window, minutes
  uint16_t hold_secs;      // voltage gate: condition must hold this long
  uint16_t chg_hold_secs;  // charge gate: average must hold this long (default 15 min)
  uint8_t  chg_min_idx;    // index into CHG_MIN_UA (4 bits in bg_st)

  // stats (persisted)
  uint32_t cycles;         // completed full-recharge cycles
  uint8_t  last_on;        // output state as of the last persisted transition
  uint8_t  seen;           // gate has been initialised at least once

  BattRadioGate() { resetConfig(); }

  void resetConfig() {
    enabled = 0;
    chg_enabled = 0;
    on_mv = 4000;
    off_mv = 3400;
    min_on_mins = 360;
    hold_secs = 600;
    chg_hold_secs = 900;
    chg_min_idx = 0;
    cycles = 0;
    last_on = 0;
    seen = 0;
    _state = STATE_OFF;
    _state_since = 0;
    _pending = STATE_OFF;
    _last_mv = 0;
    _last_charge_na = -1;
    _volt.on = _volt.cand = 0; _volt.since = 0;
    _chg.on = _chg.cand = 0;   _chg.since = 0;
    clearChargeHistory();
    _primed = false;
    _dirty = false;
    _store = NULL;
  }

  bool isEnabled() const { return enabled != 0; }
  bool isChgEnabled() const { return chg_enabled != 0; }
  // true when at least one gate is switched on, i.e. the gate is not inert
  bool isActive() const { return enabled != 0 || chg_enabled != 0; }
  // An inert gate parks nothing, so it must report the radio as on - otherwise a
  // caller that just switched the gate off would still believe it was parked.
  bool isOn() const { return !isActive() || _state == STATE_ON; }
  bool isDirty() const { return _dirty; }
  uint16_t lastMilliVolts() const { return _last_mv; }
  int32_t lastChargeNa() const { return _last_charge_na; }   // raw, -1 = unknown
  int32_t chgAverageNa() const { return _chg_n > 0 ? _chg_avg : -1; }
  uint8_t chgSamples() const { return _chg_n; }

  // true when the gate is satisfied (always true when it is switched off)
  bool voltAllowed(uint16_t mv) const {
    return !enabled || (plausible(mv) && mv >= on_mv);
  }
  bool chgAllowed() const {
    return !chg_enabled || (_chg_n > 0 && _chg_avg > chgMinNa());
  }
  uint32_t chgMinUa() const { return CHG_MIN_UA[chg_min_idx]; }
  int32_t chgMinNa() const { return (int32_t)chgMinUa() * 1000; }

  void load(KeyValueStore& store) {
    resetConfig();
    _store = &store;
    char v[24];
    if (store.getByKey("bg_en", v, sizeof(v)))    enabled = atoi(v) ? 1 : 0;
    if (store.getByKey("bg_on", v, sizeof(v)))    on_mv = (uint16_t)atoi(v);
    if (store.getByKey("bg_off", v, sizeof(v)))   off_mv = (uint16_t)atoi(v);
    if (store.getByKey("bg_min", v, sizeof(v)))   min_on_mins = (uint16_t)atoi(v);
    if (store.getByKey("bg_hold", v, sizeof(v)))  hold_secs = (uint16_t)atoi(v);
    if (store.getByKey("bg_chold", v, sizeof(v))) chg_hold_secs = (uint16_t)atoi(v);
    if (store.getByKey("bg_cyc", v, sizeof(v)))   cycles = (uint32_t)strtoul(v, NULL, 10);
    if (store.getByKey("bg_st", v, sizeof(v))) {
      uint8_t st = (uint8_t)atoi(v);
      last_on = st & 1;
      seen = (st >> 1) & 1;
      chg_enabled = (st >> 2) & 1;
      uint8_t idx = (st >> 4) & 0x0F;
      chg_min_idx = (uint8_t)(idx < CHG_MIN_UA_COUNT ? idx : 0);
    }
  }

  bool save() {
    if (!_dirty || _store == NULL) return false;
    char v[24];
    _store->setByKey("bg_en", enabled ? "1" : "0");
    snprintf(v, sizeof(v), "%u", (unsigned)on_mv);          _store->setByKey("bg_on", v);
    snprintf(v, sizeof(v), "%u", (unsigned)off_mv);         _store->setByKey("bg_off", v);
    snprintf(v, sizeof(v), "%u", (unsigned)min_on_mins);    _store->setByKey("bg_min", v);
    snprintf(v, sizeof(v), "%u", (unsigned)hold_secs);      _store->setByKey("bg_hold", v);
    snprintf(v, sizeof(v), "%u", (unsigned)chg_hold_secs);  _store->setByKey("bg_chold", v);
    snprintf(v, sizeof(v), "%lu", (unsigned long)cycles);   _store->setByKey("bg_cyc", v);
    snprintf(v, sizeof(v), "%u", (unsigned)((chg_enabled ? 4u : 0u) |
                                            (seen ? 2u : 0u) | (last_on ? 1u : 0u) |
                                            ((unsigned)chg_min_idx << 4)));
    _store->setByKey("bg_st", v);
    _dirty = false;
    return true;
  }

  // Establish the output state at boot. Counts a cycle if we come up active
  // after having gone down parked (i.e. a recharge completed while off).
  void begin(uint16_t mv, bool external_power, int32_t charge_na,
             uint32_t now_ms, uint32_t epoch) {
    _last_mv = mv;
    _last_charge_na = charge_na;
    addChargeSample(charge_na);
    prime(mv, external_power, now_ms, epoch);
  }

  // Feed a fresh battery reading + charge current. Returns true if the
  // requested state changed.
  bool update(uint16_t mv, bool external_power, int32_t charge_na,
              uint32_t now_ms, uint32_t epoch) {
    _last_mv = mv;
    _last_charge_na = charge_na;
    addChargeSample(charge_na);
    uint8_t before = _state;

    if (isActive() && !_primed) {   // gate switched on at runtime: seed from this reading
      prime(mv, external_power, now_ms, epoch);
      return _state != before;
    }
    if (!isActive() || external_power) {   // inert / USB: always revivable
      setState(STATE_ON, now_ms, epoch, false);   // USB is not a recharge
      _pending = STATE_ON;
      return _state != before;
    }
    if (enabled && !plausible(mv)) return false;   // hold state on a bad reading

    // Each gate debounces on its own timer. A gate that is switched off allows.
    int want_volt = !enabled ? 1 : (mv >= on_mv ? 1 : (mv <= off_mv ? 0 : 2));  // 2 = dead band
    int want_chg  = !chg_enabled ? 1 : (chgAllowed() ? 1 : 0);
    step(_volt, want_volt, (uint32_t)hold_secs * 1000UL, now_ms);
    step(_chg,  want_chg,  effectiveChgHoldMs(), now_ms);

    uint8_t target = (_volt.on && _chg.on) ? STATE_ON : STATE_OFF;
    _pending = target;
    if (target == _state) return false;
    // min_on guards parking, whichever gate asked for it
    if (target == STATE_OFF &&
        (uint32_t)(now_ms - _state_since) < (uint32_t)min_on_mins * 60000UL) {
      return false;
    }

    setState(target, now_ms, epoch, true);
    return true;
  }

  void formatConfig(char* reply, size_t sz) const {
    snprintf(reply, sz, "> %s,on_mv=%u,off_mv=%u,min_on=%u,hold=%u,cycles=%lu",
             enabled ? "on" : "off", (unsigned)on_mv, (unsigned)off_mv,
             (unsigned)min_on_mins, (unsigned)hold_secs, (unsigned long)cycles);
  }

  void formatChg(char* reply, size_t sz) const {
    char ma[16];
    chgFmtMa(ma, sizeof(ma), chgMinUa());
    snprintf(reply, sz, "> %s,hold=%u,min=%smA", chg_enabled ? "on" : "off",
             (unsigned)chg_hold_secs, ma);
  }

  void formatStatus(char* reply, size_t sz) const {
    const char* pend = (_pending == _state) ? "none" : (_pending == STATE_ON ? "on" : "off");

    char volt[12];
    if (!enabled)                                      strcpy(volt, "off");
    else if (plausible(_last_mv) && _last_mv >= on_mv)  strcpy(volt, "ok");
    else if (_last_mv <= off_mv)                        strcpy(volt, "low");
    else                                                strcpy(volt, "hold");

    char chg[40];
    int32_t avg = chgAverageNa();
    if (avg < 0) {
      snprintf(chg, sizeof(chg), "unknown/%s(%u)", chgAllowed() ? "ok" : "no", (unsigned)_chg_n);
    } else {
      snprintf(chg, sizeof(chg), "%ld.%03ldmA/%s(%u)",
               (long)(avg / 1000000), (long)((avg % 1000000) / 1000),
               chgAllowed() ? "ok" : "no", (unsigned)_chg_n);
    }

    snprintf(reply, sz, "> radio=%s,mv=%u,volt=%s,chg=%s,cycles=%lu,pending=%s",
             _state == STATE_ON ? "on" : "off", (unsigned)_last_mv,
             volt, chg, (unsigned long)cycles, pend);
  }

  // Returns true if `command` was a battery-gate command (handled or rejected).
  bool handleCommand(const char* command, char* reply, size_t sz) {
    if (strcmp(command, "get batt.gate.status") == 0) { formatStatus(reply, sz); return true; }
    if (strcmp(command, "get batt.gate") == 0) { formatConfig(reply, sz); return true; }
    if (strcmp(command, "get batt.chg") == 0) { formatChg(reply, sz); return true; }

    if (memcmp(command, "set batt.chg ", 13) == 0) {
      const char* value = command + 13;
      if (strcmp(value, "on") == 0) {
        chg_enabled = 1;
      } else if (strcmp(value, "off") == 0) {
        chg_enabled = 0;
      } else if (memcmp(value, "hold=", 5) == 0) {
        long v = atol(value + 5);
        if (v < 0 || v > 86400) { strcpy(reply, "Err - hold range is 0-86400"); return true; }
        chg_hold_secs = (uint16_t)v;
        _dirty = true;
        formatChg(reply, sz);
        return true;
      } else if (memcmp(value, "min=", 4) == 0) {
        float ma = strtof(value + 4, NULL);
        if (ma < 0.0f || ma > CHG_MIN_MA_MAX) {
          strcpy(reply, "Err - min range is 0-10 mA");
          return true;
        }
        chg_min_idx = nearestChgMinIdx((uint32_t)(ma * 1000.0f + 0.5f));
        _dirty = true;
        formatChg(reply, sz);
        return true;
      } else {
        strcpy(reply, "Err - use on|off|hold=<secs>|min=<mA>");
        return true;
      }
      _pending = _state;
      _primed = false;   // re-seed from the next reading
      _dirty = true;
      formatChg(reply, sz);
      return true;
    }

    if (memcmp(command, "set batt.gate ", 14) != 0) return false;
    const char* value = command + 14;

    if (strcmp(value, "on") == 0 || strcmp(value, "off") == 0) {
      enabled = (value[1] == 'n') ? 1 : 0;
      _pending = _state;
      _primed = false;   // re-seed from the next reading
      _dirty = true;
      strcpy(reply, enabled ? "OK - voltage gate on" : "OK - voltage gate off");
      return true;
    }
    if (strcmp(value, "reset") == 0) {
      cycles = 0;
      _dirty = true;
      strcpy(reply, "OK - recharge cycle counter reset");
      return true;
    }

    // CSV of key=value pairs
    char buf[96];
    size_t n = strlen(value);
    if (n >= sizeof(buf)) { strcpy(reply, "Err - too long"); return true; }
    memcpy(buf, value, n + 1);

    uint16_t new_on = on_mv, new_off = off_mv, new_min = min_on_mins, new_hold = hold_secs;
    for (char* p = buf; *p; ) {
      char* comma = strchr(p, ',');
      if (comma) *comma = 0;
      char* eq = strchr(p, '=');
      if (eq == NULL) { snprintf(reply, sz, "Err - expected key=value, got '%s'", p); return true; }
      *eq = 0;
      long v = atol(eq + 1);
      if (strcmp(p, "on_mv") == 0 && v >= 3000 && v <= 4400) new_on = (uint16_t)v;
      else if (strcmp(p, "off_mv") == 0 && v >= 2900 && v <= 4400) new_off = (uint16_t)v;
      else if (strcmp(p, "min_on") == 0 && v >= 0 && v <= 10080) new_min = (uint16_t)v;
      else if (strcmp(p, "hold") == 0 && v >= 0 && v <= 86400) new_hold = (uint16_t)v;
      else { snprintf(reply, sz, "Err - bad field: %s", p); return true; }
      if (comma == NULL) break;
      p = comma + 1;
    }
    if (new_on <= new_off) {
      strcpy(reply, "Err - on_mv must exceed off_mv");
      return true;
    }

    on_mv = new_on;
    off_mv = new_off;
    min_on_mins = new_min;
    hold_secs = new_hold;
    _dirty = true;
    formatConfig(reply, sz);
    return true;
  }

private:
  struct Latch { uint8_t on, cand; uint32_t since; };

  uint8_t  _state, _pending;
  uint32_t _state_since;
  uint16_t _last_mv;
  int32_t  _last_charge_na;   // nA, negative = unsupported/unknown
  Latch    _volt, _chg;
  int32_t  _chg_hist[CHG_AVG_SAMPLES];
  uint8_t  _chg_n, _chg_i;
  int32_t  _chg_avg;
  bool     _primed;
  bool     _dirty;
  KeyValueStore* _store;

  static bool plausible(uint16_t mv) { return mv > 1000 && mv < 4400; }

  // nearest selectable threshold (given in uA), the way the NEH7100 driver
  // snaps its own register codes
  static uint8_t nearestChgMinIdx(uint32_t ua) {
    uint8_t best = 0;
    uint32_t best_d = 0xFFFFFFFFUL;
    for (uint8_t i = 0; i < CHG_MIN_UA_COUNT; i++) {
      uint32_t d = CHG_MIN_UA[i] > ua ? CHG_MIN_UA[i] - ua : ua - CHG_MIN_UA[i];
      if (d < best_d) { best_d = d; best = i; }
    }
    return best;
  }

  uint32_t effectiveChgHoldMs() const {
    uint32_t h = (uint32_t)chg_hold_secs * 1000UL;
    return h < CHG_MIN_HOLD_MS ? CHG_MIN_HOLD_MS : h;
  }

  void clearChargeHistory() {
    for (uint8_t i = 0; i < CHG_AVG_SAMPLES; i++) _chg_hist[i] = 0;
    _chg_n = 0;
    _chg_i = 0;
    _chg_avg = 0;
  }

  // Rolling average over the last CHG_AVG_SAMPLES valid readings. A failed read
  // is dropped instead of being averaged in as zero, so a flaky I2C bus reads
  // as "no news" rather than "no sun".
  void addChargeSample(int32_t na) {
    if (na < 0) return;
    _chg_hist[_chg_i] = na;
    _chg_i = (uint8_t)((_chg_i + 1) % CHG_AVG_SAMPLES);
    if (_chg_n < CHG_AVG_SAMPLES) _chg_n++;
    int32_t sum = 0;
    for (uint8_t k = 0; k < _chg_n; k++) sum += _chg_hist[k];
    _chg_avg = sum / _chg_n;
  }

  // want: 1 = allow, 0 = refuse, 2 = no candidate change (dead band)
  static void step(Latch& l, int want, uint32_t hold_ms, uint32_t now_ms) {
    if (want == 2 || (uint8_t)want == l.on) {   // nothing pending
      l.cand = l.on;
      l.since = now_ms;
      return;
    }
    if (l.cand != (uint8_t)want) {   // candidate appeared / changed: restart the timer
      l.cand = (uint8_t)want;
      l.since = now_ms;
    }
    if ((uint32_t)(now_ms - l.since) >= hold_ms) l.on = (uint8_t)want;
  }

  void prime(uint16_t mv, bool external_power, uint32_t now_ms, uint32_t epoch) {
    _state = _pending = STATE_OFF;
    _state_since = now_ms;
    _volt.on = _volt.cand = voltAllowed(mv) ? 1 : 0;
    _chg.on = _chg.cand = chgAllowed() ? 1 : 0;
    _volt.since = _chg.since = now_ms;
    if (external_power || (_volt.on && _chg.on)) {
      // `seen` keeps a first-ever boot from being booked as a recharge cycle;
      // after that, coming up active with last_on==0 means a recharge finished
      // while we were parked.
      setState(STATE_ON, now_ms, epoch, seen && !external_power);
    } else if (isActive() && last_on) {
      last_on = 0;   // came up parked
      _dirty = true;
    }
    _pending = _state;
    _primed = true;
    if (isActive() && !seen) { seen = 1; _dirty = true; }
  }

  void setState(uint8_t s, uint32_t now_ms, uint32_t epoch, bool count_cycle) {
    if (s == _state) return;
    _state = s;
    _state_since = now_ms;
    uint8_t was = last_on;
    last_on = (s == STATE_ON) ? 1 : 0;
    // cycles track the voltage gate's recharges only
    if (s == STATE_ON && enabled && count_cycle && !was) cycles++;
    // An inert gate never persists anything, so a build that never switches it
    // on does not touch flash.
    if (isActive()) _dirty = true;
  }
};

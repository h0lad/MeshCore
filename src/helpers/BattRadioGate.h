#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <helpers/KeyValueStore.h>

/*
 * Battery-gated repeater: parks the LoRa radio until the cell has recovered.
 *
 *   radio ON   once the battery has been at/above `on_mv` for `hold_secs`
 *              (a stable reading, not a solar/ADC spike)
 *   radio OFF  once it has been at/below `off_mv` for `hold_secs` AND the
 *              minimum ON window `min_on_mins` ("run for x hours") has elapsed
 *
 * The dead band (on_mv - off_mv) plus the hold time is what keeps load-induced
 * voltage sag and ADC noise from bouncing the radio. Readings outside the
 * plausible Li-ion window are ignored rather than acted upon.
 *
 * Persisted alongside the config: the count of completed full-recharge cycles
 * (OFF -> ON transitions) and the output state as of the last transition, so
 * cycles are still counted correctly across a reboot.
 *
 * Storage budget: exactly 8 keys. The DynamicConfigSerializer blob these live
 * in (NodePrefs' `custom` store) only parses 8 key/value pairs - a 9th key
 * silently evicts one of the others - so `last_on` and `seen` share `bg_st`.
 *
 * Pure logic, no platform dependencies: the caller supplies millis(), the RTC
 * epoch and the battery reading. That makes it unit-testable (see
 * test/test_batt_radio_gate).
 */
class BattRadioGate {
public:
  enum : uint8_t { STATE_OFF = 0, STATE_ON = 1 };

  // config (persisted)
  uint8_t  enabled;        // feature on/off; disabled = radio always on
  uint16_t on_mv;          // radio may come back at/above this
  uint16_t off_mv;         // radio parks at/below this
  uint16_t min_on_mins;    // minimum ON window, minutes
  uint16_t hold_secs;      // candidate condition must hold this long

  // stats (persisted)
  uint32_t cycles;         // completed full-recharge cycles
  uint32_t last_cycle;     // RTC epoch of the last completed cycle
  uint8_t  last_on;        // output state as of the last persisted transition
  uint8_t  seen;           // gate has been initialised at least once

  BattRadioGate() { resetConfig(); }

  void resetConfig() {
    enabled = 0;
    on_mv = 4000;
    off_mv = 3400;
    min_on_mins = 360;
    hold_secs = 600;
    cycles = 0;
    last_cycle = 0;
    last_on = 0;
    seen = 0;
    _state = _pending = STATE_OFF;
    _state_since = _pending_since = 0;
    _last_mv = 0;
    _primed = false;
    _dirty = false;
    _store = NULL;
  }

  bool isEnabled() const { return enabled != 0; }
  bool isOn() const { return _state == STATE_ON; }
  bool isDirty() const { return _dirty; }
  uint16_t lastMilliVolts() const { return _last_mv; }

  void load(KeyValueStore& store) {
    resetConfig();
    _store = &store;
    char v[24];
    if (store.getByKey("bg_en", v, sizeof(v)))   enabled = atoi(v) ? 1 : 0;
    if (store.getByKey("bg_on", v, sizeof(v)))   on_mv = (uint16_t)atoi(v);
    if (store.getByKey("bg_off", v, sizeof(v)))  off_mv = (uint16_t)atoi(v);
    if (store.getByKey("bg_min", v, sizeof(v)))  min_on_mins = (uint16_t)atoi(v);
    if (store.getByKey("bg_hold", v, sizeof(v))) hold_secs = (uint16_t)atoi(v);
    if (store.getByKey("bg_cyc", v, sizeof(v)))  cycles = (uint32_t)strtoul(v, NULL, 10);
    if (store.getByKey("bg_last", v, sizeof(v))) last_cycle = (uint32_t)strtoul(v, NULL, 10);
    if (store.getByKey("bg_st", v, sizeof(v))) {
      uint8_t st = (uint8_t)atoi(v);
      last_on = st & 1;
      seen = (st >> 1) & 1;
    }
  }

  bool save() {
    if (!_dirty || _store == NULL) return false;
    char v[24];
    _store->setByKey("bg_en", enabled ? "1" : "0");
    snprintf(v, sizeof(v), "%u", (unsigned)on_mv);         _store->setByKey("bg_on", v);
    snprintf(v, sizeof(v), "%u", (unsigned)off_mv);        _store->setByKey("bg_off", v);
    snprintf(v, sizeof(v), "%u", (unsigned)min_on_mins);   _store->setByKey("bg_min", v);
    snprintf(v, sizeof(v), "%u", (unsigned)hold_secs);     _store->setByKey("bg_hold", v);
    snprintf(v, sizeof(v), "%lu", (unsigned long)cycles);  _store->setByKey("bg_cyc", v);
    snprintf(v, sizeof(v), "%lu", (unsigned long)last_cycle); _store->setByKey("bg_last", v);
    snprintf(v, sizeof(v), "%u", (unsigned)((seen ? 2u : 0u) | (last_on ? 1u : 0u)));
    _store->setByKey("bg_st", v);
    _dirty = false;
    return true;
  }

  // Establish the output state at boot. Counts a cycle if we come up active
  // after having gone down parked (i.e. a recharge completed while off).
  void begin(uint16_t mv, bool external_power, uint32_t now_ms, uint32_t epoch) {
    _last_mv = mv;
    prime(mv, external_power, now_ms, epoch);
  }

  // Feed a fresh battery reading. Returns true if the requested state changed.
  bool update(uint16_t mv, bool external_power, uint32_t now_ms, uint32_t epoch) {
    _last_mv = mv;
    uint8_t before = _state;

    if (enabled && !_primed) {   // just enabled at runtime: seed from this reading
      prime(mv, external_power, now_ms, epoch);
      return _state != before;
    }
    if (!enabled || external_power) {   // kill switch / USB: always revivable
      setState(STATE_ON, now_ms, epoch, false);   // USB is not a recharge
      _pending = STATE_ON;
      _pending_since = now_ms;
      return _state != before;
    }
    if (!plausible(mv)) return false;   // hold state on a bad reading

    uint8_t target = _state;
    if (mv >= on_mv) target = STATE_ON;
    else if (mv <= off_mv) target = STATE_OFF;

    if (target == _state) { _pending = target; return false; }

    if (_pending != target) { _pending = target; _pending_since = now_ms; }
    if ((uint32_t)(now_ms - _pending_since) < (uint32_t)hold_secs * 1000UL) return false;
    if (target == STATE_OFF && (uint32_t)(now_ms - _state_since) < (uint32_t)min_on_mins * 60000UL) {
      return false;   // still inside the minimum ON window
    }

    setState(target, now_ms, epoch, true);
    return true;
  }

  void formatConfig(char* reply, size_t sz) const {
    snprintf(reply, sz, "> %s,on_mv=%u,off_mv=%u,min_on=%u,hold=%u,cycles=%lu",
             enabled ? "on" : "off", (unsigned)on_mv, (unsigned)off_mv,
             (unsigned)min_on_mins, (unsigned)hold_secs, (unsigned long)cycles);
  }

  void formatStatus(char* reply, size_t sz) const {
    const char* pend = (_pending == _state) ? "none" : (_pending == STATE_ON ? "on" : "off");
    snprintf(reply, sz, "> radio=%s,mv=%u,cycles=%lu,last=%lu,pending=%s",
             _state == STATE_ON ? "on" : "off", (unsigned)_last_mv,
             (unsigned long)cycles, (unsigned long)last_cycle, pend);
  }

  // Returns true if `command` was a batt.gate command (handled or rejected).
  bool handleCommand(const char* command, char* reply, size_t sz) {
    if (strcmp(command, "get batt.gate.status") == 0) { formatStatus(reply, sz); return true; }
    if (strcmp(command, "get batt.gate") == 0) { formatConfig(reply, sz); return true; }

    if (memcmp(command, "set batt.gate ", 14) != 0) return false;
    const char* value = command + 14;

    if (strcmp(value, "on") == 0 || strcmp(value, "off") == 0) {
      enabled = (value[1] == 'n') ? 1 : 0;
      _pending = _state;
      _primed = false;   // re-seed from the next reading
      _dirty = true;
      if (!enabled) {
        strcpy(reply, "OK - batt.gate off (radio stays on)");
      } else {
        strcpy(reply, "OK - batt.gate on");
      }
      return true;
    }
    if (strcmp(value, "reset") == 0) {
      cycles = 0;
      last_cycle = 0;
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
    _pending = _state;
    _dirty = true;
    formatConfig(reply, sz);
    return true;
  }

private:
  uint8_t  _state, _pending;
  uint32_t _state_since, _pending_since;
  uint16_t _last_mv;
  bool     _primed;
  bool     _dirty;
  KeyValueStore* _store;

  static bool plausible(uint16_t mv) { return mv > 1000 && mv < 4400; }

  void prime(uint16_t mv, bool external_power, uint32_t now_ms, uint32_t epoch) {
    _state = _pending = STATE_OFF;
    _state_since = _pending_since = now_ms;
    if (!enabled || external_power || (plausible(mv) && mv >= on_mv)) {
      // `seen` keeps a first-ever boot from being booked as a recharge cycle;
      // after that, coming up active with last_on==0 means a recharge finished
      // while we were parked.
      setState(STATE_ON, now_ms, epoch, seen && !external_power);
    } else if (enabled && last_on) {
      last_on = 0;   // came up parked
      _dirty = true;
    }
    _pending = _state;
    _primed = true;
    if (enabled && !seen) { seen = 1; _dirty = true; }
  }

  void setState(uint8_t s, uint32_t now_ms, uint32_t epoch, bool count_cycle) {
    if (s == _state) return;
    _state = s;
    _state_since = now_ms;
    uint8_t was = last_on;
    last_on = (s == STATE_ON) ? 1 : 0;
    if (s == STATE_ON && enabled && count_cycle && !was) {   // a recharge completed
      cycles++;
      last_cycle = epoch;
    }
    // A disabled gate is a pass-through: it never persists anything, so a
    // build that never enables it does not touch flash.
    if (enabled) _dirty = true;
  }
};

#include <helpers/NEH7100.h>

// Register encodings (NEH7100 data sheet, Table 24)
static const uint32_t OVP_MV[16] = {2700, 2900, 3100, 3300, 3400, 3500, 3600, 3700,
                                    3800, 3900, 4000, 4100, 4200, 4300, 4400, 4500};
static const uint32_t LVD_MV[16] = {2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900,
                                    3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700};
static const uint32_t LDO_MV[8]  = {1200, 1500, 1800, 2000, 2400, 3000, 3300, 3600};
static const uint32_t USB_UA[8]  = {500, 1000, 2000, 10000, 50000, 100000, 150000, 200000};

// I_RANGE (0x09<1:0>) -> charge-current scale, 0.1 nA per LSB
static const uint32_t SCALE_TENTHS_NA[4] = {706, 4780, 47100, 675000};

static uint8_t nearest_code(const uint32_t* tbl, int n, uint32_t target)
{
  uint8_t best = 0;
  uint32_t best_d = 0xFFFFFFFFUL;
  for (int i = 0; i < n; i++) {
    uint32_t d = tbl[i] > target ? tbl[i] - target : target - tbl[i];
    if (d < best_d) { best_d = d; best = (uint8_t)i; }
  }
  return best;
}

// Compact milli-scaled value: /1000 with one decimal, the decimal dropped when
// it is zero (4200 mV -> "4.2" V, 200000 uA -> "200" mA, 500 uA -> "0.5" mA).
static void fmtMilli(char* out, size_t n, uint32_t milli)
{
  if (milli % 1000 == 0) snprintf(out, n, "%lu", (unsigned long)(milli / 1000));
  else                   snprintf(out, n, "%lu.%lu", (unsigned long)(milli / 1000), (unsigned long)((milli % 1000) / 100));
}

NEH7100::NEH7100(TwoWire& bus, uint8_t addr) : _bus(bus), _addr(addr) {}

bool NEH7100::begin()
{
  _bus.begin();
  uint8_t id = 0;
  if (readReg(REG_CHIP_ID, &id)) {
    Serial.printf("[NEH7100] %s (chip_id=0x%02X)\n", id == CHIP_ID_VAL ? "present" : "NOT found", id);
    return id == CHIP_ID_VAL;
  }
  Serial.println("[NEH7100] NOT found (I2C read failed)");
  return false;
}

bool NEH7100::readReg(uint8_t reg, uint8_t* val)
{
  _bus.beginTransmission(_addr);
  _bus.write(reg);
  if (_bus.endTransmission(false) != 0) return false;
  if (_bus.requestFrom(_addr, 1) != 1) return false;
  *val = (uint8_t)_bus.read();
  return true;
}

bool NEH7100::writeReg(uint8_t reg, uint8_t val)
{
  _bus.beginTransmission(_addr);
  _bus.write(reg);
  _bus.write(val);
  return _bus.endTransmission() == 0;
}

int32_t NEH7100::chargeCurrentNa()
{
  uint8_t range, meas;
  if (!readReg(REG_I_RANGE, &range) || !readReg(REG_I_MEAS, &meas)) return -1;
  return (int32_t)(((uint32_t)meas * SCALE_TENTHS_NA[range & 0x03U]) / 10U);
}

uint8_t  NEH7100::ovpCode(uint16_t mv) { return nearest_code(OVP_MV, 16, mv); }
uint8_t  NEH7100::lvdCode(uint16_t mv) { return nearest_code(LVD_MV, 16, mv); }
uint8_t  NEH7100::ldoCode(uint16_t mv) { return nearest_code(LDO_MV, 8, mv); }
uint8_t  NEH7100::usbCode(uint32_t ua) { return nearest_code(USB_UA, 8, ua); }
uint16_t NEH7100::ovpMv(uint8_t code) { return (uint16_t)OVP_MV[code & 0x0FU]; }
uint16_t NEH7100::lvdMv(uint8_t code) { return (uint16_t)LVD_MV[code & 0x0FU]; }
uint16_t NEH7100::ldoMv(uint8_t code) { return (uint16_t)LDO_MV[code & 0x07U]; }

bool NEH7100::setOvpMv(uint16_t mv, uint16_t* actual)
{
  uint8_t code = ovpCode(mv), r;
  if (!readReg(REG_OVP_LVD, &r)) return false;
  if (!writeReg(REG_OVP_LVD, (r & 0xF0U) | code)) return false;
  _ovp_code = code;
  if (actual) *actual = ovpMv(code);
  return true;
}

bool NEH7100::setLvdMv(uint16_t mv, uint16_t* actual)
{
  uint8_t code = lvdCode(mv), r;
  if (!readReg(REG_OVP_LVD, &r)) return false;
  if (!writeReg(REG_OVP_LVD, (r & 0x0FU) | (code << 4))) return false;
  _lvd_code = code;
  if (actual) *actual = lvdMv(code);
  return true;
}

bool NEH7100::setLdoMv(uint16_t mv, uint16_t* actual)
{
  if (mv < LDO_MIN_MV) return false;
  uint8_t code = ldoCode(mv), r;
  if (!readReg(REG_LDO_USB, &r)) return false;
  // clear LDO_BP (bit7) so the LDO regulates; keep LDO_CTRL (bit6) and USB (2:0)
  if (!writeReg(REG_LDO_USB, (r & 0x47U) | (code << 3))) return false;
  if (actual) *actual = ldoMv(code);
  return true;
}

bool NEH7100::setUsbUa(uint32_t ua, uint32_t* actual)
{
  uint8_t code = usbCode(ua), r;
  if (!readReg(REG_LDO_USB, &r)) return false;
  if (!writeReg(REG_LDO_USB, (r & 0xF8U) | code)) return false;
  if (actual) *actual = USB_UA[code];
  return true;
}

void NEH7100::setTempLimits(int8_t minC, int8_t maxC)
{
  _temp_min = minC;
  _temp_max = maxC;
}

void NEH7100::enableTempLimit(bool on)
{
  if (on && !_temp_enabled) {
    uint8_t r;
    if (readReg(REG_OVP_LVD, &r)) {
      _ovp_code = r & 0x0F;   // remember the OVP to restore
      _lvd_code = r >> 4;
    }
  }
  _temp_enabled = on;
  _temp_active = false;
}

void NEH7100::tick(float mcuTempC, uint16_t battMv)
{
  uint32_t now = millis();
  if (now - _last_tick_ms < 5000) return;
  _last_tick_ms = now;
  if (!_temp_enabled) return;

  int16_t t = (int16_t)mcuTempC;

  if (_temp_active) {
    // safeguard: never let the temp limit drain the battery to LVD
    uint16_t critical = lvdMv(_lvd_code) + 300;
    if (battMv > 0 && battMv < critical) {
      uint8_t r;
      if (readReg(REG_OVP_LVD, &r)) writeReg(REG_OVP_LVD, (r & 0xF0U) | (_ovp_code & 0x0FU));
      _temp_active = false;
      _temp_enabled = false;
      Serial.println("[NEH7100] temp-limit auto-disabled: battery critical");
      return;
    }
    if (t >= _temp_min && t <= _temp_max) {
      uint8_t r;
      if (readReg(REG_OVP_LVD, &r)) writeReg(REG_OVP_LVD, (r & 0xF0U) | (_ovp_code & 0x0FU));
      _temp_active = false;
      Serial.println("[NEH7100] charging re-enabled (temp in range)");
    }
  } else if (t < _temp_min || t > _temp_max) {
    uint8_t r;
    if (readReg(REG_OVP_LVD, &r)) writeReg(REG_OVP_LVD, (r & 0xF0U) | 0x00U);   // OVP 2.7 V stops charging
    _temp_active = true;
    Serial.println("[NEH7100] charging stopped (temp out of range)");
  }
}

void NEH7100::earlyRecovery(TwoWire& bus, uint8_t addr)
{
  bus.begin();
  uint8_t r = 0;
  bus.beginTransmission(addr);
  bus.write(REG_OVP_LVD);
  if (bus.endTransmission(false) == 0 && bus.requestFrom(addr, 1) == 1) r = (uint8_t)bus.read();
  if ((r & 0x0FU) == 0x00U) {   // OVP stuck at 2.7 V -> temp guard was limiting at reset
    bus.beginTransmission(addr);
    bus.write(REG_OVP_LVD);
    bus.write((uint8_t)((r & 0xF0U) | OVP_DEFAULT_CODE));
    bus.endTransmission();
  }
}

bool NEH7100::handleCommand(char* cmd, char* reply, float mcuTempC)
{
  while (*cmd == ' ') cmd++;

  if (strcmp(cmd, "neh") == 0 || strcmp(cmd, "neh.get") == 0) {
    uint8_t r0, r1;
    if (!readReg(REG_OVP_LVD, &r0) || !readReg(REG_LDO_USB, &r1)) {
      strcpy(reply, "ERR: NEH7100 I2C read failed");
      return true;
    }
    char ovp[10], lvd[10], ldo[10], usb[10], ibat[12];
    fmtMilli(ovp, sizeof(ovp), ovpMv(r0 & 0x0F));
    fmtMilli(lvd, sizeof(lvd), lvdMv(r0 >> 4));
    fmtMilli(ldo, sizeof(ldo), LDO_MV[(r1 >> 3) & 0x07]);
    fmtMilli(usb, sizeof(usb), USB_UA[r1 & 0x07]);
    int32_t na = chargeCurrentNa();
    if (na < 0) strcpy(ibat, "ERR");
    else snprintf(ibat, sizeof(ibat), "%ld.%02ld", (long)(na / 1000000), (long)((na % 1000000) / 10000));
    sprintf(reply, "OVP=%sV LVD=%sV LDO=%sV USB=%smA Ibat=%smA T=%dC %s",
            ovp, lvd, ldo, usb, ibat, (int)mcuTempC,
            _temp_active ? "LIMITED" : (_temp_enabled ? "armed" : "off"));
    return true;
  }

  if (memcmp(cmd, "neh.curr", 8) == 0) {
    int32_t na = chargeCurrentNa();
    if (na < 0) strcpy(reply, "ERR: NEH7100 I2C read failed");
    else sprintf(reply, "> %ld.%03ld mA", (long)(na / 1000000), (long)((na % 1000000) / 1000));
    return true;
  }

  if (strcmp(cmd, "neh.temp") == 0) {
    sprintf(reply, "> %s min=%dC max=%dC%s",
            _temp_enabled ? "on" : "off", _temp_min, _temp_max,
            _temp_active ? " LIMITED" : "");
    return true;
  }
  if (strcmp(cmd, "neh.temp on") == 0) {
    enableTempLimit(true);
    strcpy(reply, "OK - temp charge limit on");
    return true;
  }
  if (strcmp(cmd, "neh.temp off") == 0) {
    if (_temp_active) {
      uint8_t r;
      if (readReg(REG_OVP_LVD, &r)) writeReg(REG_OVP_LVD, (r & 0xF0U) | (_ovp_code & 0x0FU));
    }
    _temp_active = false;
    _temp_enabled = false;
    strcpy(reply, "OK - temp charge limit off");
    return true;
  }

  if (memcmp(cmd, "set neh.ovp ", 12) == 0) {
    uint16_t a = 0;
    if (!setOvpMv((uint16_t)atoi(cmd + 12), &a)) strcpy(reply, "ERR: I2C write failed");
    else sprintf(reply, "OK - OVP %u mV", a);
    return true;
  }
  if (memcmp(cmd, "set neh.lvd ", 12) == 0) {
    uint16_t a = 0;
    if (!setLvdMv((uint16_t)atoi(cmd + 12), &a)) strcpy(reply, "ERR: I2C write failed");
    else sprintf(reply, "OK - LVD %u mV", a);
    return true;
  }
  if (memcmp(cmd, "set neh.ldo ", 12) == 0) {
    uint16_t mv = (uint16_t)atoi(cmd + 12);
    if (mv < LDO_MIN_MV) { strcpy(reply, "ERR: LDO min 3000 mV"); return true; }
    uint16_t a = 0;
    if (!setLdoMv(mv, &a)) strcpy(reply, "ERR: I2C write failed");
    else sprintf(reply, "OK - LDO %u mV", a);
    return true;
  }
  if (memcmp(cmd, "set neh.usb ", 12) == 0) {
    uint32_t a = 0;
    if (!setUsbUa((uint32_t)atoi(cmd + 12) * 1000U, &a)) strcpy(reply, "ERR: I2C write failed");
    else sprintf(reply, "OK - USB %lu uA", (unsigned long)a);
    return true;
  }
  if (memcmp(cmd, "set neh.temp ", 13) == 0) {
    int minc = atoi(cmd + 13);
    char* sp = strchr(cmd + 13, ' ');
    int maxc = sp ? atoi(sp + 1) : _temp_max;
    if (minc >= maxc) { strcpy(reply, "ERR: need min < max"); return true; }
    setTempLimits((int8_t)minc, (int8_t)maxc);
    sprintf(reply, "OK - temp range %d..%dC", _temp_min, _temp_max);
    return true;
  }

  return false;
}

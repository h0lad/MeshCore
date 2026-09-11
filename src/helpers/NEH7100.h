#pragma once

#include <Arduino.h>
#include <Wire.h>

// NEH7100 energy-harvesting PMIC (I2C, 7-bit address 0x3C).
// Board-agnostic driver + charge controller.
class NEH7100 {
public:
  enum {
    ADDR_DEFAULT = 0x3C,
    REG_OVP_LVD  = 0x00,   // [7:4]=LVD, [3:0]=OVP
    REG_LDO_USB  = 0x01,   // [7]=LDO_BP, [6]=LDO_CTRL, [5:3]=VLDO, [2:0]=USB max
    REG_STATUS   = 0x08,
    REG_CHIP_ID  = 0x07,
    REG_I_RANGE  = 0x09,   // [1:0] current measurement range
    REG_I_MEAS   = 0x0A,

    CHIP_ID_VAL = 0x15,

    // LDO floor: above the RAK3172/STM32WLE5 VCC min (2.0 V) and the STM32WL
    // SMPS minimum for 22 dBm TX (2.7 V); lowest NEH7100 step above both is 3.0 V.
    LDO_MIN_MV = 3000,

    OVP_DEFAULT_CODE = 0x0B,   // 4.1 V

    // Consecutive low battery readings before the temp limit gives way. tick()
    // runs every 5 s and a TX burst lasts well under that, so one sagged
    // reading can never trip it; a real discharge strikes three times in a row.
    BATT_LOW_STRIKES = 3,
    // Margin above the critical level before the limit is re-armed, so a pack
    // hovering at the threshold does not latch the guard on and off.
    BATT_RECOVER_MV = 200,
  };

  NEH7100(TwoWire& bus = Wire, uint8_t addr = ADDR_DEFAULT);

  bool begin();   // start bus + probe CHIP_ID

  bool readReg(uint8_t reg, uint8_t* val);
  bool writeReg(uint8_t reg, uint8_t val);
  int32_t chargeCurrentNa();   // -1 on I2C failure

  bool setOvpMv(uint16_t mv, uint16_t* actual = nullptr);
  bool setLvdMv(uint16_t mv, uint16_t* actual = nullptr);
  bool setLdoMv(uint16_t mv, uint16_t* actual = nullptr);
  bool setUsbUa(uint32_t ua, uint32_t* actual = nullptr);

  // Temperature-gated charging. tick() lowers OVP below VBAT to stop charging
  // when the MCU temperature leaves [minC, maxC]. If the pack still discharges
  // towards LVD while the limit is holding charging off, tick() force-enables
  // charging (battery before temperature) and re-arms the limit once the pack
  // has recovered - a single low reading is not believed, because a transmit
  // burst sags the rail and must not silently disable the limit for good.
  void setTempLimits(int8_t minC, int8_t maxC);
  void enableTempLimit(bool on);
  bool tempLimitEnabled() const { return _temp_enabled; }
  bool tempLimitActive() const { return _temp_active; }
  bool tempChargeForced() const { return _batt_forced; }
  void tick(float mcuTempC, uint16_t battMv);

  // CLI. Returns true if the command was handled.
  bool handleCommand(char* cmd, char* reply, float mcuTempC = 0);

  // Brownout recovery. Call before setup() (e.g. initVariant): the PMIC keeps
  // OVP at the "stop" level (2.7 V) across an MCU reset, so clear it early.
  static void earlyRecovery(TwoWire& bus = Wire, uint8_t addr = ADDR_DEFAULT);

  static uint16_t ovpMv(uint8_t code);
  static uint16_t lvdMv(uint8_t code);
  static uint16_t ldoMv(uint8_t code);

private:
  uint8_t ovpCode(uint16_t mv);
  uint8_t lvdCode(uint16_t mv);
  uint8_t ldoCode(uint16_t mv);
  uint8_t usbCode(uint32_t ua);

  TwoWire& _bus;
  uint8_t  _addr;
  bool     _temp_enabled = false;
  bool     _temp_active = false;
  int8_t   _temp_min = 0;
  int8_t   _temp_max = 45;
  uint8_t  _ovp_code = OVP_DEFAULT_CODE;
  uint8_t  _lvd_code = 0x06;   // 2.8 V
  uint8_t  _low_strikes = 0;
  bool     _batt_forced = false;
  uint32_t _last_tick_ms = 0;

  void writeOvpCode(uint8_t code);
};

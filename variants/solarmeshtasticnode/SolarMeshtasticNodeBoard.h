#pragma once

#include "../heltec_ct62/HT-CT62Board.h"

#if defined(ESP_PLATFORM)

// SolarMeshtasticNode is a Heltec CT62-based solar node with custom I2C/VBAT
// wiring. It reuses the CT62 board, only overriding the battery reading to
// use the ESP32-C3 eFuse calibration + a calibrated voltage divider.
class SolarMeshtasticNodeBoard : public Heltec_CT62_Board {
public:
  uint16_t getBattMilliVolts() override {
  #ifdef PIN_VBAT_READ
    // Voltage divider on the LiPo. analogReadMilliVolts() applies the ESP32-C3
    // factory eFuse calibration, so it returns calibrated mV *at the pin*.
    // VBAT_DIVIDER = V_battery / V_pin. For a true 1:1 divider this is 2.0;
    // calibrate it against a multimeter for your actual resistor values.
    #ifndef VBAT_DIVIDER
      #define VBAT_DIVIDER 2.0f
    #endif
    analogSetPinAttenuation(PIN_VBAT_READ, ADC_11db);  // full ~0..2500mV usable range
    uint32_t raw = 0;
    uint32_t mv  = 0;
    for (int i = 0; i < 8; i++) {
      raw += analogRead(PIN_VBAT_READ);
      mv  += analogReadMilliVolts(PIN_VBAT_READ);
    }
    raw /= 8;
    mv  /= 8;

    uint16_t vbat = (uint16_t)(mv * VBAT_DIVIDER);
    MESH_DEBUG_PRINTLN("[VBAT] pin=%d raw=%u mv_at_pin=%u -> vbat=%u mV",
                       PIN_VBAT_READ, raw, mv, vbat);

    return vbat;
  #else
    return 0;  // not supported
  #endif
  }
};

#endif

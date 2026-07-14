#pragma once

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/SensorManager.h>
#include <helpers/radiolib/CustomSTM32WLxWrapper.h>
#include <helpers/radiolib/RadioLibWrappers.h>
#include <helpers/stm32/STM32Board.h>

// Shared target for boards built around the RAK3172 (STM32WLE5) module: the radio,
// the RF switch table and the battery/GPIO handling are properties of the module, not
// of the carrier. A variant opts in with -D STM32WLX_COMMON_TARGET and overrides the
// defaults below where its carrier differs.

#ifndef STM32WLX_BOARD_NAME
  #define STM32WLX_BOARD_NAME  "Generic STM32WL"
#endif

#ifndef PIN_VBAT_READ
  #define PIN_VBAT_READ   A0   // ADC_BAT on PB3 / ADC1_IN2
#endif

// A board that declares VBAT_DIVIDER (the ratio of its battery divider) is read
// exactly; the rest keep this empirically calibrated multiplier.
#if !defined(VBAT_DIVIDER) && !defined(ADC_MULTIPLIER)
  #define ADC_MULTIPLIER  (5 * 1.73 * 1000)
#endif

#ifndef STM32WLX_LED0
  #define STM32WLX_LED0   PA0
#endif
#ifndef STM32WLX_LED1
  #define STM32WLX_LED1   PA1
#endif

class STM32WLxBoard : public STM32Board {
public:
    void begin() override {
        STM32Board::begin();
        pinMode(STM32WLX_LED0, OUTPUT);
        pinMode(STM32WLX_LED1, OUTPUT);
    }

    const char* getManufacturerName() const override {
        return STM32WLX_BOARD_NAME;
    }

    uint16_t getBattMilliVolts() override {
        analogReadResolution(12);
#ifdef VBAT_DIVIDER
        // Scale against VDDA measured through the factory-trimmed internal reference,
        // rather than assuming the rail sits at exactly 3.3V.
        uint32_t vdda = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_12B);
#endif
        uint32_t raw = 0;
        for (int i=0; i<8;i++) {
            raw += analogRead(PIN_VBAT_READ);
        }
#ifdef VBAT_DIVIDER
        return __LL_ADC_CALC_DATA_TO_VOLTAGE(vdda, raw / 8, LL_ADC_RESOLUTION_12B) * VBAT_DIVIDER;
#else
        return ((double)raw) * ADC_MULTIPLIER / 8 / 4096;
#endif
    }

    void setGpio(uint32_t values) override {
        // set led values
        digitalWrite(STM32WLX_LED0, values & 1);
        digitalWrite(STM32WLX_LED1, (values & 2) >> 1);
    }

    uint32_t getGpio() override {
        // get led value
        return (digitalRead(STM32WLX_LED1) << 1) | digitalRead(STM32WLX_LED0);
    }
};

extern STM32WLxBoard board;
extern WRAPPER_CLASS radio_driver;
extern VolatileRTCClock rtc_clock;
extern SensorManager sensors;

bool radio_init();
mesh::LocalIdentity radio_new_identity();

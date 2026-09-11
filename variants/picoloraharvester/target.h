#pragma once

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/SensorManager.h>
#include <helpers/radiolib/CustomSTM32WLxWrapper.h>
#include <helpers/radiolib/RadioLibWrappers.h>
#include <helpers/stm32/STM32Board.h>
#include <Wire.h>

// PicoLoRaHarvester carrier for the RAK3172 (STM32WLE5).
//
//   ADC_LIPO       PB4 / A1  (ADC1_IN3), divider R13 390k / R14 1M  ->  x1.39
//   EN_VOL_DIVIDER PB3 / A0  n-MOSFET (Q1) load switch for the divider, active LOW
//   NEH7100 PMIC   I2C2: PA11 SDA / PA12 SCL  (the default Wire on RAK3172)

#define PIN_VBAT_READ         A1
#define VBAT_DIVIDER          1.39f

#define STM32WLX_BAT_EN_PIN    A0
#define STM32WLX_BAT_EN_ACTIVE LOW
#define STM32WLX_BAT_SETTLE_MS 200

class PicoLoRaHarvesterBoard : public STM32Board {
public:
    void begin() override {
        STM32Board::begin();
        pinMode(PA0, OUTPUT);   // LED0
        pinMode(PA1, OUTPUT);   // LED1
        pinMode(STM32WLX_BAT_EN_PIN, OUTPUT);
        digitalWrite(STM32WLX_BAT_EN_PIN, !STM32WLX_BAT_EN_ACTIVE);   // divider off by default
        Wire.begin();   // NEH7100 PMIC config bus
    }

    const char* getManufacturerName() const override {
        return "Pico LoRa Harvester";
    }

    uint16_t getBattMilliVolts() override {
        analogReadResolution(12);

        // Energize the divider and let its filter cap settle before sampling the
        // high-impedance tap.
        digitalWrite(STM32WLX_BAT_EN_PIN, STM32WLX_BAT_EN_ACTIVE);
        delay(STM32WLX_BAT_SETTLE_MS);

        // Scale against VDDA measured through the factory-trimmed internal reference,
        // rather than assuming the rail sits at exactly 3.3V.
        uint32_t vdda = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_12B);

        uint32_t raw = 0;
        for (int i = 0; i < 8; i++) {
            raw += analogRead(PIN_VBAT_READ);
        }

        digitalWrite(STM32WLX_BAT_EN_PIN, !STM32WLX_BAT_EN_ACTIVE);   // de-energize divider

        return __LL_ADC_CALC_DATA_TO_VOLTAGE(vdda, raw / 8, LL_ADC_RESOLUTION_12B) * VBAT_DIVIDER;
    }

    void setGpio(uint32_t values) override {
        // set led values
        digitalWrite(PA0, values & 1);
        digitalWrite(PA1, (values & 2) >> 1);
    }

    uint32_t getGpio() override {
        // get led value
        return (digitalRead(PA1) << 1) | digitalRead(PA0);
    }
};

extern PicoLoRaHarvesterBoard board;
extern WRAPPER_CLASS radio_driver;
extern VolatileRTCClock rtc_clock;
extern SensorManager sensors;

bool radio_init();
mesh::LocalIdentity radio_new_identity();

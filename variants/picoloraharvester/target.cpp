#include "target.h"
#include <Arduino.h>
#include <helpers/ArduinoHelpers.h>

// Runs before setup(). Restore the NEH7100 OVP as early as possible: a brownout
// while the temp guard was limiting leaves OVP stuck at 2.7 V because the PMIC
// survives the MCU reset. No Serial here (not yet initialised).
extern "C" void initVariant()
{
    NEH7100::earlyRecovery();
}

PicoLoRaHarvesterBoard board;

RADIO_CLASS radio = new STM32WLx_Module();

WRAPPER_CLASS radio_driver(radio, board);

static const uint32_t rfswitch_pins[] = {LORAWAN_RFSWITCH_PINS, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
    {STM32WLx::MODE_IDLE, {LOW, LOW}},
    {STM32WLx::MODE_RX, {HIGH, LOW}},
    {STM32WLx::MODE_TX_LP, {LOW, HIGH}},
    {STM32WLx::MODE_TX_HP, {LOW, HIGH}},
    END_OF_MODE_TABLE,
};

VolatileRTCClock rtc_clock;
SensorManager sensors;

#ifndef LORA_CR
#define LORA_CR 5
#endif

// Auto-detect the oscillator: RadioLib falls back to the crystal if it fails to
// start with the TCXO voltage, so one build runs on both crystal and -T modules.
#ifndef STM32WL_TCXO_VOLTAGE
#define STM32WL_TCXO_VOLTAGE 1.8
#endif

#ifndef LORA_TX_POWER
#define LORA_TX_POWER 22
#endif

bool radio_init()
{
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

    int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16,
                             STM32WL_TCXO_VOLTAGE, 0);

    if (status != RADIOLIB_ERR_NONE) {
        Serial.print("ERROR: radio init failed: ");
        Serial.println(status);
        return false; // fail
    }

#ifdef RX_BOOSTED_GAIN
    radio.setRxBoostedGainMode(RX_BOOSTED_GAIN);
#endif

    radio.setCRC(1);

    return true; // success
}

mesh::LocalIdentity radio_new_identity()
{
    RadioNoiseListener rng(radio);
    return mesh::LocalIdentity(&rng); // create new random identity
}

float PicoLoRaHarvesterBoard::getMCUTemperature()
{
    analogReadResolution(12);
    uint32_t vdda = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_12B);
    uint32_t ts_raw = analogRead(ATEMP);
    return (float)__LL_ADC_CALC_TEMPERATURE(vdda, ts_raw, LL_ADC_RESOLUTION_12B);
}

bool neh_handle_command(char* cmd, char* reply)
{
    return board.neh.handleCommand(cmd, reply, board.getMCUTemperature());
}

void neh_loop()
{
    board.neh.tick(board.getMCUTemperature(), board.getBattMilliVolts());
}

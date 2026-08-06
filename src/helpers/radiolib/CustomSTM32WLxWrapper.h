#pragma once

#include "CustomSTM32WLx.h"
#include "RadioLibWrappers.h"
#include "SX126xReset.h"
#include <math.h>

class CustomSTM32WLxWrapper : public RadioLibWrapper {
public:
  CustomSTM32WLxWrapper(CustomSTM32WLx& radio, mesh::MainBoard& board) : RadioLibWrapper(radio, board) { }

  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override {
    ((CustomSTM32WLx *)_radio)->setFrequency(freq);
    ((CustomSTM32WLx *)_radio)->setSpreadingFactor(sf);
    ((CustomSTM32WLx *)_radio)->setBandwidth(bw);
    ((CustomSTM32WLx *)_radio)->setCodingRate(cr);
    updatePreamble(sf);
    PacketMillis pm = calcMaxPacketMillis(sf, bw, cr, preambleLengthForSF(sf));
    ((CustomSTM32WLx *)_radio)->setPreambleMillis(pm.preambleMillis);
    ((CustomSTM32WLx *)_radio)->setMaxPayloadMillis(pm.payloadMillis);
  }

  bool isReceivingPacket() override {
    return ((CustomSTM32WLx *)_radio)->isReceiving();
  }
  bool isChipBusy() override {
    return ((CustomSTM32WLx *)_radio)->isChipBusy();
  }
  float getCurrentRSSI() override {
    return ((CustomSTM32WLx *)_radio)->getRSSI(false);
  }

  float packetScore(float snr, int packet_len) override {
    int sf = ((CustomSTM32WLx *)_radio)->spreadingFactor;
    return packetScoreInt(snr, sf, packet_len);
  }
  uint8_t getSpreadingFactor() const override { return ((CustomSTM32WLx *)_radio)->spreadingFactor; }

  bool supportsRxPowerSaving() const override { return true; }

protected:
  int startReceiveMode() override {
    if (_rx_ps_armed) {
      // leaving duty-cycle mode (after RxDone or a reconfig): stop the
      // sequencer and the still-running RTC, or its pending event can
      // silently abort the RX we are about to start
      stopReceiveDutyCycle();
    }
    if (!_rx_ps_enabled || _nf_calib_active) {
      // plain continuous RX: powersaving off, or a periodic noise-floor
      // calibration window is in progress
      return _radio->startReceive();
    }

    const RadioLibIrqFlags_t irqFlags = RADIOLIB_IRQ_RX_DEFAULT_FLAGS;
    const RadioLibIrqFlags_t irqMask =
        (1UL << RADIOLIB_IRQ_RX_DONE) |
        (1UL << RADIOLIB_IRQ_TIMEOUT) |
        (1UL << RADIOLIB_IRQ_CRC_ERR) |
        (1UL << RADIOLIB_IRQ_HEADER_ERR);

    int err = ((CustomSTM32WLx *)_radio)->startReceiveDutyCycle(_rx_ps_rx_us, _rx_ps_sleep_us, irqFlags, irqMask);
    if (err == RADIOLIB_ERR_NONE) {
      _rx_ps_armed = true;
      return err;
    }

    MESH_DEBUG_PRINTLN("CustomSTM32WLxWrapper: error: startReceiveDutyCycle(%d), falling back to continuous RX", err);
    return _radio->startReceive();
  }

  void stopReceiveDutyCycle() override {
    _radio->standby();   // also wakes the chip if it is in the sleep window
    ((CustomSTM32WLx *)_radio)->stopRTC();
    _rx_ps_armed = false;
  }

public:
  void doResetAGC() override { sx126xResetAGC((SX126x *)_radio, getRxBoostedGainMode()); }
};

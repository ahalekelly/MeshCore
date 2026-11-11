#pragma once

#include "CustomSX1262.h"
#include "RadioLibWrappers.h"
#include "SX126xReset.h"

#ifndef USE_SX1262
#define USE_SX1262
#endif

class CustomSX1262Wrapper : public RadioLibWrapper {
public:
  CustomSX1262Wrapper(CustomSX1262& radio, mesh::MainBoard& board) : RadioLibWrapper(radio, board) { }

  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override {
    ((CustomSX1262 *)_radio)->setFrequency(freq);
    ((CustomSX1262 *)_radio)->setSpreadingFactor(sf);
    ((CustomSX1262 *)_radio)->setBandwidth(bw);
    ((CustomSX1262 *)_radio)->setCodingRate(cr);
    updatePreamble(sf);
    PacketMillis pm = calcMaxPacketMillis(sf, bw, cr, preambleLengthForSF(sf));
    ((CustomSX1262 *)_radio)->setPreambleMillis(pm.preambleMillis);
    ((CustomSX1262 *)_radio)->setMaxPayloadMillis(pm.payloadMillis);
  }

  void formatChipDiag(char* out) override { ((CustomSX1262 *)_radio)->formatDiag(out); }

  bool debugCommand(const char* command, char* out) override {
    auto radio = (CustomSX1262 *)_radio;
    if (strcmp(command, "radiodiag2") == 0) {
      radio->formatDiag2(out);
      size_t len = strlen(out);
      snprintf(out + len, 150 - len, " mcu=%d up=%lu", (int)_board->getMCUTemperature(), (unsigned long)(millis() / 1000));
    } else if (strncmp(command, "radioreg ", 9) == 0) {
      unsigned addr, len;
      char extra;
      if (sscanf(command + 9, "%x %u %c", &addr, &len, &extra) != 2 || addr > 0xFFFF || len < 1 || len > 64 || addr + len > 0x10000) {
        snprintf(out, 150, "ERROR: radioreg addr_hex len(1..64)");
        return true;
      }
      uint8_t data[64] = {};
      radio->readRegister(addr, data, len);
      snprintf(out, 150, "%04X:", addr);
      for (unsigned i = 0; i < len; i++) snprintf(out + 5 + i * 2, 150 - 5 - i * 2, "%02X", data[i]);
    } else if (strncmp(command, "radiofix ", 9) == 0) {
      const char* action = command + 9;
      if (strcmp(action, "clr") == 0) radio->clearIrqStatus(0xFFFF);
      else if (strcmp(action, "rx") == 0) idle();
      else if (strcmp(action, "agc") == 0) debugResetAGC();
      else { snprintf(out, 150, "ERROR: radiofix clr|rx|agc"); return true; }
      snprintf(out, 150, "OK");
    } else if (strncmp(command, "radiofault ", 11) == 0) {
      const char* mode = command + 11;
      if (strcmp(mode, "stdby") == 0) radio->standby(RADIOLIB_SX126X_STANDBY_RC);
      else if (strcmp(mode, "xosc") == 0) radio->standby(RADIOLIB_SX126X_STANDBY_XOSC);
      else if (strcmp(mode, "fs") == 0) radio->setFs();
      else if (strcmp(mode, "sleep") == 0) radio->sleep(true);
      else if (strcmp(mode, "rxto") == 0) radio->setRx(64);
      else { snprintf(out, 150, "ERROR: radiofault stdby|xosc|fs|sleep|rxto"); return true; }
      snprintf(out, 150, "OK");
    } else if (strncmp(command, "radiostress ", 12) == 0) {
      unsigned n, delay = 5000;
      char extra;
      int args = sscanf(command + 12, "%u %u %c", &n, &delay, &extra);
      if (args < 1 || args > 2 || n < 1 || delay > 262143999) {
        snprintf(out, 150, "ERROR: radiostress n(1..2000) [tcxo_delay_us]");
        return true;
      }
      if (n > 2000) n = 2000;
#ifdef SX126X_DIO3_TCXO_VOLTAGE
      constexpr float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
#else
      constexpr float tcxo = 1.6f;
#endif
      if (args == 2) radio->setTCXO(tcxo, delay);
      unsigned errors = 0, notrx = 0, first = 0, derr = 0;
      uint8_t firstbad = 0;
      uint16_t derror = 0;
      int16_t last = 0;
      uint32_t started = millis();
      for (unsigned i = 0; i < n; i++) {
        int16_t result = radio->startReceive();
        if (result != RADIOLIB_ERR_NONE) { errors++; last = result; }
        uint8_t status = 0;
        radio->getMod()->SPIreadStream(RADIOLIB_SX126X_CMD_GET_STATUS, &status, 1);
        if (((status >> 4) & 7) != 5) {
          if (notrx == 0) { firstbad = status; first = i; }
          notrx++;
        }
        uint16_t dev = radio->getDeviceErrors();
        if (dev) { derr++; derror |= dev; radio->clearDeviceErrors(); }
        yield();
      }
      if (args == 2) radio->setTCXO(tcxo, 5000);
      debugIdle();
      snprintf(out, 150, "n=%u err=%u lasterr=%d notrx=%u firstbad=%02X@%u derr=%u derror=%04X ms=%lu",
        n, errors, last, notrx, firstbad, first, derr, derror, (unsigned long)(millis() - started));
    } else return false;
    return true;
  }

  bool isReceivingPacket() override { 
    return ((CustomSX1262 *)_radio)->isReceiving();
  }
  float getCurrentRSSI() override {
    return ((CustomSX1262 *)_radio)->getRSSI(false);
  }
  float getLastRSSI() const override { return ((CustomSX1262 *)_radio)->getRSSI(); }
  float getLastSNR() const override { return ((CustomSX1262 *)_radio)->getSNR(); }

  float packetScore(float snr, int packet_len) override {
    int sf = ((CustomSX1262 *)_radio)->spreadingFactor;
    return packetScoreInt(snr, sf, packet_len);
  }
  uint8_t getSpreadingFactor() const override { return ((CustomSX1262 *)_radio)->spreadingFactor; }
  virtual void powerOff() override {
    ((CustomSX1262 *)_radio)->sleep(false);
  }

  bool setRxBoostedGainMode(bool en) override {
    return ((CustomSX1262 *)_radio)->setRxBoostedGainMode(en) == RADIOLIB_ERR_NONE;
  }
  bool getRxBoostedGainMode() const override {
    return ((CustomSX1262 *)_radio)->getRxBoostedGainMode();
  }

  void doResetAGC() override { sx126xResetAGC((SX126x *)_radio, getRxBoostedGainMode()); }
};

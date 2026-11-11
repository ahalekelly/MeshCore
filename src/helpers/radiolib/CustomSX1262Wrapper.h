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
    } else if (strcmp(command, "radiodiag3") == 0) {
      int len = formatDiag3(out, 150);
      radio->formatDiag3(out + len, 150 - len);
    } else if (strncmp(command, "radiostorm ", 11) == 0) {
      debugStorm(strtoul(command + 11, NULL, 10));
      snprintf(out, 150, "OK");
    } else if (strncmp(command, "radiotx ", 8) == 0) {
      unsigned khz, ms;
      int dbm;
      if (sscanf(command + 8, "%u %d %u", &khz, &dbm, &ms) != 3 || khz < 902000 || khz > 928000 || dbm < -9 || dbm > 10 || ms < 1 || ms > 5000) {
        snprintf(out, 150, "ERROR: radiotx khz(902000..928000) dbm(-9..10) ms(1..5000)");
        return true;
      }
      int16_t result = radio->debugCarrier(khz / 1000.0f, dbm, ms);
      debugIdle();
      snprintf(out, 150, "result=%d", result);
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
      else if (strcmp(action, "gain") == 0) {
        uint8_t gain = 0;
        radio->readRegister(RADIOLIB_SX126X_REG_RX_GAIN, &gain, 1);
        radio->writeRegister(RADIOLIB_SX126X_REG_RX_GAIN, &gain, 1);
      } else if (strcmp(action, "fs") == 0) {
        radio->setFs();
        radio->setRx(RADIOLIB_SX126X_RX_TIMEOUT_INF);
      } else if (strcmp(action, "rx") == 0) idle();
      else if (strcmp(action, "sleep") == 0) {
        radio->sleep(true);
        radio->standby(RADIOLIB_SX126X_STANDBY_RC, true);
        debugIdle();
      } else if (strcmp(action, "cal") == 0) {
        // sx126xResetAGC without the warm sleep
        radio->standby(RADIOLIB_SX126X_STANDBY_RC, true);
        uint8_t calData = RADIOLIB_SX126X_CALIBRATE_ALL;
        radio->getMod()->SPIwriteStream(RADIOLIB_SX126X_CMD_CALIBRATE, &calData, 1, true, false);
        delay(5);
        uint32_t start = millis();
        while (digitalRead(radio->getMod()->getGpio()) && millis() - start < 50) yield();
        radio->calibrateImage(radio->freqMHz);
        delay(50);
        radio->setDio2AsRfSwitch(true);
        radio->setRxBoostedGainMode(getRxBoostedGainMode());
        debugIdle();
      } else if (strcmp(action, "agc") == 0) debugResetAGC();
      else { snprintf(out, 150, "ERROR: radiofix clr|gain|fs|rx|sleep|cal|agc"); return true; }
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
    } else if (strncmp(command, "radiowreg ", 10) == 0) {
      unsigned addr, val;
      if (sscanf(command + 10, "%x %x", &addr, &val) != 2 || addr > 0xFFFF || val > 0xFF) {
        snprintf(out, 150, "ERROR: radiowreg addr_hex val_hex");
        return true;
      }
      uint8_t before = 0, v = val, after = 0;
      radio->readRegister(addr, &before, 1);
      radio->writeRegister(addr, &v, 1);
      radio->readRegister(addr, &after, 1);
      snprintf(out, 150, "%04X: %02X -> %02X", addr, before, after);
    } else if (strncmp(command, "radiotrace ", 11) == 0) {
      // Waits for a preamble, then samples IRQ flags and RSSI 32 times at the given interval.
      unsigned us, timeout;
      if (sscanf(command + 11, "%u %u", &us, &timeout) != 2 || us > 50000 || timeout > 20000) {
        snprintf(out, 150, "ERROR: radiotrace interval_us timeout_ms");
        return true;
      }
      uint32_t started = millis();
      while (!(radio->getIrqFlags() & RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED) && millis() - started < timeout) delay(1);
      int len = snprintf(out, 150, "w=%lu ", (unsigned long)(millis() - started));
      for (int i = 0; i < 32; i++) {
        uint8_t r = 0;
        radio->getMod()->SPIreadStream(RADIOLIB_SX126X_CMD_GET_RSSI_INST, &r, 1);
        len += snprintf(out + len, 150 - len, "%02X%02X", (unsigned)(radio->getIrqFlags() & 0xFF), r);
        delayMicroseconds(us);
      }
    } else if (strncmp(command, "radiotcxo ", 10) == 0) {
      unsigned mv, delay;
      if (sscanf(command + 10, "%u %u", &mv, &delay) != 2 || delay > 100000) {
        snprintf(out, 150, "ERROR: radiotcxo mv delay_us");
        return true;
      }
      int16_t result = radio->setTCXO(mv / 1000.0f, delay);
      debugIdle();
      snprintf(out, 150, "result=%d", result);
    } else if (strncmp(command, "radiostress ", 12) == 0) {
      // Restarts the receiver n times. delay/mv reconfigure the TCXO for the run, gap adds TCXO-off time before each
      // restart, fast sends SetRx straight after SetStandby, and hold stops at the first failure and leaves it in place.
      unsigned n = 0, delay = 0, mv = 0, gap = 0, dwell = 0, fast = 0, hold = 1;
      for (const char* p = command + 12; *p; ) {
        while (*p == ' ') p++;
        if (!*p) break;
        const char* eq = strchr(p, '=');
        if (!eq) { n = 0; break; }
        char* end;
        unsigned val = strtoul(eq + 1, &end, 10);
        size_t klen = eq - p;
        auto is = [&](const char* k) { return strlen(k) == klen && strncmp(p, k, klen) == 0; };
        if (is("n")) n = val;
        else if (is("delay")) delay = val;
        else if (is("mv")) mv = val;
        else if (is("gap")) gap = val;
        else if (is("dwell")) dwell = val;
        else if (is("fast")) fast = val;
        else if (is("hold")) hold = val;
        else { n = 0; break; }
        p = end;
      }
      if (n < 1 || n > 2000 || delay > 100000 || gap > 200000 || dwell > 200000) {
        snprintf(out, 150, "ERROR: radiostress n=1..2000 [delay=us] [mv=] [gap=us] [dwell=us] [fast=1] [hold=0]");
        return true;
      }
#ifdef SX126X_DIO3_TCXO_VOLTAGE
      constexpr float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
#else
      constexpr float tcxo = 1.6f;
#endif
      if (mv || delay) {
        int16_t result = radio->setTCXO(mv ? mv / 1000.0f : tcxo, delay ? delay : 5000);
        if (result != RADIOLIB_ERR_NONE) {
          debugIdle();
          snprintf(out, 150, "ERROR: setTCXO %d", result);
          return true;
        }
      }
      auto rssi = [&]() { uint8_t r = 0; radio->getMod()->SPIreadStream(RADIOLIB_SX126X_CMD_GET_RSSI_INST, &r, 1); return r; };
      unsigned errors = 0, notrx = 0, derr = 0, stale = 0, done = 0, badat = 0, staleat = 0;
      uint8_t badst = 0, stalerssi = 0;
      uint16_t derror = 0;
      int16_t last = 0;
      uint32_t tmin = UINT32_MAX, tmax = 0, started = millis();
      bool held = false;
      for (; done < n && !held; done++) {
        uint32_t t0 = micros();
        if (gap || fast) {
          radio->standby(RADIOLIB_SX126X_STANDBY_RC);
          if (gap) delayMicroseconds(gap);
        }
        int16_t result = fast ? radio->setRx(RADIOLIB_SX126X_RX_TIMEOUT_INF) : radio->startReceive();
        uint32_t t = micros() - t0;
        if (t < tmin) tmin = t;
        if (t > tmax) tmax = t;
        if (result != RADIOLIB_ERR_NONE) { errors++; last = result; }
        uint8_t status = 0;
        radio->getMod()->SPIreadStream(RADIOLIB_SX126X_CMD_GET_STATUS, &status, 1);
        bool bad = ((status >> 4) & 7) != 5;
        if (bad) {
          if (notrx == 0) { badst = status; badat = done; }
          notrx++;
        }
        uint16_t dev = radio->getDeviceErrors();
        if (dev) {
          derr++;
          derror |= dev;
          if (!hold) radio->clearDeviceErrors();
        }
        if (dwell) delayMicroseconds(dwell);
        // The random-number registers change on every read while the receive chain runs.
        uint8_t a[4], b[4];
        radio->readRegister(0x0819, a, 4);
        delayMicroseconds(300);
        radio->readRegister(0x0819, b, 4);
        bool same = memcmp(a, b, 4) == 0;
        if (same) {
          if (stale == 0) { stalerssi = rssi(); staleat = done; }
          stale++;
        }
        held = hold && (bad || same || dev);
        yield();
      }
      if (held) {
        debugAssumeRx();
      } else {
        if (mv || delay) radio->setTCXO(tcxo, 5000);
        debugIdle();
      }
      snprintf(out, 150, "n=%u err=%u lasterr=%d notrx=%u bad=%02X@%u derr=%u derror=%04X stale=%u rssi=%02X@%u held=%d usmin=%lu usmax=%lu ms=%lu",
        done, errors, last, notrx, badst, badat, derr, derror, stale, stalerssi, staleat, held,
        (unsigned long)tmin, (unsigned long)tmax, (unsigned long)(millis() - started));
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


#define RADIOLIB_STATIC_ONLY 1
#include "RadioLibWrappers.h"

#define STATE_IDLE       0
#define STATE_RX         1
#define STATE_TX_WAIT    3
#define STATE_TX_DONE    4
#define STATE_INT_READY 16

#define NUM_NOISE_FLOOR_SAMPLES  64
#define SAMPLING_THRESHOLD  14

static volatile uint8_t state = STATE_IDLE;
static volatile uint32_t n_isr = 0;
static uint32_t n_rx_starts = 0, n_rx_fails = 0;
static int last_rx_err = 0;
static uint32_t storm_left = 0, storm_done = 0, storm_until = 0, storm_next = 0;
// Sampler starvation: time spent on rejected noise samples (RSSI at or above floor + threshold) since the last accepted one.
static uint32_t starve_ms = 0, starve_max = 0, n_starve2 = 0, n_starve5 = 0, n_starve10 = 0;
static uint32_t n_acc = 0, n_rej = 0, n_gated = 0, watch_ms = 0, n_watch = 0, loop_last = 0, loop_gap_max = 0;
static int rej_max = -200;
// RSSI survey: one reading every loop pass in receive, including the passes the noise sampler skips.
static const int SURVEY_DBM[] = {-100, -90, -80, -70, -60, -50, -40};
static bool survey_on = false;
static uint32_t survey_hist[7], n_survey = 0, n_survey_off = 0, n_loops = 0;
static int survey_max = -200;
// Bursts: runs of survey readings at or above BURST_DBM, which is stronger than any mesh packet this radio decodes.
#define BURST_DBM  -85
#define NUM_BURSTS  6
struct Burst { uint32_t start; uint16_t dur, n; int16_t peak; bool after_gap; };
static Burst bursts[NUM_BURSTS], burst;
static uint32_t n_bursts = 0, burst_ms = 0;

static void endBurst(uint32_t now) {
  burst.dur = now - burst.start;
  bursts[n_bursts++ % NUM_BURSTS] = burst;
  burst_ms += burst.dur;
  burst.n = 0;
}

static void endStarve() {
  if (starve_ms > starve_max) starve_max = starve_ms;
  if (starve_ms >= 2000) n_starve2++;
  if (starve_ms >= 5000) n_starve5++;
  if (starve_ms >= 10000) n_starve10++;
  starve_ms = 0;
}

// this function is called when a complete packet
// is transmitted by the module
static
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we sent a packet, set the flag
  state |= STATE_INT_READY;
  n_isr++;
}

void RadioLibWrapper::begin() {
  _radio->setPacketReceivedAction(setFlag);  // this is also SentComplete interrupt
  _preamble_sf = getSpreadingFactor();
  _radio->setPreambleLength(preambleLengthForSF(_preamble_sf)); // longer preamble for lower SF improves reliability
  state = STATE_IDLE;

  if (_board->getStartupReason() == BD_STARTUP_RX_PACKET) {  // received a LoRa packet (while in deep sleep)
    setFlag(); // LoRa packet is already received
  }

  _noise_floor = 0;
  _threshold = 0;
  _cad_enabled = false;

  // start average out some samples
  _num_floor_samples = 0;
  _floor_sample_sum = 0;
}

void RadioLibWrapper::formatDiag(char* out) {
  int n = sprintf(out, "state=%d floor=%d samples=%d sum=%ld rssi=%d recv=%lu rerr=%lu", state, _noise_floor, _num_floor_samples,
                  (long)_floor_sample_sum, (int)getCurrentRSSI(), (unsigned long)n_recv, (unsigned long)n_recv_errors);
  formatChipDiag(out + n);
}

uint32_t RadioLibWrapper::getRngSeed() {
  return _radio->random(0x7FFFFFFF);
}

void RadioLibWrapper::setTxPower(int8_t dbm) {
#if defined(USE_LR2021)
  idle();
#endif
  _radio->setOutputPower(dbm);
}

void RadioLibWrapper::idle() {
  _radio->standby();
  state = STATE_IDLE;   // need another startReceive()
}

void RadioLibWrapper::debugIdle() { state = STATE_IDLE; }
int RadioLibWrapper::formatDiag3(char* out, size_t size) {
  return snprintf(out, size, "isr=%lu rxs=%lu rxf=%lu rxe=%d storm=%lu/%lu", (unsigned long)n_isr, (unsigned long)n_rx_starts,
                  (unsigned long)n_rx_fails, last_rx_err, (unsigned long)storm_done, (unsigned long)storm_left);
}
void RadioLibWrapper::debugStorm(uint32_t n) {
  storm_left = n;
  storm_until = 0;
}
void RadioLibWrapper::debugWatch(uint32_t ms) { watch_ms = ms; }
int RadioLibWrapper::formatDiag4(char* out, size_t size, bool reset) {
  int len = snprintf(out, size, "fl=%d acc=%lu rej=%lu gat=%lu cur=%lu max=%lu s2=%lu s5=%lu s10=%lu rmax=%d gap=%lu rx=%lu re=%lu w=%lu/%lu",
                     _noise_floor, (unsigned long)n_acc, (unsigned long)n_rej, (unsigned long)n_gated,
                     (unsigned long)starve_ms, (unsigned long)starve_max,
                     (unsigned long)n_starve2, (unsigned long)n_starve5, (unsigned long)n_starve10, rej_max,
                     (unsigned long)loop_gap_max, (unsigned long)n_recv, (unsigned long)n_recv_errors,
                     (unsigned long)watch_ms, (unsigned long)n_watch);
  if (reset) {
    starve_max = n_starve2 = n_starve5 = n_starve10 = n_acc = n_rej = n_gated = loop_gap_max = 0;
    rej_max = -200;
  }
  return len;
}
void RadioLibWrapper::debugSurvey(bool on) { survey_on = on; }
int RadioLibWrapper::formatDiag5(char* out, size_t size, bool reset) {
  int len = snprintf(out, size, "on=%d lp=%lu n=%lu off=%lu smax=%d h=%lu,%lu,%lu,%lu,%lu,%lu,%lu", (int)survey_on,
                     (unsigned long)n_loops, (unsigned long)n_survey, (unsigned long)n_survey_off, survey_max,
                     (unsigned long)survey_hist[0], (unsigned long)survey_hist[1], (unsigned long)survey_hist[2],
                     (unsigned long)survey_hist[3], (unsigned long)survey_hist[4], (unsigned long)survey_hist[5],
                     (unsigned long)survey_hist[6]);
  if (reset) {
    memset(survey_hist, 0, sizeof(survey_hist));
    n_loops = n_survey = n_survey_off = 0;
    survey_max = -200;
  }
  return len;
}
int RadioLibWrapper::formatDiag6(char* out, size_t size, bool reset) {
  int len = snprintf(out, size, "now=%lu nb=%lu tot=%lu", (unsigned long)(millis() % 1000000), (unsigned long)n_bursts, (unsigned long)burst_ms);
  uint32_t count = n_bursts < NUM_BURSTS ? n_bursts : NUM_BURSTS;
  for (uint32_t i = 0; i < count; i++) {
    const Burst& b = bursts[(n_bursts - count + i) % NUM_BURSTS];
    len += snprintf(out + len, size - len, " %lu,%u,%u,%d%s", (unsigned long)(b.start % 1000000), b.dur, b.n, b.peak, b.after_gap ? "g" : "");
  }
  if (reset) n_bursts = burst_ms = 0;
  return len;
}
void RadioLibWrapper::debugAssumeRx() { state = STATE_RX; }

void RadioLibWrapper::debugResetAGC() {
  doResetAGC();
  state = STATE_IDLE;
  _noise_floor = 0;
  _num_floor_samples = 0;
  _floor_sample_sum = 0;
}

void RadioLibWrapper::triggerNoiseFloorCalibrate(int threshold) {
  _threshold = threshold;
  if (_num_floor_samples >= NUM_NOISE_FLOOR_SAMPLES) {  // ignore trigger if currently sampling
    _num_floor_samples = 0;
    _floor_sample_sum = 0;
  }
}

void RadioLibWrapper::doResetAGC() {
  _radio->sleep();  // warm sleep to reset analog frontend
}

void RadioLibWrapper::resetAGC() {
  // make sure we're not mid-receive of packet!
  if ((state & STATE_INT_READY) != 0 || isReceivingPacket()) return;

  doResetAGC();
  state = STATE_IDLE;   // trigger a startReceive()

  // Reset noise floor sampling so it reconverges from scratch.
  // Without this, a stuck _noise_floor of -120 makes the sampling threshold
  // too low (-106) to accept normal samples (~-105), self-reinforcing the
  // stuck value even after the receiver has recovered.
  _noise_floor = 0;
  _num_floor_samples = 0;
  _floor_sample_sum = 0;
}

void RadioLibWrapper::loop() {
  uint32_t loop_now = millis();
  uint32_t loop_dt = loop_now - loop_last;
  if (loop_dt > loop_gap_max) loop_gap_max = loop_dt;
  loop_last = loop_now;
  n_loops++;
  if (survey_on && state == STATE_RX) {
    int rssi = getCurrentRSSI();
    n_survey++;
    if (rssi <= -127) n_survey_off++;
    if (rssi > survey_max) survey_max = rssi;
    for (int i = 0; i < 7 && rssi >= SURVEY_DBM[i]; i++) survey_hist[i]++;
    if (rssi >= BURST_DBM) {
      if (!burst.n) burst = {loop_now, 0, 0, (int16_t)rssi, loop_dt > 20};
      burst.n++;
      if (rssi > burst.peak) burst.peak = rssi;
    } else if (burst.n) {
      endBurst(loop_now);
    }
  } else if (burst.n) {
    endBurst(loop_now);
  }

  // Debug storm: in alternate 10 s windows, restart receive every 15 ms for 300 ms after a packet starts arriving,
  // so the receiver re-enters RX while a LoRa signal is on the air.
  if (storm_left && (millis() / 10000) % 2 == 0) {
    uint32_t now = millis();
    if (storm_until == 0) {
      if (state == STATE_RX && isReceivingPacket()) {
        storm_until = now + 300;
        storm_next = now + 5;
      }
    } else if (now >= storm_until) {
      storm_until = 0;
    } else if (now >= storm_next && state == STATE_RX) {
      idle();   // recvRaw() restarts receive on this pass
      storm_next = now + 15;
      storm_left--;
      storm_done++;
    }
  }

  if (state == STATE_RX && _num_floor_samples < NUM_NOISE_FLOOR_SAMPLES) {
    if (!isReceivingPacket()) {
      int rssi = getCurrentRSSI();
      if (rssi < _noise_floor + SAMPLING_THRESHOLD) {  // only consider samples below current floor + sampling THRESHOLD
        _num_floor_samples++;
        _floor_sample_sum += rssi;
        n_acc++;
        endStarve();
      } else {
        n_rej++;
        starve_ms += loop_dt < 50 ? loop_dt : 50;   // a stalled main loop is not sampler starvation
        if (rssi > rej_max) rej_max = rssi;
        if (watch_ms && starve_ms >= watch_ms) {
          n_watch++;
          endStarve();
          resetAGC();
        }
      }
    } else {
      n_gated++;
    }
  } else if (_num_floor_samples >= NUM_NOISE_FLOOR_SAMPLES && _floor_sample_sum != 0) {
    _noise_floor = _floor_sample_sum / NUM_NOISE_FLOOR_SAMPLES;
    if (_noise_floor < -120) {
      _noise_floor = -120;    // clamp to lower bound of -120dBi
    }
    _floor_sample_sum = 0;

    #ifdef MESH_DEBUG_NOISE_FLOOR
    MESH_DEBUG_PRINTLN("RadioLibWrapper: noise_floor = %d", (int)_noise_floor);
    #endif
  }
}

void RadioLibWrapper::startRecv() {
  #if defined(USE_LR2021)
  _radio->standby(); // without this LR2021 can throw -706 when calling startReceive after hardware CAD when side detectors are enabled
  #endif
  int err = _radio->startReceive();
  n_rx_starts++;
  if (err == RADIOLIB_ERR_NONE) {
    state = STATE_RX;
  } else {
    n_rx_fails++;
    last_rx_err = err;
    MESH_DEBUG_PRINTLN("RadioLibWrapper: error: startReceive(%d)", err);
  }
}

bool RadioLibWrapper::isInRecvMode() const {
  return (state & ~STATE_INT_READY) == STATE_RX;
}

int RadioLibWrapper::recvRaw(uint8_t* bytes, int sz) {
  int len = 0;
  if (state & STATE_INT_READY) {
    len = _radio->getPacketLength();
    if (len > 0) {
      if (len > sz) { len = sz; }
      int err = _radio->readData(bytes, len);
      if (err != RADIOLIB_ERR_NONE) {
        MESH_DEBUG_PRINTLN("RadioLibWrapper: error: readData(%d)", err);
        len = 0;
        n_recv_errors++;
      } else {
      //  Serial.print("  readData() -> "); Serial.println(len);
        n_recv++;
      }
    }
    #if defined(USE_LR2021)
    state = STATE_RX;     // LR2021 stays in Rx after readData, calling startReceive while still in Rx throws -706 errors
    #else
    state = STATE_IDLE;   // need another startReceive()
    #endif
  }

  if (state != STATE_RX) {
    int err = _radio->startReceive();
    n_rx_starts++;
    if (err == RADIOLIB_ERR_NONE) {
      state = STATE_RX;
    } else {
      n_rx_fails++;
      last_rx_err = err;
      MESH_DEBUG_PRINTLN("RadioLibWrapper: error: startReceive(%d)", err);
    }
  }
  return len;
}

uint32_t RadioLibWrapper::getEstAirtimeFor(int len_bytes) {
  return _radio->getTimeOnAir(len_bytes) / 1000;
}

bool RadioLibWrapper::startSendRaw(const uint8_t* bytes, int len) {
  _board->onBeforeTransmit();
  int err = _radio->startTransmit((uint8_t *) bytes, len);
  if (err == RADIOLIB_ERR_NONE) {
    state = STATE_TX_WAIT;
    return true;
  }
  MESH_DEBUG_PRINTLN("RadioLibWrapper: error: startTransmit(%d)", err);
  idle();   // trigger another startRecv()
  _board->onAfterTransmit();
  return false;
}

bool RadioLibWrapper::isSendComplete() {
  if (state & STATE_INT_READY) {
    state = STATE_IDLE;
    n_sent++;
    return true;
  }
  return false;
}

void RadioLibWrapper::onSendFinished() {
  _radio->finishTransmit();
  _board->onAfterTransmit();
  state = STATE_IDLE;
}

int16_t RadioLibWrapper::performChannelScan() {
  return _radio->scanChannel();
}

bool RadioLibWrapper::isChannelActive() {
  // int.thresh: RSSI-based interference detection (relative to noise floor)
  if (_threshold != 0 && getCurrentRSSI() > _noise_floor + _threshold) return true;

  // cad: hardware channel activity detection
  if (_cad_enabled) {
    int16_t result = performChannelScan();
    // scanChannel() triggers DIO interrupt (CAD done) which sets STATE_INT_READY
    // via setFlag() ISR. Clear it before restarting RX so recvRaw() doesn't
    // try to read a non-existent packet and count a spurious recv error.
    state = STATE_IDLE;
    startRecv();
    if (result != RADIOLIB_CHANNEL_FREE) return true;
  }

  return false;
}

float RadioLibWrapper::getLastRSSI() const {
  return _radio->getRSSI();
}
float RadioLibWrapper::getLastSNR() const {
  return _radio->getSNR();
}

// Approximate SNR threshold per SF for successful reception (based on Semtech datasheets)
static float snr_threshold[] = {
    -7.5,  // SF7 needs at least -7.5 dB SNR
    -10,   // SF8 needs at least -10 dB SNR
    -12.5, // SF9 needs at least -12.5 dB SNR
    -15,  // SF10 needs at least -15 dB SNR
    -17.5,// SF11 needs at least -17.5 dB SNR
    -20   // SF12 needs at least -20 dB SNR
};

float RadioLibWrapper::packetScoreInt(float snr, int sf, int packet_len) {
  if (sf < 7) return 0.0f;

  if (snr < snr_threshold[sf - 7]) return 0.0f;    // Below threshold, no chance of success

  auto success_rate_based_on_snr = (snr - snr_threshold[sf - 7]) / 10.0;
  auto collision_penalty = 1 - (packet_len / 256.0);   // Assuming max packet of 256 bytes

  return max(0.0, min(1.0, success_rate_based_on_snr * collision_penalty));
}

PacketMillis RadioLibWrapper::calcMaxPacketMillis(uint8_t sf, float bw, uint8_t cr, uint8_t preambleSymbols) {
  // based on RadioLib's calculateTimeOnAir()
  uint32_t tsym_us = ((uint32_t)10000 << sf) / (bw * 10);
  uint32_t sfCoeff1_x4 = (sf == 5 || sf == 6) ? 25 : 17; // 6.25 : 4.25, semtech magic numbers to account for sync word + sfd

  // preamble + syncword + sfd + header
  uint32_t preamble_us = (((preambleSymbols + 8) * 4 + sfCoeff1_x4) * tsym_us) / 4;

  // airtime for max packet at current radio settings
  uint32_t total_us   = _radio->getTimeOnAir(MAX_TRANS_UNIT);
  // airtime for payload only (no preamble, header or SOF)
  uint32_t payload_us = total_us > preamble_us ? total_us - preamble_us : 4000 - preamble_us; // fallback to 4 secs at worst case
  // rescale payload_us for max possible CR
  if (cr >= 5 && cr < 8) { payload_us = (payload_us * 8) / cr; }

  return PacketMillis {(preamble_us + 999) / 1000, (payload_us + 999) / 1000};
}

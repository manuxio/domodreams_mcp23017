/*
Compact comments kept near each block; behavior unchanged from prior iteration, with:
- sensors list optional, optional pin mapping
- per-pin overrides via pins: and/or sensors:
- words configurable per instance
- on_fsm_change trigger with (pin, name, state, prev_state, time_ms, unixtime)
- immediate "single" when long_min==0 && double_max_delay==0
- off_delay==0 publishes "off" immediately (no timer)
*/

#include "domodreams_mcp23017.h"
#include "esphome/core/log.h"

#if !defined(ARDUINO)
extern "C" void esp_restart(void);
#endif

namespace esphome {
namespace domodreams_mcp23017 {

void DomodreamsMCP23017::setup() {
  ESP_LOGI(TAG, "Setting up Domodreams MCP23017 at 0x%02X", this->address_);

  bool ok = true;
  ok &= this->writeReg(REG_IODIRA, 0xFF);    // inputs
  ok &= this->writeReg(REG_IODIRB, 0xFF);
  ok &= this->writeReg(REG_GPPUA,  0xFF);    // pull-ups
  ok &= this->writeReg(REG_GPPUB,  0xFF);
  ok &= this->writeReg(REG_IPOLA,  0xFF);    // inverted
  ok &= this->writeReg(REG_IPOLB,  0xFF);
  ok &= this->writeReg(REG_GPINTENA, 0x00);  // interrupts disabled
  ok &= this->writeReg(REG_GPINTENB, 0x00);

  if (!ok) {
    fail_reason_ = str_sprintf("Init failed at 0x%02X", this->address_);
    ESP_LOGE(TAG, "%s", fail_reason_.c_str());
    // Fire once we have a real reason, after triggers are registered
    this->publishAllOff_();
    this->fireFailure_(fail_reason_, this->ioFailStreak_);
    this->mark_failed(fail_reason_.c_str());
    return;
  }

  ESP_LOGI(TAG,
           "MCP23017 0x%02X initialized | debounce=%ums longMin=%ums doubleMaxDelay=%ums "
           "offDelay=%ums releaseOffDelay=%ums rebootOnFail=%s",
           this->address_, debounceMs_, longMinMs_, doubleMaxDelayMs_,
           offDelayMs_, releaseOffDelayMs_, rebootOnFail_ ? "true" : "false");

  initDone_ = true;
  this->publishAllOff_();
  this->dump_config();

}

void DomodreamsMCP23017::dump_config() {
  ESP_LOGCONFIG(TAG, "Domodreams MCP23017");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Debounce: %u ms", (unsigned) debounceMs_);
  ESP_LOGCONFIG(TAG, "  longMin: %u ms", (unsigned) longMinMs_);
  ESP_LOGCONFIG(TAG, "  doubleMaxDelay: %u ms", (unsigned) doubleMaxDelayMs_);
  ESP_LOGCONFIG(TAG, "  offDelay: %u ms", (unsigned) offDelayMs_);
  ESP_LOGCONFIG(TAG, "  releaseOffDelay: %u ms", (unsigned) releaseOffDelayMs_);
  ESP_LOGCONFIG(TAG, "  Reboot on fail: %s", rebootOnFail_ ? "true" : "false");
  for (int i = 0; i < 16; i++) {
    if (sensors_[i] != nullptr) {
      ESP_LOGCONFIG(TAG, "  Sensor %02d: %s", i, sensors_[i]->get_name().c_str());
    } else if (hasPinAutoName_[i]) {
      ESP_LOGCONFIG(TAG, "  Headless pin %02d (auto-name: %s)", i, pinAutoName_[i].c_str());
    } else {
      ESP_LOGCONFIG(TAG, "  Headless pin %02d", i);
    }
  }
  if (!fail_reason_.empty()) {
    ESP_LOGCONFIG(TAG, "  Last failure: %s", fail_reason_.c_str());
  }
}

void DomodreamsMCP23017::update() {
  if (!initDone_) return;

  uint16_t word = 0;
  if (!this->readGpio(word)) {
    ESP_LOGW(TAG, "MCP23017 0x%02X read failed", this->address_);
    this->handleIoFail_();
    return;
  }
  this->ioFailStreak_ = 0;

  const uint32_t now = millis();
  this->runDebounce_(word, now);
  this->evaluateAndPublish_();
}

// ---------- I2C helpers ----------

bool DomodreamsMCP23017::writeReg(uint8_t reg, uint8_t val) {
  return this->write_register(reg, &val, 1) == i2c::ERROR_OK;
}

bool DomodreamsMCP23017::readReg(uint8_t reg, uint8_t *val) {
  return this->read_register(reg, val, 1) == i2c::ERROR_OK;
}

bool DomodreamsMCP23017::readGpio(uint16_t &word) {
  uint8_t a = 0, b = 0;
  bool okA = this->readReg(REG_GPIOA, &a);
  bool okB = this->readReg(REG_GPIOB, &b);
  if (!(okA && okB)) return false;
  word = (uint16_t(b) << 8) | a;
  return true;
}

// ---------- Debounce ----------

void DomodreamsMCP23017::runDebounce_(uint16_t word, uint32_t now) {
  for (int i = 0; i < 16; i++) {
    const bool raw = (word >> i) & 0x1;

    if (raw != candState_[i]) {
      candState_[i] = raw;
      candSince_[i] = now;
      continue;
    }

    const uint32_t held = now - candSince_[i];
    if (held < debounceMs_) continue;

    if (!stableValid_[i]) {
      stableValid_[i] = true;
      stableState_[i] = candState_[i];
      changedMask_ |= (1u << i);
      continue;
    }

    if (stableState_[i] != candState_[i]) {
      stableState_[i] = candState_[i];
      changedMask_ |= (1u << i);
    }
  }
}

// ---------- FSM + publish ----------

void DomodreamsMCP23017::evaluateAndPublish_() {
  const uint32_t now = millis();

  if (changedMask_ == 0) {
    for (int i = 0; i < 16; i++) this->fsmOnTick_(i, now);
    return;
  }

  for (int i = 0; i < 16; i++) {
    if ((changedMask_ & (1u << i)) == 0) continue;
    const bool newLevel = stableState_[i];
    bool havePrev = fsmPrevValid_[i];
    bool prev = fsmPrevLevel_[i];
    if (!havePrev || newLevel != prev) {
      this->fsmOnEdge_(i, newLevel, now);
      fsmPrevValid_[i] = true;
      fsmPrevLevel_[i] = newLevel;
    }
  }

  for (int i = 0; i < 16; i++) this->fsmOnTick_(i, now);

  changedMask_ = 0;
}

void DomodreamsMCP23017::cancelOff_(int i) {
  tOffDeadline_[i] = 0;
}

void DomodreamsMCP23017::scheduleOff_(int i, uint32_t delayMs) {
  if (delayMs == 0) {
    // immediate publish (no timer)
    this->publishPin_(i, wordOff_.c_str());
    fsmState_[i] = PinFSM::IDLE;
    tOffDeadline_[i] = 0;
  } else {
    tOffDeadline_[i] = millis() + delayMs;
    fsmState_[i] = PinFSM::POST_DELAY_OFF;
  }
}

void DomodreamsMCP23017::fsmOnEdge_(int i, bool newLevel, uint32_t now) {
  switch (fsmState_[i]) {
    case PinFSM::IDLE:
      if (newLevel) {
        // If single is immediate (long_min==0 && double_max_delay==0): publish now
        if (effLongMin_(i) == 0 && effDoubleMaxDelay_(i) == 0) {
          this->publishPin_(i, wordSingle_.c_str());
          // schedule off using offDelay (may be 0 → immediate off)
          this->scheduleOff_(i, effOffDelay_(i));
          // remain in POST_DELAY_OFF or IDLE depending on off_delay
          return;
        }
        cancelOff_(i);
        fsmState_[i] = PinFSM::PRESSING;
        tPressStart_[i] = now;
        publishedLong_[i] = false;
      }
      break;

    case PinFSM::PRESSING:
      if (!newLevel) {
        const uint32_t dur = now - tPressStart_[i];
        if (dur < effLongMin_(i)) {
          if (effDoubleMaxDelay_(i) == 0) {
            // double disabled → single immediately on release
            this->publishPin_(i, wordSingle_.c_str());
            this->scheduleOff_(i, effOffDelay_(i));
            fsmState_[i] = PinFSM::POST_DELAY_OFF;
          } else {
            tWindowDeadline_[i] = now + effDoubleMaxDelay_(i);
            fsmState_[i] = PinFSM::WAIT_DOUBLE_WINDOW;
          }
        } else {
          // defensive path; normally LONG_HELD tick would have fired
          this->publishPin_(i, wordReleased_.c_str());
          this->scheduleOff_(i, releaseOffDelayMs_);
        }
      }
      break;

    case PinFSM::WAIT_DOUBLE_WINDOW:
      if (newLevel) {
        this->publishPin_(i, wordDouble_.c_str());
        cancelOff_(i);
        fsmState_[i] = PinFSM::DOUBLE_WAIT_RELEASE;
      }
      break;

    case PinFSM::LONG_HELD:
      if (!newLevel) {
        this->publishPin_(i, wordReleased_.c_str());
        this->scheduleOff_(i, releaseOffDelayMs_);
      }
      break;

    case PinFSM::DOUBLE_WAIT_RELEASE:
      if (!newLevel) {
        this->scheduleOff_(i, effOffDelay_(i));
      }
      break;

    case PinFSM::POST_DELAY_OFF:
      if (newLevel) {
        cancelOff_(i);
        fsmState_[i] = PinFSM::PRESSING;
        tPressStart_[i] = now;
        publishedLong_[i] = false;
      }
      break;
  }
}

void DomodreamsMCP23017::fsmOnTick_(int i, uint32_t now) {
  switch (fsmState_[i]) {
    case PinFSM::PRESSING: {
      const uint32_t held = now - tPressStart_[i];
      if (!publishedLong_[i] && held >= effLongMin_(i)) {
        this->publishPin_(i, wordLong_.c_str());
        publishedLong_[i] = true;
        fsmState_[i] = PinFSM::LONG_HELD;
      }
      break;
    }

    case PinFSM::WAIT_DOUBLE_WINDOW:
      if (now >= tWindowDeadline_[i]) {
        this->publishPin_(i, wordSingle_.c_str());
        this->scheduleOff_(i, effOffDelay_(i));
      }
      break;

    case PinFSM::DOUBLE_WAIT_RELEASE:
      // wait release
      break;

    case PinFSM::POST_DELAY_OFF:
      if (tOffDeadline_[i] != 0 && now >= tOffDeadline_[i]) {
        this->publishPin_(i, wordOff_.c_str());
        tOffDeadline_[i] = 0;
        fsmState_[i] = PinFSM::IDLE;
      }
      break;

    case PinFSM::LONG_HELD:
    case PinFSM::IDLE:
      break;
  }
}

void DomodreamsMCP23017::publishPin_(int i, const char *state) {
  // suppress repeated publishes
  if (lastPublishedState_[i] == state) return;

  // INFO log once per change
  ESP_LOGI(TAG, "Mcp at 0x%02X - pin %d - state change: %s", this->address_, i, state);

  // save prev for trigger
  const std::string prev = lastPublishedState_[i];
  lastPublishedState_[i] = state;

  // publish to text sensor if present
  if (sensors_[i] != nullptr)
    sensors_[i]->publish_state(state);

  // mirrors (button + time)
  if (!(state == wordOff_)) {
    this->publishLastTriggered_(i, millis());
  }

  // trigger automations
  if (!fsm_triggers_.empty()) {
    std::string name;
    if (sensors_[i] != nullptr) {
      name = sensors_[i]->get_name().c_str();
    } else if (hasPinAutoName_[i]) {
      name = pinAutoName_[i];
    } else {
      name = "";
    }
    const uint32_t t_ms = millis();
    const uint64_t ut = this->current_unixtime_();
    for (auto *t : fsm_triggers_) {
      t->trigger(i, name, std::string(state), prev, t_ms, ut);
    }
  }
}

void DomodreamsMCP23017::publishLastTriggered_(int i, uint32_t now) {
  if (lastTriggeredButton_ != nullptr) {
    if (sensors_[i] != nullptr)
      lastTriggeredButton_->publish_state(sensors_[i]->get_name().c_str());
    else if (hasPinAutoName_[i])
      lastTriggeredButton_->publish_state(pinAutoName_[i]);
  }
  if (lastTriggeredTime_ != nullptr) {
    if (rtc_ != nullptr) {
      auto t = rtc_->now();
      if (t.is_valid()) {
        std::string iso = t.strftime("%Y-%m-%dT%H:%M:%SZ");
        lastTriggeredTime_->publish_state(iso);
        return;
      }
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", (unsigned) now);
    lastTriggeredTime_->publish_state(buf);
  }
}

uint64_t DomodreamsMCP23017::current_unixtime_() const {
  if (rtc_ != nullptr) {
    auto t = rtc_->now();
    if (t.is_valid()) {
      return static_cast<uint64_t>(t.timestamp);
    }
  }
  // fallback: seconds since boot (not real UNIX time, but monotonic)
  return static_cast<uint64_t>(millis() / 1000UL);
}

// ---------- Guard (I2C fail after setup) ----------

void DomodreamsMCP23017::handleIoFail_() {
  if (!this->initDone_) return;

  if (this->ioFailStreak_ < 255)
    this->ioFailStreak_++;

  if (this->ioFailStreak_ >= 3) {
    fail_reason_ = str_sprintf("I/O failed at 0x%02X", this->address_);
    ESP_LOGE(TAG, "%s", fail_reason_.c_str());
    // Fire on_failure before publishing Off/mark_failed
    this->fireFailure_(fail_reason_, this->ioFailStreak_);
    this->publishAllOff_();
    this->mark_failed(fail_reason_.c_str());
    this->initDone_ = false;

    if (this->rebootOnFail_) {
      ESP_LOGE(TAG, "Scheduling reboot in 1s due to I/O failures (0x%02X)", this->address_);
      this->set_timeout(1000, []() {
        #ifdef ARDUINO
          ESP.restart();
        #else
          esp_restart();
        #endif
      });
    }
  }
}


void DomodreamsMCP23017::fireFailure_(const std::string &reason, uint8_t streak) {
  if (!this->failure_triggers_.empty()) {
    ESP_LOGE(TAG, "Running error trigger!");
  }
  for (auto *t : this->failure_triggers_) {
    t->trigger(reason, streak);
  }
}
void DomodreamsMCP23017::publishAllOff_() {
  for (int i = 0; i < 16; i++) this->publishPin_(i, wordOff_.c_str());
}

}  // namespace domodreams_mcp23017
}  // namespace esphome

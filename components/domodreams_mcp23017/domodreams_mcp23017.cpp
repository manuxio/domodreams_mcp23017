/*
Logic overview (what the code does)

setup():
Initializes the MCP23017 (all pins = inputs, pull-ups ON, polarity inverted, interrupts OFF).
If any I²C write fails, it marks the component failed with a reason and exits.
It also publishes “off” to all 16 sensors at boot so HA never sees “unknown”.

dump_config():
Prints the current configuration (timings, address, sensor names) and the last failure
reason if one exists.

update():
Runs every update_interval. It reads both GPIO ports; on failure, increments a failure
streak and after 3 consecutive failures marks the component failed (and optionally reboots).
On success, it runs the debounce stage and then the FSM + publish stage.

Debounce (runDebounce):
Per pin, a candidate level must be stable for debounceMs before it becomes the “stable” level.
When a stable level changes, a bit in changedMask is set to feed the FSM.

FSM (evaluateAndPublish, fsmOnEdge, fsmOnTick):
Detects edges from the debounced stream and transitions per-pin states.
Implements gesture logic:

- long when held ≥ longMinMs, then on release released, then off after releaseOffDelay.
- single if released before longMinMs and no second press within doubleMaxDelay.
- double if a second press starts within doubleMaxDelay.

Uses timers to schedule delayed off publications (offDelay / releaseOffDelay).

publishPin():
Publishes the text state to the corresponding text sensor and updates the optional
“last triggered” sensors (button name + wall-clock time if available).

I²C guard (handleIoFail):
After 3 consecutive GPIO read failures, publish off for all pins, mark failed with
a persistent reason, stop updates, and optionally reboot after 1s if reboot_on_fail is true.

This keeps states clean for HA, provides explicit failure reasons, and cleanly separates
debounce → FSM → publish for maintainability and future extensions.
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
  this->publishAllOff_();

  bool ok = true;
  ok &= this->writeReg(REG_IODIRA, 0xFF);
  ok &= this->writeReg(REG_IODIRB, 0xFF);
  ok &= this->writeReg(REG_GPPUA,  0xFF);
  ok &= this->writeReg(REG_GPPUB,  0xFF);
  ok &= this->writeReg(REG_IPOLA,  0xFF);
  ok &= this->writeReg(REG_IPOLB,  0xFF);
  ok &= this->writeReg(REG_GPINTENA, 0x00);
  ok &= this->writeReg(REG_GPINTENB, 0x00);

  if (!ok) {
    fail_reason_ = str_sprintf("Init failed at 0x%02X", this->address_);
    ESP_LOGE(TAG, "%s", fail_reason_.c_str());
    this->mark_failed(fail_reason_.c_str());
    return;
  }

  ESP_LOGI(TAG,
           "MCP23017 0x%02X initialized | debounce=%ums longMin=%ums doubleMaxDelay=%ums "
           "offDelay=%ums releaseOffDelay=%ums rebootOnFail=%s",
           this->address_, debounceMs, longMinMs, doubleMaxDelayMs,
           offDelayMs, releaseOffDelayMs, rebootOnFail ? "true" : "false");

  initDone = true;
  this->dump_config();
}

void DomodreamsMCP23017::dump_config() {
  ESP_LOGCONFIG(TAG, "Domodreams MCP23017");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Debounce: %u ms", (unsigned) debounceMs);
  ESP_LOGCONFIG(TAG, "  longMin: %u ms", (unsigned) longMinMs);
  ESP_LOGCONFIG(TAG, "  doubleMaxDelay: %u ms", (unsigned) doubleMaxDelayMs);
  ESP_LOGCONFIG(TAG, "  offDelay: %u ms", (unsigned) offDelayMs);
  ESP_LOGCONFIG(TAG, "  releaseOffDelay: %u ms", (unsigned) releaseOffDelayMs);
  ESP_LOGCONFIG(TAG, "  Reboot on fail: %s", rebootOnFail ? "true" : "false");
  for (int i = 0; i < 16; i++) {
    if (sensors[i] != nullptr) {
      const char *last = lastPublishedState[i].empty() ? "(none)" : lastPublishedState[i].c_str();
      ESP_LOGCONFIG(TAG, "  Sensor %d: %s | last: %s",
                    i, sensors[i]->get_name().c_str(), last);
    }
  }
  if (!fail_reason_.empty()) {
    ESP_LOGCONFIG(TAG, "  Last failure: %s", fail_reason_.c_str());
  }
}

void DomodreamsMCP23017::update() {
  if (!initDone) return;

  uint16_t word = 0;
  if (!this->readGpio(word)) {
    ESP_LOGW(TAG, "MCP23017 0x%02X read failed", this->address_);
    this->handleIoFail();
    return;
  }
  this->ioFailStreak = 0;

  const uint32_t now = millis();
  this->runDebounce(word, now);
  this->evaluateAndPublish();
}

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

void DomodreamsMCP23017::runDebounce(uint16_t word, uint32_t now) {
  for (int i = 0; i < 16; i++) {
    const bool raw = (word >> i) & 0x1;

    if (raw != candState[i]) {
      candState[i] = raw;
      candSince[i] = now;
      continue;
    }

    const uint32_t held = now - candSince[i];
    if (held < debounceMs) continue;

    if (!stableValid[i]) {
      stableValid[i] = true;
      stableState[i] = candState[i];
      changedMask |= (1u << i);
      continue;
    }

    if (stableState[i] != candState[i]) {
      stableState[i] = candState[i];
      changedMask |= (1u << i);
    }
  }
}

void DomodreamsMCP23017::evaluateAndPublish() {
  const uint32_t now = millis();

  if (changedMask == 0) {
    for (int i = 0; i < 16; i++) this->fsmOnTick(i, now);
    return;
  }

  for (int i = 0; i < 16; i++) {
    if ((changedMask & (1u << i)) == 0) continue;
    const bool newLevel = stableState[i];
    bool havePrev = fsmPrevValid[i];
    bool prev = fsmPrevLevel[i];
    if (!havePrev || newLevel != prev) {
      this->fsmOnEdge(i, newLevel, now);
      fsmPrevValid[i] = true;
      fsmPrevLevel[i] = newLevel;
    }
  }

  for (int i = 0; i < 16; i++) this->fsmOnTick(i, now);

  changedMask = 0;
}

void DomodreamsMCP23017::cancelOff(int i) {
  tOffDeadline[i] = 0;
}

void DomodreamsMCP23017::scheduleOff(int i, uint32_t delayMs) {
  if (delayMs == 0) {
    publishPin(i, wordOff.c_str());
    tOffDeadline[i] = 0;
    fsmState[i] = PinFSM::IDLE;
    return;
  }
  tOffDeadline[i] = millis() + delayMs;
  fsmState[i] = PinFSM::POST_DELAY_OFF;
}

void DomodreamsMCP23017::fsmOnEdge(int i, bool newLevel, uint32_t now) {
  switch (fsmState[i]) {
    case PinFSM::IDLE:
      if (newLevel) {
        cancelOff(i);
        fsmState[i] = PinFSM::PRESSING;
        tPressStart[i] = now;
        publishedLong[i] = false;
        if (effLongMin(i) == 0 && effDoubleDelay(i) == 0) {
          publishPin(i, wordSingle.c_str());
          scheduleOff(i, effOffDelay(i));
        }
      }
      break;

    case PinFSM::PRESSING:
      if (!newLevel) {
        const uint32_t dur = now - tPressStart[i];
        const uint32_t lm = effLongMin(i);
        if (lm > 0 && dur < lm) {
          if (effDoubleDelay(i) == 0) {
            publishPin(i, wordSingle.c_str());
            scheduleOff(i, effOffDelay(i));
            fsmState[i] = PinFSM::POST_DELAY_OFF;
          } else {
            tRelease[i] = now;
            tWindowDeadline[i] = now + effDoubleDelay(i);
            fsmState[i] = PinFSM::WAIT_DOUBLE_WINDOW;
          }
        } else if (lm == 0) {
          // instant mode already handled on press; nothing on release
        } else {
          publishPin(i, wordReleased.c_str());
          scheduleOff(i, releaseOffDelayMs);
        }
      }
      break;

    case PinFSM::WAIT_DOUBLE_WINDOW:
      if (newLevel) {
        publishPin(i, wordDouble.c_str());
        cancelOff(i);
        tPressStart[i] = now;
        fsmState[i] = PinFSM::DOUBLE_WAIT_RELEASE;
      }
      break;

    case PinFSM::LONG_HELD:
      if (!newLevel) {
        publishPin(i, wordReleased.c_str());
        scheduleOff(i, releaseOffDelayMs);
      }
      break;

    case PinFSM::DOUBLE_WAIT_RELEASE:
      if (!newLevel) {
        scheduleOff(i, effOffDelay(i));
      }
      break;

    case PinFSM::POST_DELAY_OFF:
      if (newLevel) {
        cancelOff(i);
        fsmState[i] = PinFSM::PRESSING;
        tPressStart[i] = now;
        publishedLong[i] = false;
      }
      break;
  }
}

void DomodreamsMCP23017::fsmOnTick(int i, uint32_t now) {
  switch (fsmState[i]) {
    case PinFSM::PRESSING: {
      const uint32_t lm = effLongMin(i);
      if (lm > 0) {
        const uint32_t held = now - tPressStart[i];
        if (!publishedLong[i] && held >= lm) {
          publishPin(i, wordLong.c_str());
          publishedLong[i] = true;
          fsmState[i] = PinFSM::LONG_HELD;
        }
      }
      break;
    }
    case PinFSM::WAIT_DOUBLE_WINDOW:
      if (now >= tWindowDeadline[i]) {
        publishPin(i, wordSingle.c_str());
        scheduleOff(i, effOffDelay(i));
      }
      break;
    case PinFSM::DOUBLE_WAIT_RELEASE:
      break;
    case PinFSM::POST_DELAY_OFF:
      if (tOffDeadline[i] != 0 && now >= tOffDeadline[i]) {
        publishPin(i, wordOff.c_str());
        tOffDeadline[i] = 0;
        fsmState[i] = PinFSM::IDLE;
      }
      break;
    case PinFSM::LONG_HELD:
    case PinFSM::IDLE:
      break;
  }
}

void DomodreamsMCP23017::publishPin(int i, const char *state) {
  auto *ts = sensors[i];
  if (ts == nullptr) return;
  if (lastPublishedState[i] == state) return;
  ESP_LOGI(TAG, "Mcp at 0x%02X - pin %d - state change: %s", this->address_, i, state);
  ts->publish_state(state);
  lastPublishedState[i] = state;
  if (std::string(state) != wordOff) {
    const uint32_t now = millis();
    this->publishLastTriggered_(i, state, now);
  }
}

void DomodreamsMCP23017::publishLastTriggered_(int i, const char * /*state*/, uint32_t now) {
  if (lastTriggeredButton != nullptr) {
    if (sensors[i] != nullptr)
      lastTriggeredButton->publish_state(sensors[i]->get_name().c_str());
  }
  if (lastTriggeredTime != nullptr) {
    if (rtc != nullptr) {
      auto t = rtc->now();
      if (t.is_valid()) {
        std::string iso = t.strftime("%Y-%m-%dT%H:%M:%SZ");
        lastTriggeredTime->publish_state(iso);
        return;
      }
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", (unsigned) now);
    lastTriggeredTime->publish_state(buf);
  }
}

void DomodreamsMCP23017::handleIoFail() {
  if (!this->initDone) return;
  if (this->ioFailStreak < 255) this->ioFailStreak++;
  if (this->ioFailStreak >= 3) {
    fail_reason_ = str_sprintf("I/O failed at 0x%02X", this->address_);
    ESP_LOGE(TAG, "%s", fail_reason_.c_str());
    this->publishAllOff_();
    this->mark_failed(fail_reason_.c_str());
    this->initDone = false;
    if (this->rebootOnFail) {
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

void DomodreamsMCP23017::publishAllOff_() {
  for (int i = 0; i < 16; i++) this->publishPin(i, wordOff.c_str());
}

}  // namespace domodreams_mcp23017
}  // namespace esphome

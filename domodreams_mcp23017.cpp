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

long when held ≥ longMinMs, then on release released, then off after releaseOffDelay.

single if released before longMinMs and no second press within doubleMaxDelay.

double if a second press starts within doubleMaxDelay.

Uses timers to schedule delayed off publications (offDelay / releaseOffDelay).

publishPin():
Publishes the text state to the corresponding text sensor and updates the optional
“last triggered” sensors (button name + wall-clock time if available).
Now gated: we only publish/log when the state actually changes.

I²C guard (handleIoFail):
After 3 consecutive GPIO read failures, publish off for all pins, mark failed with
a persistent reason, stop updates, and optionally reboot after 1s if reboot_on_fail is true.

*/

#include "domodreams_mcp23017.h"         // Component class & register defs
#include "esphome/core/log.h"            // ESPHome logging macros

#if !defined(ARDUINO)
extern "C" void esp_restart(void);       // Fallback restart for non-Arduino builds
#endif

namespace esphome {
namespace domodreams_mcp23017 {

void DomodreamsMCP23017::setup() {
  ESP_LOGI(TAG, "Setting up MCP23017 at 0x%02X", this->address_);

  // Ensure HA doesn't see 'unknown': publish "off" for all pins immediately.
  this->publishAllOff_();

  bool ok = true;
  ok &= this->writeReg(REG_IODIRA, 0xFF);   // Set port A as inputs
  ok &= this->writeReg(REG_IODIRB, 0xFF);   // Set port B as inputs
  ok &= this->writeReg(REG_GPPUA,  0xFF);   // Enable pull-ups on port A
  ok &= this->writeReg(REG_GPPUB,  0xFF);   // Enable pull-ups on port B
  ok &= this->writeReg(REG_IPOLA,  0xFF);   // Invert port A (active-low buttons read as high)
  ok &= this->writeReg(REG_IPOLB,  0xFF);   // Invert port B (same reasoning)
  ok &= this->writeReg(REG_GPINTENA, 0x00); // Disable interrupts on port A (we poll instead)
  ok &= this->writeReg(REG_GPINTENB, 0x00); // Disable interrupts on port B

  // If any I2C write failed, mark the component failed with a clear reason.
  if (!ok) {
    fail_reason_ = str_sprintf("Init failed at 0x%02X", this->address_);
    ESP_LOGE(TAG, "%s", fail_reason_.c_str());
    this->mark_failed(fail_reason_.c_str());
    return;
  }

  // Log finalized configuration for visibility during bring-up/tuning.
  ESP_LOGI(TAG, "MCP23017 0x%02X initialized | debounce=%ums longMin=%ums doubleMaxDelay=%ums offDelay=%ums releaseOffDelay=%ums rebootOnFail=%s",
           this->address_, debounceMs, longMinMs, doubleMaxDelayMs, offDelayMs, releaseOffDelayMs,
           rebootOnFail ? "true" : "false");

  initDone = true; // Setup succeeded; enable updates.
}

void DomodreamsMCP23017::dump_config() {
  // Print static config (address & timings) and sensor names.
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
  // If there was a previous failure, show the last reason to aid diagnostics.
  if (!fail_reason_.empty()) {
    ESP_LOGCONFIG(TAG, "  Last failure: %s", fail_reason_.c_str());
  }
}

void DomodreamsMCP23017::update() {
  if (!initDone) return;  // Skip if setup failed or we’re marked failed.

  uint16_t word = 0;
  // Read both GPIO ports; on failure, handle streak/mark failed logic.
  if (!this->readGpio(word)) {
    ESP_LOGW(TAG, "MCP23017 0x%02X read failed", this->address_);
    this->handleIoFail();
    return;
  }
  this->ioFailStreak = 0; // Clear failure streak on success.

  const uint32_t now = millis();
  this->runDebounce(word, now);  // Stage 1: per-pin debounce to stable edges
  this->evaluateAndPublish();    // Stage 2/3: FSM + publish text states
}

// ---------- I2C helpers ----------

bool DomodreamsMCP23017::writeReg(uint8_t reg, uint8_t val) {
  // Write a single register; return true on success.
  return this->write_register(reg, &val, 1) == i2c::ERROR_OK;
}

bool DomodreamsMCP23017::readReg(uint8_t reg, uint8_t *val) {
  // Read a single register; return true on success.
  return this->read_register(reg, val, 1) == i2c::ERROR_OK;
}

bool DomodreamsMCP23017::readGpio(uint16_t &word) {
  // Fetch GPIOA and GPIOB bytes back-to-back.
  uint8_t a = 0, b = 0;
  bool okA = this->readReg(REG_GPIOA, &a);
  bool okB = this->readReg(REG_GPIOB, &b);
  if (!(okA && okB)) return false;            // Propagate failure upward.
  word = (uint16_t(b) << 8) | a;              // Pack into 16-bit bitfield (B:15..8, A:7..0)
  return true;
}

// ---------- Debounce ----------

void DomodreamsMCP23017::runDebounce(uint16_t word, uint32_t now) {
  // Simple per-pin debounce: candidate state must hold for debounceMs to become stable.
  for (int i = 0; i < 16; i++) {
    const bool raw = (word >> i) & 0x1;       // Raw bit from MCP (already inverted in setup)

    if (raw != candState[i]) {                // Candidate changed → reset timer
      candState[i] = raw;
      candSince[i] = now;
      continue;
    }

    const uint32_t held = now - candSince[i]; // How long candidate has been steady
    if (held < debounceMs) continue;          // Not stable long enough yet

    if (!stableValid[i]) {                    // First time stabilizing this pin
      stableValid[i] = true;
      stableState[i] = candState[i];
      changedMask |= (1u << i);               // Flag a change for FSM
      continue;
    }

    if (stableState[i] != candState[i]) {     // Existing stable vs new stable differs
      stableState[i] = candState[i];
      changedMask |= (1u << i);               // Flag change for FSM
    }
  }
}

// ---------- FSM + publish ----------

void DomodreamsMCP23017::evaluateAndPublish() {
  const uint32_t now = millis();

  // No edges? still run per-pin timers (long/timeout/off scheduling).
  if (changedMask == 0) {
    for (int i = 0; i < 16; i++) this->fsmOnTick(i, now);
    return;
  }

  // Process all pins that reported a debounced edge this cycle.
  for (int i = 0; i < 16; i++) {
    if ((changedMask & (1u << i)) == 0) continue; // Skip unchanged
    const bool newLevel = stableState[i];         // Debounced logic level
    bool havePrev = fsmPrevValid[i];
    bool prev = fsmPrevLevel[i];
    if (!havePrev || newLevel != prev) {          // Rising or falling edge
      this->fsmOnEdge(i, newLevel, now);          // Edge-driven FSM transition
      fsmPrevValid[i] = true;                     // Store last level for next edge detection
      fsmPrevLevel[i] = newLevel;
    }
  }

  // Advance timers for all pins (double window, long hold, delayed off).
  for (int i = 0; i < 16; i++) this->fsmOnTick(i, now);

  changedMask = 0; // Clear changed bits for next update cycle.
}

void DomodreamsMCP23017::cancelOff(int i) {
  tOffDeadline[i] = 0; // Cancel any scheduled "off" for this pin.
}

void DomodreamsMCP23017::scheduleOff(int i, uint32_t delayMs) {
  tOffDeadline[i] = millis() + delayMs;  // Schedule a future "off" publish.
  fsmState[i] = PinFSM::POST_DELAY_OFF;  // Enter waiting-for-off state.
}

void DomodreamsMCP23017::fsmOnEdge(int i, bool newLevel, uint32_t now) {
  switch (fsmState[i]) {
    case PinFSM::IDLE:
      if (newLevel) {
        cancelOff(i);                 // If any off was pending, cancel it.
        fsmState[i] = PinFSM::PRESSING;
        tPressStart[i] = now;         // Start measuring press duration
        publishedLong[i] = false;     // Reset long flag for this press
      }
      break;

    case PinFSM::PRESSING:
      if (!newLevel) {
        // First press ended before being long? → candidate for single/double.
        const uint32_t dur = now - tPressStart[i];
        const uint32_t pinLong = effLongMin(i);

        if (dur < pinLong) {
          const uint32_t dbl = effDoubleDelay(i);
          if (dbl == 0) {
            // Double disabled: publish single immediately, then schedule off.
            publishPin(i, "single");
            scheduleOff(i, offDelayMs);
            fsmState[i] = PinFSM::POST_DELAY_OFF;
          } else {
            // Wait for an optional second press within the double window.
            tRelease[i] = now;
            tWindowDeadline[i] = now + dbl;
            fsmState[i] = PinFSM::WAIT_DOUBLE_WINDOW;
          }
        } else {
          // Defensive path (normally handled in tick when longMinMs elapses).
          publishPin(i, "released");
          scheduleOff(i, releaseOffDelayMs);
        }
      }
      break;

    case PinFSM::WAIT_DOUBLE_WINDOW:
      if (newLevel) {
        // Second press started within double window.
        publishPin(i, "double");
        cancelOff(i);                 // No off until release
        tPressStart[i] = now;         // Track the second press timing
        fsmState[i] = PinFSM::DOUBLE_WAIT_RELEASE;  // Wait for release to schedule off
      }
      break;

    case PinFSM::LONG_HELD:
      if (!newLevel) {
        // Long was published earlier; on release publish "released", then off.
        publishPin(i, "released");
        scheduleOff(i, releaseOffDelayMs);
      }
      break;

    case PinFSM::DOUBLE_WAIT_RELEASE:
      if (!newLevel) {
        // After double, schedule off once the press ends.
        scheduleOff(i, offDelayMs);
      }
      break;

    case PinFSM::POST_DELAY_OFF:
      if (newLevel) {
        // New press started while waiting to go "off": cancel off and press again.
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
      // While pressed, check if we’ve crossed long threshold (per-pin or global).
      const uint32_t held = now - tPressStart[i];
      const uint32_t pinLong = effLongMin(i);
      if (!publishedLong[i] && held >= pinLong) {
        publishPin(i, "long");        // One-time "long" gesture
        publishedLong[i] = true;
        fsmState[i] = PinFSM::LONG_HELD; // Wait for release to publish "released"
      }
      break;
    }

    case PinFSM::WAIT_DOUBLE_WINDOW:
      // No second press in time → finalize "single".
      if (now >= tWindowDeadline[i]) {
        publishPin(i, "single");
        scheduleOff(i, offDelayMs);
      }
      break;

    case PinFSM::DOUBLE_WAIT_RELEASE:
      // No timer work here; release handled in edge.
      break;

    case PinFSM::POST_DELAY_OFF:
      // When deadline hits, publish "off" and return to IDLE.
      if (tOffDeadline[i] != 0 && now >= tOffDeadline[i]) {
        publishPin(i, "off");
        tOffDeadline[i] = 0;
        fsmState[i] = PinFSM::IDLE;
      }
      break;

    case PinFSM::LONG_HELD:
    case PinFSM::IDLE:
      // Nothing to do on tick for these states.
      break;
  }
}

void DomodreamsMCP23017::publishPin(int i, const char *state) {
  auto *ts = sensors[i];
  if (ts == nullptr) return;           // Skip if sensor not configured

  // Prevent redundant publish/logging if state is same as previous
  if (lastPublishedState[i] == state) return;

  ESP_LOGI(TAG, "Mcp at 0x%02X - pin %d - state change: %s", this->address_, i, state);
  ts->publish_state(state);            // Push state to HA
  lastPublishedState[i] = state;       // Track last published state

  // Update "last triggered" metadata for gestures (skip "off")
  if (!(strcmp(state, "off") == 0)) {
    const uint32_t now = millis();
    this->publishLastTriggered_(i, state, now);
  }
}

void DomodreamsMCP23017::publishLastTriggered_(int i, const char * /*state*/, uint32_t now) {
  // Publish which button (sensor name) caused the last gesture.
  if (lastTriggeredButton != nullptr) {
    if (sensors[i] != nullptr)
      lastTriggeredButton->publish_state(sensors[i]->get_name().c_str());
  }
  // Publish when it happened: prefer wall-clock time if available, else ms since boot.
  if (lastTriggeredTime != nullptr) {
    if (rtc != nullptr) {
      auto t = rtc->now();
      if (t.is_valid()) {
        std::string iso = t.strftime("%Y-%m-%dT%H:%M:%SZ"); // ISO 8601 UTC
        lastTriggeredTime->publish_state(iso);
        return;
      }
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", (unsigned) now);
    lastTriggeredTime->publish_state(buf);
  }
}

// ---------- Guard (I2C fail after setup) ----------

void DomodreamsMCP23017::handleIoFail() {
  if (!this->initDone) return;     // Ignore if we’re not active

  if (this->ioFailStreak < 255)
    this->ioFailStreak++;          // Increment consecutive failure counter

  // After 3 consecutive read failures, fail the component and (optionally) reboot.
  if (this->ioFailStreak >= 3) {
    fail_reason_ = str_sprintf("I/O failed at 0x%02X", this->address_);
    ESP_LOGE(TAG, "%s", fail_reason_.c_str());
    this->publishAllOff_();                    // Force all text sensors to "off"
    this->mark_failed(fail_reason_.c_str());   // Mark failed with persistent reason
    this->initDone = false;                    // Stop further updates

    if (this->rebootOnFail) {
      ESP_LOGE(TAG, "Scheduling reboot in 1s due to I/O failures (0x%02X)", this->address_);
      this->set_timeout(1000, []() {
        #ifdef ARDUINO
          ESP.restart();                       // Arduino restart path
        #else
          esp_restart();                       // ESP-IDF restart path
        #endif
      });
    }
  }
}

void DomodreamsMCP23017::publishAllOff_() {
  // Helper to set all 16 text sensors to "off".
  for (int i = 0; i < 16; i++) this->publishPin(i, "off");
}

}  // namespace domodreams_mcp23017
}  // namespace esphome

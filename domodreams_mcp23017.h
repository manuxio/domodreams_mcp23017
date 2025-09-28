#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"  // for str_sprintf
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"
#include <string>

namespace esphome {
namespace domodreams_mcp23017 {

static const char *const TAG = "domodreams_mcp23017";

// MCP23017 registers
static const uint8_t REG_IODIRA   = 0x00;
static const uint8_t REG_IODIRB   = 0x01;
static const uint8_t REG_IPOLA    = 0x02;
static const uint8_t REG_IPOLB    = 0x03;
static const uint8_t REG_GPINTENA = 0x04;
static const uint8_t REG_GPINTENB = 0x05;
static const uint8_t REG_GPPUA    = 0x0C;
static const uint8_t REG_GPPUB    = 0x0D;
static const uint8_t REG_GPIOA    = 0x12;
static const uint8_t REG_GPIOB    = 0x13;

class DomodreamsMCP23017 : public PollingComponent, public i2c::I2CDevice {
 public:
  DomodreamsMCP23017() : PollingComponent(10) {}

  void setup() override;
  void dump_config() override;
  void update() override;

  // setters from codegen
  void setTextSensor(int pin, text_sensor::TextSensor *ts) { sensors[pin] = ts; }
  void setDebounce(uint32_t v) { debounceMs = v; }
  void setLongMin(uint32_t v) { longMinMs = v; }
  void setDoubleMaxDelay(uint32_t v) { doubleMaxDelayMs = v; }
  void setOffDelay(uint32_t v) { offDelayMs = v; }
  void setReleaseOffDelay(uint32_t v) { releaseOffDelayMs = v; }
  void setRebootOnFail(bool v) { rebootOnFail = v; }
  void setLastTriggered(text_sensor::TextSensor *btn, text_sensor::TextSensor *ts) {
    lastTriggeredButton = btn;
    lastTriggeredTime = ts;
  }
  void setTime(time::RealTimeClock *t) { rtc = t; }

 protected:
  // I2C helpers
  bool writeReg(uint8_t reg, uint8_t val);
  bool readReg(uint8_t reg, uint8_t *val);
  bool readGpio(uint16_t &word);

  // stage 1: debounce
  void runDebounce(uint16_t word, uint32_t now);

  // stage 2+3: FSM + publish
  void evaluateAndPublish();
  void publishPin(int i, const char *state);
  void publishAllOff_();
  void publishLastTriggered_(int i, const char *state, uint32_t now);

  // FSM (simplified spec)
  enum class PinFSM : uint8_t {
    IDLE,
    PRESSING,            // first press in progress
    WAIT_DOUBLE_WINDOW,  // first short released; waiting for second to start
    LONG_HELD,           // long recognized; waiting for release
    DOUBLE_WAIT_RELEASE, // double published; wait release to schedule off
    POST_DELAY_OFF       // 'off' scheduled
  };

  void fsmOnEdge(int i, bool newLevel, uint32_t now);
  void fsmOnTick(int i, uint32_t now);
  void scheduleOff(int i, uint32_t delayMs);
  void cancelOff(int i);

  // FSM state storage
  PinFSM  fsmState[16] = {PinFSM::IDLE};
  bool    fsmPrevValid[16] = {false};
  bool    fsmPrevLevel[16] = {false};

  // timing
  uint32_t tPressStart[16] = {0};
  uint32_t tRelease[16] = {0};
  uint32_t tWindowDeadline[16] = {0};
  uint32_t tOffDeadline[16] = {0};

  // flags
  bool publishedLong[16] = {false};

  // sensors
  text_sensor::TextSensor *sensors[16] = {nullptr};
  text_sensor::TextSensor *lastTriggeredButton{nullptr};
  text_sensor::TextSensor *lastTriggeredTime{nullptr};

  // optional time source
  time::RealTimeClock *rtc{nullptr};

  // debounce state
  bool candState[16] = {false};
  uint32_t candSince[16] = {0};
  bool stableState[16] = {false};
  bool stableValid[16] = {false};
  uint16_t changedMask{0};

  // configuration
  uint32_t debounceMs{50};
  uint32_t longMinMs{1000};
  uint32_t doubleMaxDelayMs{300};
  uint32_t offDelayMs{100};
  uint32_t releaseOffDelayMs{1000};
  bool rebootOnFail{false};

  // lifecycle & guard
  bool initDone{false};
  uint8_t ioFailStreak{0};
  void handleIoFail();

  // persistent failure reason (avoid dangling c_str)
  std::string fail_reason_;
};

}  // namespace domodreams_mcp23017
}  // namespace esphome

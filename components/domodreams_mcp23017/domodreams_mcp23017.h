#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"   // str_sprintf
#include "esphome/core/automation.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"
#include <string>
#include <vector>

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

// Forward declare trigger
class FSMChangeTrigger : public Trigger<int, std::string, std::string, std::string, uint32_t, uint64_t> {};

class DomodreamsMCP23017 : public PollingComponent, public i2c::I2CDevice {
 public:
  DomodreamsMCP23017() : PollingComponent(10) {}

  void setup() override;
  void dump_config() override;
  void update() override;

  // setters from codegen
  void setTextSensor(int pin, text_sensor::TextSensor *ts) { sensors_[pin] = ts; }
  void setDebounce(uint32_t v) { debounceMs_ = v; }
  void setLongMin(uint32_t v) { longMinMs_ = v; }
  void setDoubleMaxDelay(uint32_t v) { doubleMaxDelayMs_ = v; }
  void setOffDelay(uint32_t v) { offDelayMs_ = v; }
  void setReleaseOffDelay(uint32_t v) { releaseOffDelayMs_ = v; }
  void setRebootOnFail(bool v) { rebootOnFail_ = v; }

  void setLastTriggered(text_sensor::TextSensor *btn, text_sensor::TextSensor *ts) {
    lastTriggeredButton_ = btn;
    lastTriggeredTime_ = ts;
  }
  void setTime(time::RealTimeClock *t) { rtc_ = t; }

  // Per-instance words
  void setWords(const std::string &wo, const std::string &ws, const std::string &wd,
                const std::string &wl, const std::string &wr) {
    wordOff_ = wo; wordSingle_ = ws; wordDouble_ = wd; wordLong_ = wl; wordReleased_ = wr;
  }

  // Per-pin overrides (may be set via pins: or sensors:)
  void setPinLongMin(int pin, uint32_t v) { pinLongMin_[pin] = v; hasPinLongMin_[pin] = true; }
  void setPinDoubleMaxDelay(int pin, uint32_t v) { pinDoubleMaxDelay_[pin] = v; hasPinDoubleMaxDelay_[pin] = true; }
  void setPinOffDelay(int pin, uint32_t v) { pinOffDelay_[pin] = v; hasPinOffDelay_[pin] = true; }
  void setPinAutoName(int pin, const std::string &n) { pinAutoName_[pin] = n; hasPinAutoName_[pin] = true; }

  // Trigger registration
  void registerOnFSMChange(FSMChangeTrigger *t) { fsm_triggers_.push_back(t); }

 protected:
  // I2C helpers
  bool writeReg(uint8_t reg, uint8_t val);
  bool readReg(uint8_t reg, uint8_t *val);
  bool readGpio(uint16_t &word);

  // stage 1: debounce
  void runDebounce_(uint16_t word, uint32_t now);

  // stage 2+3: FSM + publish
  void evaluateAndPublish_();
  void publishPin_(int i, const char *state);
  void publishAllOff_();
  void publishLastTriggered_(int i, uint32_t now);
  uint64_t current_unixtime_() const;

  // FSM (simplified spec)
  enum class PinFSM : uint8_t {
    IDLE,
    PRESSING,            // first press in progress
    WAIT_DOUBLE_WINDOW,  // first short released; waiting for second to start
    LONG_HELD,           // long recognized; waiting for release
    DOUBLE_WAIT_RELEASE, // double published; wait release to schedule off
    POST_DELAY_OFF       // 'off' scheduled
  };

  void fsmOnEdge_(int i, bool newLevel, uint32_t now);
  void fsmOnTick_(int i, uint32_t now);
  void scheduleOff_(int i, uint32_t delayMs);
  void cancelOff_(int i);

  // Effective per-pin getters with precedence: global -> pins: -> sensors:
  uint32_t effLongMin_(int i) const { return hasPinLongMin_[i] ? pinLongMin_[i] : longMinMs_; }
  uint32_t effDoubleMaxDelay_(int i) const { return hasPinDoubleMaxDelay_[i] ? pinDoubleMaxDelay_[i] : doubleMaxDelayMs_; }
  uint32_t effOffDelay_(int i) const { return hasPinOffDelay_[i] ? pinOffDelay_[i] : offDelayMs_; }

  // FSM state storage
  PinFSM  fsmState_[16] = {PinFSM::IDLE};
  bool    fsmPrevValid_[16] = {false};
  bool    fsmPrevLevel_[16] = {false};

  // timing
  uint32_t tPressStart_[16] = {0};
  uint32_t tWindowDeadline_[16] = {0};
  uint32_t tOffDeadline_[16] = {0};

  // flags
  bool publishedLong_[16] = {false};

  // sensors
  text_sensor::TextSensor *sensors_[16] = {nullptr};
  text_sensor::TextSensor *lastTriggeredButton_{nullptr};
  text_sensor::TextSensor *lastTriggeredTime_{nullptr};

  // optional time source
  time::RealTimeClock *rtc_{nullptr};

  // debounce state
  bool candState_[16] = {false};
  uint32_t candSince_[16] = {0};
  bool stableState_[16] = {false};
  bool stableValid_[16] = {false};
  uint16_t changedMask_{0};

  // last published state (suppress repeats)
  std::string lastPublishedState_[16];

  // words
  std::string wordOff_{"off"};
  std::string wordSingle_{"single"};
  std::string wordDouble_{"double"};
  std::string wordLong_{"long"};
  std::string wordReleased_{"released"};

  // configuration
  uint32_t debounceMs_{50};
  uint32_t longMinMs_{1000};
  uint32_t doubleMaxDelayMs_{300};
  uint32_t offDelayMs_{100};
  uint32_t releaseOffDelayMs_{1000};
  bool rebootOnFail_{false};

  // per-pin overrides presence + values
  bool hasPinLongMin_[16] = {false};
  bool hasPinDoubleMaxDelay_[16] = {false};
  bool hasPinOffDelay_[16] = {false};
  bool hasPinAutoName_[16] = {false};
  uint32_t pinLongMin_[16] = {0};
  uint32_t pinDoubleMaxDelay_[16] = {0};
  uint32_t pinOffDelay_[16] = {0};
  std::string pinAutoName_[16];

  // lifecycle & guard
  bool initDone_{false};
  uint8_t ioFailStreak_{0};
  void handleIoFail_();

  // persistent failure reason
  std::string fail_reason_;

  // triggers
  std::vector<FSMChangeTrigger*> fsm_triggers_;
};

}  // namespace domodreams_mcp23017
}  // namespace esphome

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor, i2c, time as time_comp
from esphome.const import CONF_ID

DEPENDENCIES = ["i2c"]

domodreams_mcp23017_ns = cg.esphome_ns.namespace("domodreams_mcp23017")
DomodreamsMCP23017 = domodreams_mcp23017_ns.class_(
    "DomodreamsMCP23017", cg.PollingComponent, i2c.I2CDevice
)

CONF_SENSORS = "sensors"
CONF_DEBOUNCE = "debounce"
CONF_REBOOT_ON_FAIL = "reboot_on_fail"
CONF_LONG_MIN = "long_min"
CONF_DOUBLE_MAX_DELAY = "double_max_delay"
CONF_OFF_DELAY = "off_delay"
CONF_RELEASE_OFF_DELAY = "release_off_delay"

CONF_LAST_TRIGGERED_BUTTON = "last_triggered_button"
CONF_LAST_TRIGGERED_TIME = "last_triggered_time"
CONF_TIME_ID = "time_id"  # optional RTC source

# Instance-level customizable words
CONF_WORD_OFF = "word_off"
CONF_WORD_SINGLE = "word_single"
CONF_WORD_DOUBLE = "word_double"
CONF_WORD_LONG = "word_long"
CONF_WORD_RELEASEED = "word_released"  # spelled as requested: released

# Per-pin override keys (same names as globals)
CONF_PIN_LONG_MIN = CONF_LONG_MIN
CONF_PIN_DOUBLE_MAX_DELAY = CONF_DOUBLE_MAX_DELAY
CONF_PIN_OFF_DELAY = CONF_OFF_DELAY

# Build per-pin schema by extending the standard text_sensor schema
PIN_SCHEMA = text_sensor.text_sensor_schema().extend(
    {
        cv.Optional(CONF_PIN_LONG_MIN): cv.positive_int,
        cv.Optional(CONF_PIN_DOUBLE_MAX_DELAY): cv.positive_int,
        cv.Optional(CONF_PIN_OFF_DELAY): cv.int_range(min=0),
    }
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DomodreamsMCP23017),

            # Global timings (can be overridden per pin)
            cv.Optional(CONF_DEBOUNCE, default=50): cv.positive_int,
            cv.Optional(CONF_LONG_MIN, default=1000): cv.positive_int,
            cv.Optional(CONF_DOUBLE_MAX_DELAY, default=300): cv.positive_int,
            cv.Optional(CONF_OFF_DELAY, default=100): cv.int_range(min=0),
            cv.Optional(CONF_RELEASE_OFF_DELAY, default=1000): cv.int_range(min=0),

            cv.Optional(CONF_REBOOT_ON_FAIL, default=False): cv.boolean,

            # Optional “last triggered” outputs
            cv.Optional(CONF_LAST_TRIGGERED_BUTTON): cv.use_id(text_sensor.TextSensor),
            cv.Optional(CONF_LAST_TRIGGERED_TIME): cv.use_id(text_sensor.TextSensor),

            # Optional time source (for wall-clock timestamps)
            cv.Optional(CONF_TIME_ID): cv.use_id(time_comp.RealTimeClock),

            # Instance-level words
            cv.Optional(CONF_WORD_OFF, default="off"): cv.string,
            cv.Optional(CONF_WORD_SINGLE, default="single"): cv.string,
            cv.Optional(CONF_WORD_DOUBLE, default="double"): cv.string,
            cv.Optional(CONF_WORD_LONG, default="long"): cv.string,
            cv.Optional(CONF_WORD_RELEASEED, default="released"): cv.string,

            # Sensors (exactly 16)
            cv.Required(CONF_SENSORS): cv.All(
                cv.ensure_list(PIN_SCHEMA),
                cv.Length(min=16, max=16),
            ),
        }
    )
    .extend(i2c.i2c_device_schema(default_address=0x20))
    .extend(cv.polling_component_schema("10ms"))  # allows update_interval:
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Global timing/config
    cg.add(var.setDebounce(config[CONF_DEBOUNCE]))
    cg.add(var.setLongMin(config[CONF_LONG_MIN]))
    cg.add(var.setDoubleMaxDelay(config[CONF_DOUBLE_MAX_DELAY]))
    cg.add(var.setOffDelay(config[CONF_OFF_DELAY]))
    cg.add(var.setReleaseOffDelay(config[CONF_RELEASE_OFF_DELAY]))
    cg.add(var.setRebootOnFail(config[CONF_REBOOT_ON_FAIL]))

    # Optional “last triggered” outputs
    last_btn = None
    last_time = None
    if CONF_LAST_TRIGGERED_BUTTON in config:
        last_btn = await cg.get_variable(config[CONF_LAST_TRIGGERED_BUTTON])
    if CONF_LAST_TRIGGERED_TIME in config:
        last_time = await cg.get_variable(config[CONF_LAST_TRIGGERED_TIME])
    if last_btn is not None or last_time is not None:
        cg.add(var.setLastTriggered(last_btn, last_time))

    # Optional time source (for wall-clock timestamps)
    if CONF_TIME_ID in config:
        rtc = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.setTime(rtc))

    # Instance-level words
    cg.add(
        var.setWords(
            config[CONF_WORD_OFF],
            config[CONF_WORD_SINGLE],
            config[CONF_WORD_DOUBLE],
            config[CONF_WORD_LONG],
            config[CONF_WORD_RELEASEED],
        )
    )

    # Sensors + per-pin overrides
    for i, ts_conf in enumerate(config[CONF_SENSORS]):
        ts = await text_sensor.new_text_sensor(ts_conf)
        cg.add(var.setTextSensor(i, ts))
        if CONF_PIN_LONG_MIN in ts_conf:
            cg.add(var.setPinLongMin(i, ts_conf[CONF_PIN_LONG_MIN]))
        if CONF_PIN_DOUBLE_MAX_DELAY in ts_conf:
            cg.add(var.setPinDoubleMaxDelay(i, ts_conf[CONF_PIN_DOUBLE_MAX_DELAY]))
        if CONF_PIN_OFF_DELAY in ts_conf:
            cg.add(var.setPinOffDelay(i, ts_conf[CONF_PIN_OFF_DELAY]))

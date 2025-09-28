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

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DomodreamsMCP23017),
            cv.Optional(CONF_DEBOUNCE, default=50): cv.positive_int,
            cv.Optional(CONF_LONG_MIN, default=1000): cv.positive_int,
            cv.Optional(CONF_DOUBLE_MAX_DELAY, default=300): cv.positive_int,
            cv.Optional(CONF_OFF_DELAY, default=100): cv.int_range(min=0),
            cv.Optional(CONF_RELEASE_OFF_DELAY, default=1000): cv.int_range(min=0),
            cv.Optional(CONF_REBOOT_ON_FAIL, default=False): cv.boolean,
            cv.Optional(CONF_LAST_TRIGGERED_BUTTON): cv.use_id(text_sensor.TextSensor),
            cv.Optional(CONF_LAST_TRIGGERED_TIME): cv.use_id(text_sensor.TextSensor),
            cv.Optional(CONF_TIME_ID): cv.use_id(time_comp.RealTimeClock),
            cv.Required(CONF_SENSORS): cv.All(
                cv.ensure_list(text_sensor.text_sensor_schema()),
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

    for i, ts_conf in enumerate(config[CONF_SENSORS]):
        ts = await text_sensor.new_text_sensor(ts_conf)
        cg.add(var.setTextSensor(i, ts))

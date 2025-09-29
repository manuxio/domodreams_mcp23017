import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor, i2c, time as time_comp
from esphome.const import CONF_ID, CONF_NAME, CONF_ADDRESS, CONF_I2C_ID

DEPENDENCIES = ["i2c"]

domodreams_mcp23017_ns = cg.esphome_ns.namespace("domodreams_mcp23017")
DomodreamsMCP23017 = domodreams_mcp23017_ns.class_(
    "DomodreamsMCP23017", cg.PollingComponent, i2c.I2CDevice
)

# Top-level keys
CONF_SENSORS = "sensors"
CONF_PINS = "pins"
CONF_PIN = "pin"

# Timings / options
CONF_DEBOUNCE = "debounce"
CONF_REBOOT_ON_FAIL = "reboot_on_fail"
CONF_LONG_MIN = "long_min"
CONF_DOUBLE_MAX_DELAY = "double_max_delay"
CONF_OFF_DELAY = "off_delay"
CONF_RELEASE_OFF_DELAY = "release_off_delay"
CONF_UPDATE_INTERVAL = "update_interval"

# Autogeneration flag
CONF_GENERATE_SENSORS = "generate_sensors"

# Optional “last triggered”
CONF_LAST_TRIGGERED_BUTTON = "last_triggered_button"
CONF_LAST_TRIGGERED_TIME = "last_triggered_time"
CONF_TIME_ID = "time_id"

# Custom words
CONF_WORD_OFF = "word_off"
CONF_WORD_SINGLE = "word_single"
CONF_WORD_DOUBLE = "word_double"
CONF_WORD_LONG = "word_long"
CONF_WORD_RELEASED = "word_released"

# -----------------------------------------------------------------------------
# Per-pin override schema (can be headless or named)
# -----------------------------------------------------------------------------
PIN_OVERRIDE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_PIN): cv.int_range(min=0, max=15),
        cv.Optional(CONF_NAME): cv.string,
        cv.Optional(CONF_LONG_MIN): cv.positive_int,
        cv.Optional(CONF_DOUBLE_MAX_DELAY): cv.int_range(min=0),
        cv.Optional(CONF_OFF_DELAY): cv.int_range(min=0),
    }
)

# -----------------------------------------------------------------------------
# Sensors list schema
# NOTE: Do NOT require CONF_ID here; let text_sensor.text_sensor_schema()
#       insert a GenerateID so IDs are created during validation.
# -----------------------------------------------------------------------------
SENSORS_LIST_SCHEMA = cv.All(
    cv.ensure_list(
        text_sensor.text_sensor_schema().extend(
            {
                cv.Optional(CONF_PIN): cv.int_range(min=0, max=15),
                cv.Optional(CONF_LONG_MIN): cv.positive_int,
                cv.Optional(CONF_DOUBLE_MAX_DELAY): cv.int_range(min=0),
                cv.Optional(CONF_OFF_DELAY): cv.int_range(min=0),
            }
        )
    ),
    cv.Length(min=0, max=16),
)

# -----------------------------------------------------------------------------
# Base CONFIG_SCHEMA (before autogen expansion)
# -----------------------------------------------------------------------------
_BASE_CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DomodreamsMCP23017),

        # Timings (component defaults)
        cv.Optional(CONF_DEBOUNCE, default=50): cv.positive_int,
        cv.Optional(CONF_LONG_MIN, default=1000): cv.int_range(min=0),
        cv.Optional(CONF_DOUBLE_MAX_DELAY, default=300): cv.int_range(min=0),
        cv.Optional(CONF_OFF_DELAY, default=100): cv.int_range(min=0),
        cv.Optional(CONF_RELEASE_OFF_DELAY, default=1000): cv.int_range(min=0),

        cv.Optional(CONF_REBOOT_ON_FAIL, default=False): cv.boolean,

        # Optional “last triggered”
        cv.Optional(CONF_LAST_TRIGGERED_BUTTON): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_LAST_TRIGGERED_TIME): cv.use_id(text_sensor.TextSensor),

        # Optional time source
        cv.Optional(CONF_TIME_ID): cv.use_id(time_comp.RealTimeClock),

        # Autogenerate any missing pins
        cv.Optional(CONF_GENERATE_SENSORS, default=False): cv.boolean,

        # Optional per-pin overrides and/or explicit sensors
        cv.Optional(CONF_PINS, default=[]): cv.ensure_list(PIN_OVERRIDE_SCHEMA),
        cv.Optional(CONF_SENSORS, default=[]): SENSORS_LIST_SCHEMA,

        # Optional custom words
        cv.Optional(CONF_WORD_OFF): cv.string,
        cv.Optional(CONF_WORD_SINGLE): cv.string,
        cv.Optional(CONF_WORD_DOUBLE): cv.string,
        cv.Optional(CONF_WORD_LONG): cv.string,
        cv.Optional(CONF_WORD_RELEASED): cv.string,
    }
).extend(i2c.i2c_device_schema(default_address=0x20)).extend(cv.polling_component_schema("10ms"))

# -----------------------------------------------------------------------------
# Autogen expander (runs during validation, BEFORE to_code)
# This injects missing sensors (with names and pins) so that the normal
# text_sensor schema generates proper IDs automatically.
# -----------------------------------------------------------------------------
def _expand_autogen(cfg):
    # Work on a shallow copy to avoid side-effects
    config = dict(cfg)

    generate = config.get(CONF_GENERATE_SENSORS, False)
    if not generate:
        return config

    # Collect already assigned pins (from sensors + pins)
    assigned = set()

    # From explicit sensors
    for s in config.get(CONF_SENSORS, []):
        if CONF_PIN in s:
            assigned.add(s[CONF_PIN])

    # From pin overrides
    for p in config.get(CONF_PINS, []):
        assigned.add(p[CONF_PIN])

    # We will append new entries into sensors[] for any pin not yet represented.
    sensors_list = list(config.get(CONF_SENSORS, []))

    # Use I2C address to create per-instance unique names
    addr = config.get(CONF_ADDRESS, 0x20)

    # Prefix names with instance id (if available) to avoid collisions across instances
    inst_id = getattr(config.get(CONF_ID, None), "id", None)
    bus_id = getattr(config.get(CONF_I2C_ID, None), "id", None)
    inst_prefix = f"{inst_id} " if inst_id else (f"{bus_id} " if bus_id else f"mcp_{addr:02X} ")
    for p in range(16):
        if p in assigned:
            continue
        # Create a minimal sensor entry with a NAME and PIN.
        # text_sensor.text_sensor_schema() will add a generated ID during validation.
        sensors_list.append(
            {
                CONF_NAME: f"{inst_prefix}0x{addr:02X} MCP Pin {p}",
                CONF_PIN: p,
            }
        )
        assigned.add(p)

    config[CONF_SENSORS] = sensors_list
    return config

# Final CONFIG_SCHEMA: validate, then expand autogen, then re-validate sensors list
CONFIG_SCHEMA = cv.All(
    _BASE_CONFIG_SCHEMA,
    _expand_autogen,
    # Ensure the expanded sensors list still conforms to the sensors schema
    cv.Schema({cv.Optional(CONF_SENSORS, default=[]): SENSORS_LIST_SCHEMA}, extra=cv.ALLOW_EXTRA),
)

# -----------------------------------------------------------------------------
# Codegen
# -----------------------------------------------------------------------------
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Timings
    cg.add(var.setDebounce(config[CONF_DEBOUNCE]))
    cg.add(var.setLongMin(config[CONF_LONG_MIN]))
    cg.add(var.setDoubleMaxDelay(config[CONF_DOUBLE_MAX_DELAY]))
    cg.add(var.setOffDelay(config[CONF_OFF_DELAY]))
    cg.add(var.setReleaseOffDelay(config[CONF_RELEASE_OFF_DELAY]))
    cg.add(var.setRebootOnFail(config[CONF_REBOOT_ON_FAIL]))

    # Optional “last triggered”
    last_btn = None
    last_time = None
    if CONF_LAST_TRIGGERED_BUTTON in config:
        last_btn = await cg.get_variable(config[CONF_LAST_TRIGGERED_BUTTON])
    if CONF_LAST_TRIGGERED_TIME in config:
        last_time = await cg.get_variable(config[CONF_LAST_TRIGGERED_TIME])
    if last_btn is not None or last_time is not None:
        cg.add(var.setLastTriggered(last_btn, last_time))

    # Optional time source
    if CONF_TIME_ID in config:
        rtc = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.setTime(rtc))

    # Optional words
    wo = config.get(CONF_WORD_OFF)
    ws = config.get(CONF_WORD_SINGLE)
    wd = config.get(CONF_WORD_DOUBLE)
    wl = config.get(CONF_WORD_LONG)
    wr = config.get(CONF_WORD_RELEASED)
    if all(x is not None for x in [wo, ws, wd, wl, wr]):
        cg.add(var.setWords(wo, ws, wd, wl, wr))

    # Track assigned pins to avoid duplicates at codegen time too
    assigned = set()

    # Explicit sensors
    next_free = 0
    for s_conf in config[CONF_SENSORS]:
        if CONF_PIN in s_conf:
            pin = s_conf[CONF_PIN]
        else:
            # Fallback: assign next free pin deterministically
            while next_free in assigned and next_free < 16:
                next_free += 1
            if next_free >= 16:
                continue
            pin = next_free

        ts = await text_sensor.new_text_sensor(s_conf)
        cg.add(var.setTextSensor(pin, ts))
        assigned.add(pin)

        if CONF_LONG_MIN in s_conf:
            cg.add(var.setPinLongMin(pin, s_conf[CONF_LONG_MIN]))
        if CONF_DOUBLE_MAX_DELAY in s_conf:
            cg.add(var.setPinDoubleMaxDelay(pin, s_conf[CONF_DOUBLE_MAX_DELAY]))
        if CONF_OFF_DELAY in s_conf:
            cg.add(var.setPinOffDelay(pin, s_conf[CONF_OFF_DELAY]))

    # Per-pin overrides (headless or named)
    for pconf in config.get(CONF_PINS, []):
        pin = pconf[CONF_PIN]
        if CONF_NAME in pconf:
            cg.add(var.setPinName(pin, pconf[CONF_NAME]))
        if CONF_LONG_MIN in pconf:
            cg.add(var.setPinLongMin(pin, pconf[CONF_LONG_MIN]))
        if CONF_DOUBLE_MAX_DELAY in pconf:
            cg.add(var.setPinDoubleMaxDelay(pin, pconf[CONF_DOUBLE_MAX_DELAY]))
        if CONF_OFF_DELAY in pconf:
            cg.add(var.setPinOffDelay(pin, pconf[CONF_OFF_DELAY]))
        assigned.add(pin)

# Domodreams MCP23017 ESPHome Component

Custom ESPHome component for the **MCP23017 I/O expander**, with:

- **Debouncing** (per-pin, global default).
- **FSM gesture recognition**:  
  - `single`  
  - `double`  
  - `long`  
  - `released`  
  - `off`
- **Off scheduling** (`off_delay`, `release_off_delay`).
- **Per-pin overrides** for `long_min` and `double_max_delay`.
- **Last triggered metadata**: which pin fired and when.
- **I²C guard**: after 3 consecutive failures, marks the component failed, publishes all pins as `off`, and optionally reboots.

---

## Configuration

### Global options

```yaml
external_components:
  - source:
      type: local
      path: components
    components: [domodreams_mcp23017]

i2c:
  sda: 13
  scl: 16
  frequency: 100kHz
  id: i2c_bus_1

time:
  - platform: homeassistant
    id: homeassistant_time

text_sensor:
  - platform: template
    id: last_btn
    name: "Last Triggered Button"

  - platform: template
    id: last_time
    name: "Last Triggered Time"

  - platform: domodreams_mcp23017
    id: mcp1
    i2c_id: i2c_bus_1
    address: 0x20
    debounce: 50               # debounce in ms (default 50)
    long_min: 1000             # global long press threshold (ms)
    double_max_delay: 300      # global double click window (ms, 0 disables)
    off_delay: 100             # delay before returning to "off" after single/double
    release_off_delay: 1000    # delay before returning to "off" after release
    update_interval: 10ms      # poll interval (default 10ms)
    reboot_on_fail: false
    time_id: homeassistant_time
    last_triggered_button: last_btn
    last_triggered_time: last_time
    sensors:
      - name: MCP1 Pin 0
        id: mcp1_0
      - name: MCP1 Pin 1
      - name: MCP1 Pin 2
      - name: MCP1 Pin 3
      - name: MCP1 Pin 4
      - name: MCP1 Pin 5
      - name: MCP1 Pin 6
      - name: MCP1 Pin 7
      - name: MCP1 Pin 8
      - name: MCP1 Pin 9
      - name: MCP1 Pin 10
      - name: MCP1 Pin 11
      - name: MCP1 Pin 12
      - name: MCP1 Pin 13
      - name: MCP1 Pin 14
      - name: MCP1 Pin 15
```

---

### Per-pin overrides

Each `sensor` can override **`long_min`** and/or **`double_max_delay`**:

```yaml
    sensors:
      - name: MCP1 Pin 0
        id: mcp1_0
        long_min: 2000            # Only for this pin: long press ≥ 2000ms
        double_max_delay: 0       # Disable double click → always single
      - name: MCP1 Pin 1
        id: mcp1_1
        # uses global defaults
```

- If **per-pin value is set**, it takes precedence.  
- If not set, **global value** is used.

---

## Gesture logic

- **Press < long_min → candidate**
  - If another press begins within `double_max_delay` → `double`
  - Else → `single`
- **Press ≥ long_min → `long`**
  - On release → `released`
- **Every gesture → transitions to `off`**
  - After `off_delay` for `single`/`double`
  - After `release_off_delay` for `released`

---

## Failure behavior

- If **3 consecutive I²C reads fail**:
  - Publishes `"off"` for all pins
  - Marks the component **FAILED** with a reason (`I/O failed at 0x20`, etc.)
  - If `reboot_on_fail: true`, schedules reboot in 1s.

---

## Example automations

```yaml
automation:
  - alias: "Button 0 single press"
    trigger:
      - platform: state
        entity_id: text_sensor.mcp1_pin_0
        to: "single"
    action:
      - service: light.toggle
        target: { entity_id: light.kitchen }

  - alias: "Button 0 long press"
    trigger:
      - platform: state
        entity_id: text_sensor.mcp1_pin_0
        to: "long"
    action:
      - service: light.turn_off
        target: { entity_id: light.kitchen }
```

---

## Notes

- **Update interval** (`update_interval`) controls polling and debounce responsiveness.  
  - 10ms is typical, but 15–20ms may reduce I²C load.
- **Double click**: if `double_max_delay: 0`, all short presses are immediately `single`.
- **Resource use**:  
  - Each MCP23017 adds ~2kB RAM and <1ms CPU per poll cycle.  
  - ESP32 handles up to 8 MCP23017 comfortably with update_interval ≥10ms.

# Domodreams MCP23017 ESPHome Component

Custom ESPHome component for the MCP23017 16‑bit I/O expander with rich button/gesture detection, debouncing, failure guards, and Home Assistant integration.

---

## ✨ Features

- 16 pins per MCP23017 (supports multiple devices on same I²C bus).
- Per‑pin **debouncing**.
- Gesture FSM with:
  - `single` press
  - `double` press
  - `long` press
  - `released`
  - `off` (idle state after delay)
- Configurable delays (`debounce`, `long_min`, `double_max_delay`, `off_delay`, `release_off_delay`).
- Optional **per‑pin overrides** for `double_max_delay` and `long_min`.
- Publishes button state as `text_sensor` values (`single`, `double`, `long`, `released`, `off`).
- Optional *last triggered button* and *last triggered time* sensors.
- Robust I²C guard:
  - Publishes all `off` after 3 consecutive read failures.
  - Marks component failed with clear reason.
  - Optional reboot on fail.
- Clean **debounce → FSM → publish** pipeline.
- Works on ESP32 + ESPHome `external_components`.

---

## 📦 Installation

Place this repo under `esphome/components/domodreams_mcp23017/` in your ESPHome config folder:

```
esphome:
  name: mynode

external_components:
  - source:
      type: local
      path: components
    components: [domodreams_mcp23017]
```

---

## ⚙️ Configuration

### Global MCP23017 block

```yaml
text_sensor:
  - platform: domodreams_mcp23017
    id: mcp1
    i2c_id: i2c_bus_1
    address: 0x20
    debounce: 50                # Debounce in ms (default: 50)
    long_min: 1000              # Long press min duration (ms)
    double_max_delay: 300       # Max delay between clicks (ms)
    off_delay: 100              # Delay after single/double before publishing "off"
    release_off_delay: 1000     # Delay after release before publishing "off"
    reboot_on_fail: false       # Reboot device if I2C fails (default: false)
    update_interval: 10ms       # Polling interval (default: 10ms)
    time_id: homeassistant_time # Optional RTC for timestamps
    last_triggered_button: last_btn
    last_triggered_time: last_time
    sensors:
      - name: MCP1 Pin 0
      - name: MCP1 Pin 1
      - name: MCP1 Pin 2
      - name: MCP1 Pin 3
      # ...
      - name: MCP1 Pin 15
```

### Per‑pin overrides

You can override `double_max_delay` and `long_min` **per sensor**:

```yaml
    sensors:
      - name: MCP1 Pin 0
        double_max_delay: 0   # Disable double-click on this pin
      - name: MCP1 Pin 1
        long_min: 2000        # Require 2s hold for long press
      - name: MCP1 Pin 2
        long_min: 1500
        double_max_delay: 500
```

If not specified, pins inherit the **global defaults**.

---

## ⏱️ Published States

Each pin publishes one of:

- `single` → short press, no double followed.
- `double` → two short presses within `double_max_delay`.
- `long` → pressed ≥ `long_min`.
- `released` → release after long.
- `off` → idle state after off delay.

---

## 🛡️ Failure Handling

- All pins initialized to `off` at boot (never “unknown” in HA).
- If 3 consecutive I²C read failures occur:
  - All pins → `off`
  - Component marked failed with reason
  - Optional reboot after 1s if `reboot_on_fail: true`

---

## 🏗️ Architecture

- **setup()**
  - Configure MCP23017 registers (inputs, pullups, inverted, no interrupts).
  - Publish initial `off` to all sensors.
- **update()**
  - Read GPIOA/B → debounce → FSM → publish.
  - On failure → guard logic.
- **Debounce**
  - Candidate stable for ≥ debounceMs → stable state.
- **FSM**
  - Tracks per‑pin press/release windows.
  - Publishes gestures (`single`, `double`, `long`, `released`, `off`).
- **Publish**
  - Push to text_sensors.
  - Update optional “last triggered button/time”.

---

## 📊 FSM Diagram

```
 IDLE
   │ press
   ▼
 PRESSING ── release (<longMin) ──► WAIT_DOUBLE_WINDOW ── timeout ──► single + offDelay
   │ hold ≥longMin                                     │ second press
   ▼                                                   ▼
 LONG ── release ──► released + releaseOffDelay     double + waitRelease ──► offDelay
```

---
## State Diagram
stateDiagram-v2
    direction LR

    %% ===========================
    %% States
    %% ===========================
    [*] --> IDLE
    IDLE: idle
    PRESSING: pressing
    WAIT_DOUBLE: waitDoubleWindow
    LONG_HELD: longHeld
    DOUBLE_WAIT_RELEASE: doubleWaitRelease
    POST_DELAY_OFF: postDelayOff (timer)
    OFF: off (published)

    %% ===========================
    %% Transitions (with actions)
    %% ===========================
    IDLE --> PRESSING: press (level=HIGH)\n/cancelOff()
    PRESSING --> WAIT_DOUBLE: release && held < longMinMs\n/setDoubleWindow(now + doubleMaxDelayMs)
    PRESSING --> LONG_HELD: held >= longMinMs\n/publish("long")

    %% If double disabled globally or per-pin (doubleMaxDelayMs==0)
    PRESSING --> POST_DELAY_OFF: release && held < longMinMs && doubleMaxDelayMs==0\n/publish("single"), scheduleOff(offDelayMs)

    %% Double path
    WAIT_DOUBLE --> DOUBLE_WAIT_RELEASE: press within window\n/publish("double")
    WAIT_DOUBLE --> POST_DELAY_OFF: window timeout\n/publish("single"), scheduleOff(offDelayMs)

    %% Long path release
    LONG_HELD --> POST_DELAY_OFF: release\n/publish("released"), scheduleOff(releaseOffDelayMs)

    %% Double ends on release -> then off
    DOUBLE_WAIT_RELEASE --> POST_DELAY_OFF: release\n/scheduleOff(offDelayMs)

    %% Any new press while waiting for OFF cancels pending off
    POST_DELAY_OFF --> PRESSING: press\n/cancelOff()

    %% Timer expiry drives the final OFF publication
    POST_DELAY_OFF --> OFF: offDeadline reached\n/publish("off")
    OFF --> IDLE: (implicit return)

    %% ===========================
    %% Notes / Legend
    %% ===========================
    note right of PRESSING
      held = now - tPressStart
      longMinMs: per-pin override or global
    end note

    note right of WAIT_DOUBLE
      window: now < tWindowDeadline
      doubleMaxDelayMs: per-pin override or global
      if 0 => single immediately
    end note

    note right of POST_DELAY_OFF
      scheduleOff(delay):
        - single/double -> offDelayMs
        - released      -> releaseOffDelayMs
    end note
---

---

## 🚀 Roadmap / TODO

- [x] Debounce engine
- [x] FSM for single/double/long/release/off
- [x] I²C guard with fail reason
- [x] Last triggered metadata
- [x] Per‑pin overrides for `long_min` and `double_max_delay`
- [ ] Option for per‑pin debounce
- [ ] Configurable pullup enable/disable
- [ ] Configurable invert logic
- [ ] Group sensors (multi-MCP coordination)
- [ ] Expose binary_sensor variant
- [ ] Smarter logging levels

---

## 🧠 Best Practices

- Keep `update_interval` ≥ 10ms (I²C + debounce load).
- `double_max_delay: 0` disables double-click → faster singles.
- Use per‑pin `long_min` for critical vs secondary buttons.
- Use reboot_on_fail only if hardware is unreliable.

---

## 📚 Example with 4 chips

```yaml
text_sensor:
  - platform: domodreams_mcp23017
    id: mcp1
    address: 0x20
    sensors: [ ... 16 pins ... ]

  - platform: domodreams_mcp23017
    id: mcp2
    address: 0x21
    sensors: [ ... ]

  - platform: domodreams_mcp23017
    id: mcp3
    address: 0x22
    sensors: [ ... ]

  - platform: domodreams_mcp23017
    id: mcp4
    address: 0x23
    sensors: [ ... ]
```

---

## 📌 Notes

- Default init sets **all pins to input with pullup**.
- Buttons wired active-low → logic inverted in driver.
- Works well with long I²C cables up to ~100kHz.

---

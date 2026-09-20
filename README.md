# ESPHome Feetech STS3215 component

An ESPHome external component for Feetech STS3215 serial-bus servos. It speaks
the Feetech protocol directly over ESPHome's UART component; no Arduino servo
library is pulled into the firmware.

## Current features

- Multiple independently addressed servos on one 1 Mbps UART bus
- Absolute position commands and relative `sts3215.step` automations
- Torque enable and torque-limit controls
- Speed-limit and acceleration controls
- Position, speed, signed load/torque-output, voltage, temperature, current,
  and moving-state telemetry
- Per-servo direction inversion for mirrored blind installations
- One block read for control settings and one block read for telemetry per
  servo per polling interval

The component currently targets the STS3215's normal position mode. It does not
change EEPROM settings such as servo ID, baud rate, limits, or operating mode.
That is intentional: unexpected EEPROM writes can strand a servo on the bus or
move it outside a blind's safe travel.

## Hardware

For a Seeed Studio XIAO ESP32-S3 plugged into the Seeed Bus Servo Driver Board:

| Signal | XIAO label | ESP32-S3 GPIO | ESPHome setting |
|---|---:|---:|---|
| Host TX to driver RX | D6 / TX | GPIO43 | `tx_pin: GPIO43` |
| Host RX from driver TX | D7 / RX | GPIO44 | `rx_pin: GPIO44` |

Put the driver board in **UART mode** and power the 12 V servos from a suitable
external supply. Do not power a stalled STS3215 from USB. Give every servo on
the daisy chain a unique ID before connecting them together.

Seeed's generic adapter article contains examples for several XIAO variants and
uses inconsistent D6/D7 labels in one sample. The table above follows the
official XIAO ESP32-S3 pin definition: D6 is TX/GPIO43 and D7 is RX/GPIO44.

## Use from a local checkout

```yaml
external_components:
  - source:
      type: local
      path: components
    components: [sts3215]
```

See [`examples/xiao_esp32s3_blind.yaml`](examples/xiao_esp32s3_blind.yaml) for
a complete one-servo demo. Add another item under `servos:` with a different
`servo_id` to use multiple motors on the same bus.

## Servo configuration

```yaml
sts3215:
  id: servo_bus
  uart_id: servo_uart
  update_interval: 1s
  servos:
    - servo_id: 1
      inverted: false
      position:
        name: Encoder Position
      target_position:
        name: Target Position
      speed_limit:
        name: Speed Limit
      torque_enabled:
        name: Torque Enabled
```

All entities are optional. `servo_id` must be unique within a component and is
validated in the range 0-253. The bus must be configured as 1,000,000 baud,
8 data bits, no parity, and 1 stop bit.

### Units and behavior

- Position is exposed in degrees. Raw position is available separately in
  encoder counts (4096 counts/revolution).
- Speed is exposed in degrees/second. A commanded speed limit of zero preserves
  the STS convention meaning "maximum/unlimited speed."
- Load is the signed motor-output duty/load feedback, in percent. It is useful
  as a torque-output proxy but is not a calibrated torque measurement in N·m.
- Current uses the documented STS scale of 6.5 mA/count.
- Acceleration is the raw 0-254 servo setting; one count represents the
  STS-series acceleration increment documented by Feetech.
- `inverted: true` reverses user-facing position, speed, load, and relative-step
  direction without writing the servo's EEPROM.

## Relative movement

```yaml
button:
  - platform: template
    name: Open 10 degrees
    on_press:
      - sts3215.step:
          id: servo_bus
          servo_id: 1
          degrees: 10
```

Relative moves are clamped to the component's 0-360 degree software range. For
a real blind, enforce narrower mechanical limits in Home Assistant/ESPHome and
configure the servo's own angle limits as a final safety layer.

## Repository use later

Once published, replace the local source with the repository URL:

```yaml
external_components:
  - source: github://DexiTheOne/esphome-sts3215@main
    components: [sts3215]
```

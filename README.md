# ESPHome Feetech STS3215 component

An ESPHome external component for Feetech STS3215 serial-bus servos. It speaks
the Feetech protocol directly over ESPHome's UART component; no Arduino servo
library is pulled into the firmware.

## Current features

- Multiple independently addressed servos on one 1 Mbps UART bus
- Multi-turn position commands and relative `sts3215.step` automations
- Persistent speed, acceleration, torque-limit, jog, and calibration settings
- Three-point (down/middle/up) calibration and one cover per servo
- Optional aggregate cover with bus-wide staggered motor starts
- Automatic torque enable while moving and torque disable at rest
- Position, speed, signed load/torque-output, voltage, temperature, current,
  and moving-state telemetry
- Per-servo direction inversion for manual jog/position controls
- One block read for control settings and one block read for telemetry per
  servo per polling interval

The component uses signed STS3215 multi-turn positions. It does not change
EEPROM settings such as servo ID or baud rate during normal operation;
unexpected EEPROM writes are intentionally excluded.
For this cover implementation, EEPROM register 33 (`Operating_Mode`) must be
`3`. The optional `commission_step_mode` button can perform the complete setup
once through the ESP32: operating mode 3, Phase bit 4 for multi-turn feedback,
and zero minimum/maximum limits for unrestricted motion in either direction. It
writes only when the full configuration is incomplete, relocks EEPROM, and
verifies every value. Normal startup and polling never write EEPROM.

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
      cover:
        name: Blind 1
      commission_step_mode:
        name: Commission Multi-Turn Mode
      speed_limit:
        name: Speed Limit
      torque_enabled:
        name: Torque Enabled
      multi_turn_mode:
        name: Multi-Turn Mode Active
      calibration:
        jog_increment:
          name: Jog Increment
        jog_forward:
          name: Jog Forward
        jog_reverse:
          name: Jog Reverse
        set_down:
          name: Set Fully Down
        set_middle:
          name: Set Middle
        set_up:
          name: Set Fully Up
```

All entities are optional. `servo_id` must be unique within a component and is
validated in the range 0-253. The bus must be configured as 1,000,000 baud,
8 data bits, no parity, and 1 stop bit.

### Units and behavior

- Position is signed multi-turn degrees. Raw position is available separately
  in encoder counts (4096 counts/revolution).
- Speed is exposed in degrees/second. A commanded speed limit of zero preserves
  the STS convention meaning "maximum/unlimited speed."
- Load is the signed motor-output duty/load feedback, in percent. It is useful
  as a torque-output proxy but is not a calibrated torque measurement in N·m.
- Current uses the documented STS scale of 6.5 mA/count.
- Acceleration is the raw 0-254 servo setting; one count represents the
  STS-series acceleration increment documented by Feetech.
- `inverted: true` reverses user-facing position, speed, load, and relative-step
  direction without writing the servo's EEPROM. Cover direction itself is
  inferred from the saved down/middle/up encoder positions.

The ESP32 saves settings and calibration in flash and restores the servo's
volatile RAM registers after reboot. Number entities publish the selected UI
precision (0.1 degree for position/jog and whole units for limits) instead of
replacing it with register-conversion artifacts during every poll.

### One-time multi-turn commissioning

With the motor unloaded, press **Commission Multi-Turn Mode** once for each
servo, then power-cycle the complete ESP32/servo system. The action addresses
only that servo ID, disables torque, unlocks EEPROM, writes operating mode 3,
enables multi-turn feedback, widens the position limit, relocks EEPROM, and
reads all four settings back. **Multi-Turn Mode Active** becomes ON only when
the mode, Phase bit, and both limits verify correctly. Pressing the button again
after that is a read-only no-op, which avoids repeated EEPROM wear.

## Blind calibration

1. Press **Reset Calibration**. This clears all three points, defines the
   current stationary position as logical zero, and unlocks the point buttons.
2. Set **Jog Increment** to a convenient amount. It accepts 0.1 through 2520
   degrees, so commissioning can use tiny movements or multiple turns.
3. Jog to each position and press **Set Fully Down**, **Set Middle**, or **Set
   Fully Up**. The three points can be recorded in any order.

The middle encoder value must be strictly between the endpoint values, in
either direction. Once all three points are valid, Home Assistant exposes a
tilt control: 0% is closed in the down direction, 50% is the calibrated fully
open position, and 100% is closed in the up direction. Calculated openness is
0% at either endpoint and 100% at the middle. This piecewise mapping preserves
an intentionally off-center middle point. Completing calibration locks the
three point buttons; press **Reset Calibration** to deliberately start over.
Cover commands are ignored until calibration is complete. The logical zero is
not changed again after the points are saved, because doing so would shift all
three endpoints. Calibration is independent for every servo and survives ESP32
resets. The last completed position is also saved; on
startup the component restores that logical position because the worm drive
cannot back-drive while power is off. In Mode 3 the servo reports progress for
the current relative move and returns its position counter to zero afterward,
so the ESP32 maintains the accumulated blind position.

Torque is enabled immediately before a queued move starts and disabled when
the target is reached, motion stops, or `move_timeout` expires. The status
entities expose encoder position, moving state, and actual torque-enable state.
While moving, the Home Assistant cover entity continues to report the final
requested tilt instead of publishing intermediate progress. Live progress stays
available through the position diagnostics.

For blind leaves that settle by gravity, enable this per servo:

```yaml
servos:
  - servo_id: 1
    gravity_return_to_zero: true
```

With this option, a command from the 100% closed-up side to an intermediate
tilt automatically queues 0% first and then the requested tilt. Movement toward
100%, and commands directly to 0%, remain direct. Home Assistant displays only
the final requested tilt throughout this sequence.

Home Assistant does not define native named presets for cover entities, so the
component can expose preset buttons that use the same buffered, gravity-aware
tilt command path:

```yaml
servos:
  - servo_id: 1
    presets:
      - name: Blind 1 Closed Down 0%
        tilt_position: 0%
      - name: Blind 1 Tilt 25%
        tilt_position: 25%
      - name: Blind 1 Fully Open 50%
        tilt_position: 50%
      - name: Blind 1 Tilt 75%
        tilt_position: 75%
      - name: Blind 1 Closed Up 100%
        tilt_position: 100%
```

The cover's tilt slider retains the physical orientation scale: both endpoints
are fully closed and 50% is fully open. The component calculates the cover's
open/closed state as 0% open at either endpoint and fully open at 50% tilt.

## Multiple blinds and start sequencing

`start_delay` is enforced by one queue shared by all individual covers and the
optional `main_cover`. Therefore group commands and several individual commands
received at nearly the same time cannot start all motors together. The main
cover queues calibrated servos in their `servos:` list order and reports their
average tilt and openness. The delay applies when starting different motors;
successive commands for one motor begin as soon as its active move completes.
Copy a servo list item and give it a unique ID to scale from one motor to six or
more.

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

Mode 3 uses zero minimum and maximum angle limits so relative jogging can cross
zero in either direction. The calibrated cover never commands beyond its saved
down/up endpoints. Repeated jog presses accumulate into one buffered target, so
quick button presses are not discarded and cannot interrupt the active move.

## Repository use later

Once published, replace the local source with the repository URL:

```yaml
external_components:
  - source: github://DexiTheOne/esphome-sts3215@main
    components: [sts3215]
```

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
- Optional shared relay/MOSFET GPIO that removes motor power at rest
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
  # Optional; omit for an always-powered bus.
  power_pin:
    number: GPIO1
    inverted: false  # Set true for an active-low relay.
  power_on_delay: 1s
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
        reset_blinds:
          name: Reset Calibration
        status:
          name: Calibration Status
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
- `max_acceleration` sets the upper end of each servo's acceleration number
  control (default 254). The XIAO demo uses 50 because its tested STS3215-12V
  repeatedly read back 50 after requests of 51, 60, 100, and 170. The cause
  of that limit is not yet confirmed. Use a higher value only after verifying
  it on the particular servo.
- `inverted: true` reverses user-facing position, speed, load, and relative-step
  direction without writing the servo's EEPROM. Cover direction itself is
  inferred from the saved down/middle/up encoder positions.

The ESP32 saves settings and calibration in flash and restores the servo's
volatile RAM registers after reboot. Number entities publish the selected UI
precision (0.1 degree for position/jog and whole units for limits) instead of
replacing it with register-conversion artifacts during every poll.

### Optional motor power switch

`power_pin` controls a relay or MOSFET for the shared motor supply. The pin is
inactive at ESP32 startup, then briefly turns on to verify each servo's saved
Mode 3 configuration and restore volatile speed, acceleration, and torque-limit
settings. It turns off when all servos have torque disabled and no command or
commissioning work is pending. A move or calibration request wakes the bus;
the component waits `power_on_delay` (default `1s`) before speaking to it. The
GPIO pin's `inverted` option selects active-low hardware. Without `power_pin`,
the bus behaves as an always-powered installation.

UART reads allow 50 ms per attempt and retry twice after a missed or malformed
response. On wake, the component verifies that torque is off and the requested
torque limit was accepted before allowing movement. It also reads back the
accepted acceleration, since a servo may cap a requested value, and saves that
value for later moves. Goal speed is written in each move command; a zero idle
speed readback does not block movement.
If feedback shows no movement within five seconds of a motion command, the
component stops that command, clears its queued follow-up moves, and returns
the cover to its measured position instead of showing an indefinite operation.
While a move is active, feedback is polled every 25 ms so short unloaded moves
can be observed even when the normal telemetry interval is slower.
The current diagnostic release enables `uart_trace` by default to log every UART read request, write
request, received byte stream, discarded late acknowledgement, and timeout
as hexadecimal bytes. Set `uart_trace: false` under `sts3215:` after testing
to avoid the extra log traffic during motion.
The queue skips targets already at the current encoder position without
sending a zero-distance motor command. Debug logs also show cover targets,
remaining queued moves, and explicit stop requests.
On the XIAO ESP32-S3, D0 is GPIO1 and is a suitable relay control pin while
D6/GPIO43 and D7/GPIO44 serve the servo UART. The example uses D0 with
`inverted: false` for an active-high relay. Use a relay input that accepts
3.3 V logic and an external pull-down so its supply stays off while the ESP32
pin is high impedance during reset.

The numeric motor telemetry becomes unavailable while power is off, and the
component skips polling and communication warnings in that state. The logical
blind position and calibration remain in ESP32 memory. Mode 3 is stored in
the servo EEPROM by the one-time commissioning action, so power switching does
not change it. Each wake verifies the mode, Phase bit, and angle limits; an
unconfigured or unresponsive motor cannot start a move. Normal startup and
wake-up never write EEPROM. Speed and torque-limit changes made while the
motor is off are saved and applied on its next wake.

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
an intentionally off-center middle point. Completing calibration locks the jog
and three point controls; press **Reset Calibration** to deliberately start
over. ESPHome buttons cannot publish per-entity availability, so Home Assistant
may continue to draw the locked buttons normally, but presses are rejected by
the device until calibration is reset.

The optional **Calibration Status** text sensor reports `None` before a
calibration session, `Active` after Reset Calibration unlocks jogging and point
capture, `Ok` when all three points form a valid sequence, or `Error` when a
point cannot be captured or the three points are inconsistent. A successful
retry clears a transient error; invalid saved points remain `Error` after a
reboot until corrected or reset. The example exposes this sensor in the same
device configuration group as Reset Calibration; Home Assistant controls the
display order.

Cover commands are ignored until calibration is complete. The logical zero is
not changed again after the points are saved, because doing so would shift all
three endpoints. Calibration is independent for every servo and survives ESP32
resets. The last completed position is also saved; on
startup the component restores that logical position because the worm drive
cannot back-drive while power is off. In Mode 3 the servo reports the signed
distance remaining in the current move and returns its position counter to
zero afterward, so the ESP32 maintains the accumulated blind position.
Firmware upgrades from preference version 2 clear saved position and blind
calibration because the earlier feedback interpretation could persist incorrect
coordinates. Speed, acceleration, torque limit, and jog increment are retained;
press Reset Calibration and establish all three points again after upgrading.
This assumes the worm gearbox holds the blind still while unpowered. A stalled
or interrupted move, or manual movement with power off, can make the saved
position inaccurate; reset calibration before relying on its endpoints again.

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

With this option, every decreasing tilt command to a value above 0% automatically
queues 0% first and then the requested tilt. For example, 75% to 25% runs as
75% to 0% to 25%. Increasing transitions, and commands directly to 0%, remain
direct. Home Assistant displays only the final requested tilt throughout this
sequence.

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

The standard Home Assistant open and close controls move toward the next 25%
tilt boundary instead of jumping to an endpoint. They snap in the requested
direction: for example, open from 16% targets 25%, open from 25% targets 50%,
and close from 26% targets 25%. Repeated presses use the latest queued target,
so they can be used predictably while a move is still pending. Home Assistant's
tilt slider and favorite tilt positions continue to command exact percentages.

Home Assistant draws a slatted background for every tilt-position control. The
frontend does not expose an entity capability or ESPHome option for disabling
that decoration, so it cannot be changed by this external component.

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

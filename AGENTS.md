# ESPHome STS3215 project notes

## Project goal

Build a repository-ready ESPHome external component for one or more Feetech
STS3215-12V smart servos on a shared serial bus, initially for automated blinds
using a Seeed Studio XIAO ESP32-S3 and Seeed Bus Servo Driver Board.

The component must provide position commands, torque enable/limit, speed and
direction handling, telemetry, and multiple servo IDs without requiring users
to write C++ lambdas. The demo must expose encoder position and buttons for
positive/negative 10-degree moves.

## Current implementation

- Component domain: `sts3215`
- Source: `components/sts3215/`
- Demo: `examples/xiao_esp32s3_blind.yaml`
- User documentation: `README.md`
- Transport: ESPHome `uart::UARTDevice`, 1,000,000 baud, 8N1
- Multi-servo model: one `STS3215Component` owns a list of unique servo IDs
- Protocol implementation is local and dependency-free.
- The poller reads settings registers 40-49 and telemetry registers 56-70 in
  contiguous transactions for each servo.
- Relative movement is exposed as the `sts3215.step` automation action.
- Optional `power_pin` controls the shared motor supply; the XIAO demo uses
  active-high D0/GPIO1 and waits `power_on_delay: 1s` before UART access.
- On each power-up, the component verifies EEPROM Mode 3/Phase/limits and
  restores volatile acceleration, speed, and torque-limit registers.

## Verified protocol findings

Feetech packets use:

`FF FF ID LENGTH INSTRUCTION PARAMETERS... CHECKSUM`

The checksum is the low byte of the one's complement of the sum from ID through
the final parameter. Status packets replace instruction with an error/status
byte. Reads use instruction `0x02`; writes use `0x03`.

Relevant STS3215 registers (decimal):

| Register | Width | Meaning |
|---:|---:|---|
| 40 | 1 | Torque enable |
| 41 | 1 | Acceleration |
| 42 | 2 | Goal position |
| 44 | 2 | Goal time |
| 46 | 2 | Goal speed |
| 48 | 2 | Torque limit, 0-1000 |
| 56 | 2 | Present position |
| 58 | 2 | Present speed |
| 60 | 2 | Present load/output, sign bit 10 |
| 62 | 1 | Voltage, 0.1 V/count |
| 63 | 1 | Temperature, °C |
| 65 | 1 | Servo status/error bitmap |
| 66 | 1 | Moving flag |
| 69 | 2 | Present current, 6.5 mA/count |

Position, speed, and current use a sign-magnitude representation with bit 15 as
the sign. Load uses bit 10 as the sign. The normal single-turn encoder has 4096
counts/revolution. The official Feetech Arduino library's `WritePosEx` writes a
single seven-byte block beginning at register 41: acceleration, goal position,
zero goal time, and goal speed. The component follows that behavior.

Primary/relevant references consulted:

- Feetech official [`FTServo_Arduino`](https://github.com/ftservo/FTServo_Arduino)
  (`SMS_STS.h`, `SMS_STS.cpp`, and `SCS.cpp`)
- [Feetech/Seeed STS3215 datasheet](https://files.seeedstudio.com/products/Feetech/108090023_STS3215-C001_Datasheet.pdf)
  and [communication protocol manual](https://files.seeedstudio.com/wiki/robotics/Actuator/feetech/Communication_Protocol_Manual.pdf)
- [ESPHome component architecture](https://developers.esphome.io/architecture/components/)
- [ESPHome external components](https://esphome.io/components/external_components/)
  and [UART component](https://esphome.io/components/uart/) documentation
- [Seeed XIAO Bus Servo Adapter guide](https://wiki.seeedstudio.com/xiao_bus_servo_adapter/)
- [Seeed XIAO ESP32-S3 pin documentation](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
  and [Espressif Arduino pin variant](https://github.com/espressif/arduino-esp32/blob/master/variants/XIAO_ESP32S3/pins_arduino.h)

## Hardware findings

For the XIAO ESP32-S3 specifically:

- D6 is UART TX and maps to GPIO43.
- D7 is UART RX and maps to GPIO44.
- ESPHome must therefore use `tx_pin: GPIO43` and `rx_pin: GPIO44` when the XIAO
  is plugged into the Seeed servo driver board.
- The driver board must be placed in UART mode. Its RX receives host TX, and its
  TX drives host RX.
- The servos require an external 12 V supply sized for startup/stall current.
- All daisy-chained servos need unique IDs before sharing the bus.

Important documentation discrepancy: Seeed's generic driver-board page says
driver RX connects to host TX and driver TX to host RX (correct), but one sample
labels `S_RXD D7` and `S_TXD D6` in a way that can be read from the opposite
perspective. Always use the XIAO ESP32-S3's authoritative mapping above.

## ESPHome architecture findings

- Modern custom integrations must be external components with Python config
  validation/code generation plus C++ runtime code; the old `custom_components`
  auto-loader was removed in ESPHome 2026.6.
- Shareable repositories should place components in `components/<domain>/` (or
  `esphome/components/<domain>/`).
- UART clients must extend `UART_DEVICE_SCHEMA`, register the UART device, and
  use final validation to claim RX/TX and enforce bus settings.
- This component validates 1 Mbps, RX+TX, 8 data bits, no parity, one stop bit.

## Live device access with ESPHome CLI

The current demo device is `sts3215-blind-demo` at `10.0.0.110`. Its native
ESPHome API uses port 6053 without Noise encryption as of 2026-09-21. Confirm
the address and API settings from current device logs before relying on them.
The `esphome logs` command streams logs and can include entity states; it does not provide
cover, button, or number commands. Use the `aioesphomeapi` package installed
with ESPHome for those native API commands.

On Windows PowerShell, install the CLI in a temporary virtual environment:

```powershell
python -m venv "$env:TEMP\sts3215-cli"
& "$env:TEMP\sts3215-cli\Scripts\python.exe" -m pip install esphome
& "$env:TEMP\sts3215-cli\Scripts\esphome.exe" version
```

The repository's demo YAML references local Wi-Fi secrets. For log access,
create a temporary connection-only YAML; its placeholder Wi-Fi values are
never uploaded to the device:

```powershell
@'
esphome:
  name: sts3215-blind-demo
esp32:
  board: seeed_xiao_esp32s3
logger:
  hardware_uart: USB_SERIAL_JTAG
wifi:
  ssid: diagnostics-only
  password: diagnostics-only
api:
'@ | Set-Content -LiteralPath "$env:TEMP\sts3215-logs.yaml" -Encoding utf8

& "$env:TEMP\sts3215-cli\Scripts\esphome.exe" logs "$env:TEMP\sts3215-logs.yaml" --device 10.0.0.110 --no-states
```

Replace `--no-states` with `--states` to see entity updates. Stop the stream
with Ctrl+C. To inspect native API entities or send a control command, use the
same virtual environment's Python. This example lists the entities and stops
the individual cover; stopping does not command a new position:

```python
import asyncio
from aioesphomeapi import APIClient

async def main():
    client = APIClient("10.0.0.110", 6053, password=None, noise_psk=None)
    await client.connect()
    try:
        entities, _ = await client.list_entities_services()
        for entity in entities:
            print(type(entity).__name__, entity.name, entity.key)
        cover = next(e for e in entities if e.name == "Blind 1")
        client.cover_command(cover.key, stop=True)
        await asyncio.sleep(1)  # Allow the command to leave the API queue.
    finally:
        await client.disconnect()

asyncio.run(main())
```

Save the Python example as `script.py` and run it with
`& "$env:TEMP\sts3215-cli\Scripts\python.exe" script.py`.
For an authorized position command, use
`client.cover_command(cover.key, tilt=0.50)` (50% calibrated tilt).
`client.number_command(entity.key, value)` changes a Number entity and
`client.button_command(entity.key)` presses a Button entity. Look up keys by
name each session rather than hardcoding them. A cover tilt command can move
the blind through a gravity-return sequence; the multi-turn commissioning
button can write servo EEPROM. Do not use the temporary diagnostic YAML with
`esphome run` or `esphome upload`; deploy with the real device YAML and secrets.

During the 2026-09-21 diagnosis, repeated wake attempts produced a valid
zero-parameter write acknowledgement (status packet length 2) just before the
settings read response. The old reader rejected the first packet and discarded
the move. Commit `ca72a94` makes the reader consume and skip such acknowledgements
until it receives the requested response. A full ESPHome 2026.9.0 build passed;
the fix still needs verification on the actual device after firmware deployment.

## Scope and safety decisions

- Mode 3 step-servo operation is required for the blind. Continuous wheel/PWM
  modes are not exposed.
- Normal startup, wake-up, and movement do not write EEPROM. The explicit
  commissioning button can write Mode 3, Phase bit 4, and angle limits once.
- `inverted` is a software coordinate transform, not an EEPROM direction write.
- Reported load is a signed output-duty/load proxy, not calibrated mechanical
  torque. Avoid presenting it as N·m.
- Blind-specific endpoints must be calibrated before cover movement.
- Writes are fire-and-observe: the next settings/telemetry poll confirms state.
  Reads skip delayed write acknowledgements from status-return level 2 servos.

## Remaining physical validation

Software validation cannot replace bench testing. Before a blind is attached:

1. Test one unloaded servo with a current-limited 12 V supply.
2. Confirm its ID and 1 Mbps baud rate.
3. Verify encoder direction and set `inverted` if needed.
4. Verify +10/-10 degree buttons at low speed and torque limit.
5. Confirm temperature, current, voltage, load, and moving telemetry.
6. Add servos one at a time after assigning unique IDs.
7. Establish mechanical open/closed endpoints and independent angle limits.

## Development guidance

Preserve the no-EEPROM-write default and multi-servo behavior. Run ESPHome YAML
validation/compilation against the current stable release after changes. Bench
tests should include bad checksum, absent servo, duplicate ID validation,
inverted motion, endpoint clamping, and more than one servo on the bus.

Before stopping repository work, commit completed local changes and push local
commits to GitHub. If a push cannot be completed, report the reason and the
unpushed commits.

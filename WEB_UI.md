# Motor diagnostics and calibration page

The multi-blind example serves ESPHome's version 3 web page at the device's IP
address on port 80. The live Common Area Blinds installation uses the same
configuration at `http://10.0.0.110/`.

Each individual blind and **All Blinds** remain native API covers for Home
Assistant. The motor telemetry, settings, and calibration entities have
`internal: true`, so the native API does not advertise them. The web server's
`include_internal: true` makes them available through its local HTTP API.
`web_ui: true` serves the component's self-contained blind dashboard at `/`.
It shows **All Blinds** and a card for each motor, with a normalized vertical
encoder bar, jog and calibration controls, cover tilt, diagnostics, and saved
settings.

```yaml
web_server:
  port: 80
  version: 3
  local: true
  include_internal: true
  sorting_groups:
    - id: all_blinds_group
      name: All Blinds
      sorting_weight: 0
    - id: blind_6_group
      name: Blind 6
      sorting_weight: 10
  ota: false

sts3215:
  web_ui: true
  servos:
    - servo_id: 6
      cover:
        name: Blind 6
        web_server:
          sorting_group_id: blind_6_group
          sorting_weight: 0
      speed_limit:
        name: Blind 6 Speed Limit
        internal: true
        web_server:
          sorting_group_id: blind_6_group
      calibration:
        status:
          name: Blind 6 Calibration Status
          internal: true
          web_server:
            sorting_group_id: blind_6_group
```

`local: true` embeds the page assets in firmware, so opening the page does not
require internet access. Native ESPHome OTA remains enabled by the separate
`ota: - platform: esphome` block. The web server does not expose firmware
uploads.

The bar maps the saved down and up encoder points to opposite ends. Selecting
**Negative is up** flips the entire graph, placing fully up at the bottom.
Current position is blue fill clamped to the range. Green lines mark saved
calibrated or manually entered points; blue lines mark calculated points. Red
marks servo zero, and a grey line ends the blue fill at the live encoder
position. A red/green striped line marks an endpoint that overlaps zero.
The right column shows down, 25%, 50%, 75%, and up encoder points. For
incomplete calibration, the bar uses available points and the current reading
with a minimum display span. The top button starts auto calibration; the left
controls handle jogging, stopping, and point capture. Jog increment and output
speed use degrees. Stop also cancels auto calibration.

Manual Control starts off after boot for valid calibration and on when
calibration is absent or invalid. Turning it on permits jogging and capturing
positions without clearing calibration. Edit Positions starts off after boot.
It accepts whole number encoder counts for all five positions in monotonic
order. Saving marks all five as Manual, including formerly calculated points.
Calibration point values and their provenance
persist through power cycles and OTA; the two editing toggles do not.

The component saves per-motor speed, acceleration, startup force, torque limit,
jog increment, direction choice, calibration points, and last logical position
in ESP32 flash. Startup force is restored to servo RAM when motor power returns.
The calibration status is reconstructed from those saved points at boot. An OTA
update preserves these values as long as the servo IDs and the `sts3215` ID
remain the same. A complete flash erase or a change to either ID resets them.

The page permits calibration and motor control. Keep it on a trusted local
network; add `web_server.auth` with secrets if clients on the network should
not have access to those controls.

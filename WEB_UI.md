# Motor diagnostics and calibration page

The six-blind example serves ESPHome's version 3 web page at the device's IP
address on port 80. The live Common Area Blinds installation uses the same
configuration at `http://10.0.0.110/`.

Each individual blind and **All Blinds** remain native API covers for Home
Assistant. The motor telemetry, settings, and calibration entities have
`internal: true`, so the native API does not advertise them. The web server's
`include_internal: true` displays them on the device page. The web page also
shows the covers.

```yaml
web_server:
  port: 80
  version: 3
  local: true
  include_internal: true
  ota: false

sts3215:
  servos:
    - servo_id: 6
      cover:
        name: Blind 6
      speed_limit:
        name: Blind 6 Speed Limit
        internal: true
      calibration:
        status:
          name: Blind 6 Calibration Status
          internal: true
```

`local: true` embeds the page assets in firmware, so opening the page does not
require internet access. Native ESPHome OTA remains enabled by the separate
`ota: - platform: esphome` block. The web server does not expose firmware
uploads.

The component saves per-motor speed, acceleration, torque limit, jog increment,
direction choice, calibration points, and last logical position in ESP32 flash.
The calibration status is reconstructed from those saved points at boot. An OTA
update preserves these values as long as the servo IDs and the `sts3215` ID
remain the same. A complete flash erase or a change to either ID resets them.

The page permits calibration and motor control. Keep it on a trusted local
network; add `web_server.auth` with secrets if clients on the network should
not have access to those controls.

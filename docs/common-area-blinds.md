# Common-area six-blind controller

The live ESPHome configuration is `motorized-blinds.yaml`, with device name
`himblinds` and friendly name `HIMBlinds`. Its ignored local copy is
`local/common-area-blinds.yaml`. Keep
its installation-specific movement settings when updating the public example
in `examples/xiao_esp32s3_six_blinds.yaml`.

Use ESPHome's `web_server` with `include_internal: true` and `sts3215.web_ui:
true` for servo telemetry, calibration, and motor setup. Mark each motor
telemetry and setup entity `internal: true` so it remains on the device web
page without becoming a Home Assistant entity. Home Assistant should receive
only the All Blinds and six individual cover entities plus these I²C sensors:

- Pressure from the BMP280 at `0x77` on the main bus.
- Temperature and Humidity from the AHT20 at `0x38` on the main bus.
- +45° Lux from the VEML7700 tilted up 45° on TCA9548A channel 1.
- -45° Lux from the VEML7700 tilted down 45° on TCA9548A channel 2.
- Combined Lux, the average of the two channel readings.
- Sun Angle in degrees, positive above the window normal and negative below it.

The XIAO ESP32-S3 uses D4/GPIO5 for SDA and D5/GPIO6 for SCL. With all TCA9548A
address pins grounded, its address is `0x70`; each VEML7700 uses `0x10` on its
separate channel. Keep the sensor entities non-internal so Home Assistant can
receive them.

Sun Angle uses `atan((L1 - L2) / (L1 + L2)) × 180 / π`, with the ratio clamped
to [-1, 1]. The denominator is the sum even though Combined Lux is the average.
When Combined Lux is below 50 lx or either channel has no valid reading, the
ESPHome numeric sensor publishes `NAN`, which Home Assistant displays as an
unknown state.

The two VEML7700 sensors and their Combined Lux and Sun Angle calculations
update every 100 ms (10 Hz). The VEML7700 sensors use fixed 50 ms integration
so automatic gain and integration adjustments do not delay a 100 ms polling
cycle. BMP280 and AHT20 remain at 60-second intervals.

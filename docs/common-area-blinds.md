# Common-area six-blind controller

The live installation configuration is `local/common-area-blinds.yaml`. Keep
its installation-specific movement settings when updating the public example
in `examples/xiao_esp32s3_six_blinds.yaml`.

Use ESPHome's `web_server` with `include_internal: true` and `sts3215.web_ui:
true` for servo telemetry, calibration, and motor setup. Mark each motor
telemetry and setup entity `internal: true` so it remains on the device web
page without becoming a Home Assistant entity. Home Assistant should receive
only the All Blinds and six individual cover entities plus these I²C sensors:

- BMP280 temperature and pressure at `0x77` on the main bus.
- AHT20 temperature and humidity at `0x38` on the main bus.
- VEML7700 illuminance on TCA9548A channel 1.
- VEML7700 illuminance on TCA9548A channel 2.

The XIAO ESP32-S3 uses D4/GPIO5 for SDA and D5/GPIO6 for SCL. With all TCA9548A
address pins grounded, its address is `0x70`; each VEML7700 uses `0x10` on its
separate channel. Keep the sensor entities non-internal so Home Assistant can
receive them.

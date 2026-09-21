import zlib

from esphome import automation
import esphome.codegen as cg
from esphome.components import binary_sensor, button, cover, number, sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID, CONF_INVERTED, DEVICE_CLASS_CURRENT, DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE, ENTITY_CATEGORY_CONFIG, ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT, UNIT_AMPERE, UNIT_CELSIUS, UNIT_DEGREES,
    UNIT_PERCENT, UNIT_VOLT,
)

CODEOWNERS = []
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "button", "cover", "number", "sensor"]
MULTI_CONF = True

CONF_SERVOS = "servos"
CONF_SERVO_ID = "servo_id"
CONF_POSITION = "position"
CONF_POSITION_RAW = "position_raw"
CONF_SPEED = "speed"
CONF_LOAD = "load"
CONF_TEMPERATURE = "temperature"
CONF_VOLTAGE = "voltage"
CONF_CURRENT = "current"
CONF_STATUS = "status"
CONF_MOVING = "moving"
CONF_TARGET_POSITION = "target_position"
CONF_SPEED_LIMIT = "speed_limit"
CONF_ACCELERATION = "acceleration"
CONF_TORQUE_LIMIT = "torque_limit"
CONF_TORQUE_ENABLED = "torque_enabled"
CONF_DEGREES = "degrees"
CONF_COVER = "cover"
CONF_MAIN_COVER = "main_cover"
CONF_START_DELAY = "start_delay"
CONF_MOVE_TIMEOUT = "move_timeout"
CONF_POSITION_TOLERANCE = "position_tolerance"
CONF_CALIBRATION = "calibration"
CONF_JOG_INCREMENT = "jog_increment"
CONF_JOG_FORWARD = "jog_forward"
CONF_JOG_REVERSE = "jog_reverse"
CONF_SET_DOWN = "set_down"
CONF_SET_MIDDLE = "set_middle"
CONF_SET_UP = "set_up"
CONF_RESET_BLINDS = "reset_blinds"
CONF_INITIAL_SPEED = "initial_speed"
CONF_INITIAL_ACCELERATION = "initial_acceleration"
CONF_INITIAL_TORQUE_LIMIT = "initial_torque_limit"
CONF_GRAVITY_RETURN_TO_ZERO = "gravity_return_to_zero"
CONF_COMMISSION_STEP_MODE = "commission_step_mode"
CONF_MULTI_TURN_MODE = "multi_turn_mode"
CONF_PRESETS = "presets"
CONF_TILT_POSITION = "tilt_position"

sts3215_ns = cg.esphome_ns.namespace("sts3215")
STS3215Component = sts3215_ns.class_("STS3215Component", cg.PollingComponent, uart.UARTDevice)
STS3215PositionNumber = sts3215_ns.class_("STS3215PositionNumber", number.Number)
STS3215SpeedNumber = sts3215_ns.class_("STS3215SpeedNumber", number.Number)
STS3215AccelerationNumber = sts3215_ns.class_("STS3215AccelerationNumber", number.Number)
STS3215TorqueLimitNumber = sts3215_ns.class_("STS3215TorqueLimitNumber", number.Number)
STS3215JogIncrementNumber = sts3215_ns.class_("STS3215JogIncrementNumber", number.Number)
STS3215Cover = sts3215_ns.class_("STS3215Cover", cover.Cover)
STS3215GroupCover = sts3215_ns.class_("STS3215GroupCover", cover.Cover)
STS3215CalibrationButton = sts3215_ns.class_("STS3215CalibrationButton", button.Button)
STS3215PresetButton = sts3215_ns.class_("STS3215PresetButton", button.Button)
STS3215StepAction = sts3215_ns.class_("STS3215StepAction", automation.Action)


def _unique_servo_ids(config):
    seen = set()
    for servo_config in config[CONF_SERVOS]:
        servo_id = servo_config[CONF_SERVO_ID]
        if servo_id in seen:
            raise cv.Invalid(f"Duplicate STS3215 servo_id {servo_id}")
        seen.add(servo_id)
    return config


CALIBRATION_SCHEMA = cv.Schema({
    cv.Optional(CONF_RESET_BLINDS): button.button_schema(
        STS3215CalibrationButton, entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:restart-alert"),
    cv.Optional(CONF_JOG_INCREMENT): number.number_schema(
        STS3215JogIncrementNumber, unit_of_measurement=UNIT_DEGREES,
        entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:rotate-360"),
    cv.Optional(CONF_JOG_FORWARD): button.button_schema(
        STS3215CalibrationButton, entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:rotate-right"),
    cv.Optional(CONF_JOG_REVERSE): button.button_schema(
        STS3215CalibrationButton, entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:rotate-left"),
    cv.Optional(CONF_SET_DOWN): button.button_schema(
        STS3215CalibrationButton, entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:arrow-collapse-down"),
    cv.Optional(CONF_SET_MIDDLE): button.button_schema(
        STS3215CalibrationButton, entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:arrow-collapse-vertical"),
    cv.Optional(CONF_SET_UP): button.button_schema(
        STS3215CalibrationButton, entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:arrow-collapse-up"),
})

PRESET_SCHEMA = button.button_schema(
    STS3215PresetButton, icon="mdi:blinds-horizontal"
).extend({
    cv.Required(CONF_TILT_POSITION): cv.percentage,
})

SERVO_SCHEMA = cv.Schema({
    cv.Required(CONF_SERVO_ID): cv.int_range(min=0, max=253),
    cv.Optional(CONF_INVERTED, default=False): cv.boolean,
    cv.Optional(CONF_INITIAL_SPEED, default=90.0): cv.float_range(min=0, max=360),
    cv.Optional(CONF_INITIAL_ACCELERATION, default=20): cv.int_range(min=0, max=254),
    cv.Optional(CONF_INITIAL_TORQUE_LIMIT, default=30.0): cv.float_range(min=0, max=100),
    cv.Optional(CONF_GRAVITY_RETURN_TO_ZERO, default=False): cv.boolean,
    cv.Optional(CONF_POSITION): sensor.sensor_schema(
        unit_of_measurement=UNIT_DEGREES, accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT, icon="mdi:angle-acute"),
    cv.Optional(CONF_POSITION_RAW): sensor.sensor_schema(
        accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:counter"),
    cv.Optional(CONF_SPEED): sensor.sensor_schema(
        unit_of_measurement="°/s", accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT, icon="mdi:speedometer"),
    cv.Optional(CONF_LOAD): sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT, accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT, icon="mdi:arm-flex"),
    cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=0,
        device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
    cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT, accuracy_decimals=1,
        device_class=DEVICE_CLASS_VOLTAGE, state_class=STATE_CLASS_MEASUREMENT),
    cv.Optional(CONF_CURRENT): sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE, accuracy_decimals=3,
        device_class=DEVICE_CLASS_CURRENT, state_class=STATE_CLASS_MEASUREMENT),
    cv.Optional(CONF_STATUS): sensor.sensor_schema(
        accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:alert-circle-outline"),
    cv.Optional(CONF_MOVING): binary_sensor.binary_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:motion"),
    cv.Optional(CONF_TORQUE_ENABLED): binary_sensor.binary_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:engine"),
    cv.Optional(CONF_MULTI_TURN_MODE): binary_sensor.binary_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:rotate-orbit"),
    cv.Optional(CONF_TARGET_POSITION): number.number_schema(
        STS3215PositionNumber, unit_of_measurement=UNIT_DEGREES, icon="mdi:angle-acute"),
    cv.Optional(CONF_SPEED_LIMIT): number.number_schema(
        STS3215SpeedNumber, unit_of_measurement="°/s",
        entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:speedometer"),
    cv.Optional(CONF_ACCELERATION): number.number_schema(
        STS3215AccelerationNumber, entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:run-fast"),
    cv.Optional(CONF_TORQUE_LIMIT): number.number_schema(
        STS3215TorqueLimitNumber, unit_of_measurement=UNIT_PERCENT,
        entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:arm-flex"),
    cv.Optional(CONF_COVER): cover.cover_schema(STS3215Cover, device_class="blind"),
    cv.Optional(CONF_COMMISSION_STEP_MODE): button.button_schema(
        STS3215CalibrationButton, entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:memory-arrow-down"),
    cv.Optional(CONF_CALIBRATION): CALIBRATION_SCHEMA,
    cv.Optional(CONF_PRESETS): cv.ensure_list(PRESET_SCHEMA),
})

CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(STS3215Component),
        cv.Required(CONF_SERVOS): cv.All(cv.ensure_list(SERVO_SCHEMA), cv.Length(min=1)),
        cv.Optional(CONF_MAIN_COVER): cover.cover_schema(STS3215GroupCover, device_class="blind"),
        cv.Optional(CONF_START_DELAY, default="2s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_MOVE_TIMEOUT, default="2min"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_POSITION_TOLERANCE, default=5): cv.int_range(min=1, max=1000),
    }).extend(uart.UART_DEVICE_SCHEMA).extend(cv.polling_component_schema("500ms")),
    _unique_servo_ids,
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "sts3215", baud_rate=1_000_000, require_tx=True, require_rx=True,
    data_bits=8, parity="NONE", stop_bits=1)


async def _new_number(config, parent, servo_id, min_value, max_value, step):
    var = await number.new_number(config, min_value=min_value, max_value=max_value, step=step)
    cg.add(var.set_parent(parent))
    cg.add(var.set_servo_id(servo_id))
    return var


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_start_delay(config[CONF_START_DELAY].total_milliseconds))
    cg.add(var.set_move_timeout(config[CONF_MOVE_TIMEOUT].total_milliseconds))
    cg.add(var.set_position_tolerance(config[CONF_POSITION_TOLERANCE]))

    component_key = zlib.crc32(str(config[CONF_ID]).encode("utf-8")) & 0xFFFFFFFF
    for servo_config in config[CONF_SERVOS]:
        servo_id = servo_config[CONF_SERVO_ID]
        pref_key = (component_key ^ 0x5A321500 ^ servo_id) & 0xFFFFFFFF
        cg.add(var.add_servo(
            servo_id, servo_config[CONF_INVERTED], pref_key,
            servo_config[CONF_INITIAL_SPEED], servo_config[CONF_INITIAL_ACCELERATION],
            servo_config[CONF_INITIAL_TORQUE_LIMIT], servo_config[CONF_GRAVITY_RETURN_TO_ZERO]))

        for key, setter in (
            (CONF_POSITION, "set_position_sensor"), (CONF_POSITION_RAW, "set_position_raw_sensor"),
            (CONF_SPEED, "set_speed_sensor"), (CONF_LOAD, "set_load_sensor"),
            (CONF_TEMPERATURE, "set_temperature_sensor"), (CONF_VOLTAGE, "set_voltage_sensor"),
            (CONF_CURRENT, "set_current_sensor"), (CONF_STATUS, "set_status_sensor"),
        ):
            if key in servo_config:
                sens = await sensor.new_sensor(servo_config[key])
                cg.add(getattr(var, setter)(servo_id, sens))

        for key, setter in ((CONF_MOVING, "set_moving_sensor"),
                            (CONF_TORQUE_ENABLED, "set_torque_sensor"),
                            (CONF_MULTI_TURN_MODE, "set_multi_turn_sensor")):
            if key in servo_config:
                sens = await binary_sensor.new_binary_sensor(servo_config[key])
                cg.add(getattr(var, setter)(servo_id, sens))

        if CONF_TARGET_POSITION in servo_config:
            target = await _new_number(servo_config[CONF_TARGET_POSITION], var, servo_id,
                                       -2520.0, 2520.0, 0.1)
            cg.add(var.set_target_position_number(servo_id, target))
        if CONF_SPEED_LIMIT in servo_config:
            speed = await _new_number(servo_config[CONF_SPEED_LIMIT], var, servo_id, 0.0, 360.0, 1.0)
            cg.add(var.set_speed_limit_number(servo_id, speed))
        if CONF_ACCELERATION in servo_config:
            accel = await _new_number(servo_config[CONF_ACCELERATION], var, servo_id, 0.0, 254.0, 1.0)
            cg.add(var.set_acceleration_number(servo_id, accel))
        if CONF_TORQUE_LIMIT in servo_config:
            torque = await _new_number(servo_config[CONF_TORQUE_LIMIT], var, servo_id, 0.0, 100.0, 1.0)
            cg.add(var.set_torque_limit_number(servo_id, torque))

        if CONF_COVER in servo_config:
            cov = await cover.new_cover(servo_config[CONF_COVER])
            cg.add(cov.set_parent(var))
            cg.add(cov.set_servo_id(servo_id))
            cg.add(var.set_cover(servo_id, cov))

        if CONF_COMMISSION_STEP_MODE in servo_config:
            btn = await button.new_button(servo_config[CONF_COMMISSION_STEP_MODE])
            cg.add(btn.set_parent(var))
            cg.add(btn.set_servo_id(servo_id))
            cg.add(btn.set_action(5))

        if calibration := servo_config.get(CONF_CALIBRATION):
            if CONF_JOG_INCREMENT in calibration:
                jog = await _new_number(calibration[CONF_JOG_INCREMENT], var, servo_id,
                                        0.1, 2520.0, 0.1)
                cg.add(var.set_jog_increment_number(servo_id, jog))
            for key, action in {
                CONF_RESET_BLINDS: 6,
                CONF_JOG_FORWARD: 0, CONF_JOG_REVERSE: 1, CONF_SET_DOWN: 2,
                CONF_SET_MIDDLE: 3, CONF_SET_UP: 4,
            }.items():
                if key in calibration:
                    btn = await button.new_button(calibration[key])
                    cg.add(btn.set_parent(var))
                    cg.add(btn.set_servo_id(servo_id))
                    cg.add(btn.set_action(action))

        for preset_config in servo_config.get(CONF_PRESETS, []):
            preset = await button.new_button(preset_config)
            cg.add(preset.set_parent(var))
            cg.add(preset.set_servo_id(servo_id))
            cg.add(preset.set_tilt(preset_config[CONF_TILT_POSITION]))

    if CONF_MAIN_COVER in config:
        group = await cover.new_cover(config[CONF_MAIN_COVER])
        cg.add(group.set_parent(var))
        cg.add(var.set_group_cover(group))


STEP_ACTION_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(STS3215Component),
    cv.Required(CONF_SERVO_ID): cv.templatable(cv.int_range(min=0, max=253)),
    cv.Required(CONF_DEGREES): cv.templatable(cv.float_),
})


@automation.register_action("sts3215.step", STS3215StepAction, STEP_ACTION_SCHEMA, synchronous=True)
async def sts3215_step_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    servo_id = await cg.templatable(config[CONF_SERVO_ID], args, cg.uint8)
    degrees = await cg.templatable(config[CONF_DEGREES], args, cg.float_)
    cg.add(var.set_servo_id(servo_id))
    cg.add(var.set_degrees(degrees))
    return var

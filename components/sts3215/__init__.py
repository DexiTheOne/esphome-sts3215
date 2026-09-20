from esphome import automation
import esphome.codegen as cg
from esphome.components import binary_sensor, number, sensor, switch, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_INVERTED,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_DEGREES,
    UNIT_PERCENT,
    UNIT_VOLT,
)

CODEOWNERS = []
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "number", "sensor", "switch"]
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
CONF_MOVING = "moving"
CONF_TARGET_POSITION = "target_position"
CONF_SPEED_LIMIT = "speed_limit"
CONF_ACCELERATION = "acceleration"
CONF_TORQUE_LIMIT = "torque_limit"
CONF_TORQUE_ENABLED = "torque_enabled"
CONF_DEGREES = "degrees"

sts3215_ns = cg.esphome_ns.namespace("sts3215")
STS3215Component = sts3215_ns.class_(
    "STS3215Component", cg.PollingComponent, uart.UARTDevice
)
STS3215PositionNumber = sts3215_ns.class_("STS3215PositionNumber", number.Number)
STS3215SpeedNumber = sts3215_ns.class_("STS3215SpeedNumber", number.Number)
STS3215AccelerationNumber = sts3215_ns.class_(
    "STS3215AccelerationNumber", number.Number
)
STS3215TorqueLimitNumber = sts3215_ns.class_(
    "STS3215TorqueLimitNumber", number.Number
)
STS3215TorqueSwitch = sts3215_ns.class_("STS3215TorqueSwitch", switch.Switch)
STS3215StepAction = sts3215_ns.class_("STS3215StepAction", automation.Action)


def _unique_servo_ids(config):
    seen = set()
    for servo_config in config[CONF_SERVOS]:
        servo_id = servo_config[CONF_SERVO_ID]
        if servo_id in seen:
            raise cv.Invalid(f"Duplicate STS3215 servo_id {servo_id}")
        seen.add(servo_id)
    return config


SERVO_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SERVO_ID): cv.int_range(min=0, max=253),
        cv.Optional(CONF_INVERTED, default=False): cv.boolean,
        cv.Optional(CONF_POSITION): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:angle-acute",
        ),
        cv.Optional(CONF_POSITION_RAW): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:counter",
        ),
        cv.Optional(CONF_SPEED): sensor.sensor_schema(
            unit_of_measurement="°/s",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:speedometer",
        ),
        cv.Optional(CONF_LOAD): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:arm-flex",
        ),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_MOVING): binary_sensor.binary_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:motion",
        ),
        cv.Optional(CONF_TARGET_POSITION): number.number_schema(
            STS3215PositionNumber,
            unit_of_measurement=UNIT_DEGREES,
            icon="mdi:angle-acute",
        ),
        cv.Optional(CONF_SPEED_LIMIT): number.number_schema(
            STS3215SpeedNumber,
            unit_of_measurement="°/s",
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:speedometer",
        ),
        cv.Optional(CONF_ACCELERATION): number.number_schema(
            STS3215AccelerationNumber,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:run-fast",
        ),
        cv.Optional(CONF_TORQUE_LIMIT): number.number_schema(
            STS3215TorqueLimitNumber,
            unit_of_measurement=UNIT_PERCENT,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:arm-flex",
        ),
        cv.Optional(CONF_TORQUE_ENABLED): switch.switch_schema(
            STS3215TorqueSwitch,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:engine",
        ),
    }
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(STS3215Component),
            cv.Required(CONF_SERVOS): cv.All(
                cv.ensure_list(SERVO_SCHEMA), cv.Length(min=1)
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.polling_component_schema("1s")),
    _unique_servo_ids,
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "sts3215",
    baud_rate=1_000_000,
    require_tx=True,
    require_rx=True,
    data_bits=8,
    parity="NONE",
    stop_bits=1,
)


async def _new_number(config, cls, parent, servo_id, min_value, max_value, step):
    var = await number.new_number(
        config, min_value=min_value, max_value=max_value, step=step
    )
    cg.add(var.set_parent(parent))
    cg.add(var.set_servo_id(servo_id))
    return var


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    for servo_config in config[CONF_SERVOS]:
        servo_id = servo_config[CONF_SERVO_ID]
        cg.add(var.add_servo(servo_id, servo_config[CONF_INVERTED]))

        for key, setter in (
            (CONF_POSITION, "set_position_sensor"),
            (CONF_POSITION_RAW, "set_position_raw_sensor"),
            (CONF_SPEED, "set_speed_sensor"),
            (CONF_LOAD, "set_load_sensor"),
            (CONF_TEMPERATURE, "set_temperature_sensor"),
            (CONF_VOLTAGE, "set_voltage_sensor"),
            (CONF_CURRENT, "set_current_sensor"),
        ):
            if key in servo_config:
                sens = await sensor.new_sensor(servo_config[key])
                cg.add(getattr(var, setter)(servo_id, sens))

        if CONF_MOVING in servo_config:
            moving = await binary_sensor.new_binary_sensor(servo_config[CONF_MOVING])
            cg.add(var.set_moving_sensor(servo_id, moving))

        if CONF_TARGET_POSITION in servo_config:
            target = await _new_number(
                servo_config[CONF_TARGET_POSITION],
                STS3215PositionNumber,
                var,
                servo_id,
                0.0,
                360.0,
                0.1,
            )
            cg.add(var.set_target_position_number(servo_id, target))

        if CONF_SPEED_LIMIT in servo_config:
            speed_limit = await _new_number(
                servo_config[CONF_SPEED_LIMIT],
                STS3215SpeedNumber,
                var,
                servo_id,
                0.0,
                360.0,
                1.0,
            )
            cg.add(var.set_speed_limit_number(servo_id, speed_limit))

        if CONF_ACCELERATION in servo_config:
            acceleration = await _new_number(
                servo_config[CONF_ACCELERATION],
                STS3215AccelerationNumber,
                var,
                servo_id,
                0.0,
                254.0,
                1.0,
            )
            cg.add(var.set_acceleration_number(servo_id, acceleration))

        if CONF_TORQUE_LIMIT in servo_config:
            torque_limit = await _new_number(
                servo_config[CONF_TORQUE_LIMIT],
                STS3215TorqueLimitNumber,
                var,
                servo_id,
                0.0,
                100.0,
                1.0,
            )
            cg.add(var.set_torque_limit_number(servo_id, torque_limit))

        if CONF_TORQUE_ENABLED in servo_config:
            torque_switch = await switch.new_switch(servo_config[CONF_TORQUE_ENABLED])
            cg.add(torque_switch.set_parent(var))
            cg.add(torque_switch.set_servo_id(servo_id))
            cg.add(var.set_torque_switch(servo_id, torque_switch))


STEP_ACTION_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(STS3215Component),
        cv.Required(CONF_SERVO_ID): cv.templatable(cv.int_range(min=0, max=253)),
        cv.Required(CONF_DEGREES): cv.templatable(cv.float_),
    }
)


@automation.register_action(
    "sts3215.step", STS3215StepAction, STEP_ACTION_SCHEMA, synchronous=True
)
async def sts3215_step_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    servo_id = await cg.templatable(config[CONF_SERVO_ID], args, cg.uint8)
    degrees = await cg.templatable(config[CONF_DEGREES], args, cg.float_)
    cg.add(var.set_servo_id(servo_id))
    cg.add(var.set_degrees(degrees))
    return var

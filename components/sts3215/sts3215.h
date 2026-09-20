#pragma once

#include <cstdint>
#include <vector>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace sts3215 {

class STS3215Component;

class STS3215PositionNumber : public number::Number {
 public:
  void set_parent(STS3215Component *parent) { this->parent_ = parent; }
  void set_servo_id(uint8_t servo_id) { this->servo_id_ = servo_id; }

 protected:
  void control(float value) override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
};

class STS3215SpeedNumber : public number::Number {
 public:
  void set_parent(STS3215Component *parent) { this->parent_ = parent; }
  void set_servo_id(uint8_t servo_id) { this->servo_id_ = servo_id; }

 protected:
  void control(float value) override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
};

class STS3215AccelerationNumber : public number::Number {
 public:
  void set_parent(STS3215Component *parent) { this->parent_ = parent; }
  void set_servo_id(uint8_t servo_id) { this->servo_id_ = servo_id; }

 protected:
  void control(float value) override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
};

class STS3215TorqueLimitNumber : public number::Number {
 public:
  void set_parent(STS3215Component *parent) { this->parent_ = parent; }
  void set_servo_id(uint8_t servo_id) { this->servo_id_ = servo_id; }

 protected:
  void control(float value) override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
};

class STS3215TorqueSwitch : public switch_::Switch {
 public:
  void set_parent(STS3215Component *parent) { this->parent_ = parent; }
  void set_servo_id(uint8_t servo_id) { this->servo_id_ = servo_id; }

 protected:
  void write_state(bool state) override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
};

struct STS3215Servo {
  uint8_t id;
  bool inverted;
  bool has_position{false};
  int16_t position_raw{0};
  uint16_t speed_limit_raw{0};
  uint8_t acceleration_raw{0};

  sensor::Sensor *position_sensor{nullptr};
  sensor::Sensor *position_raw_sensor{nullptr};
  sensor::Sensor *speed_sensor{nullptr};
  sensor::Sensor *load_sensor{nullptr};
  sensor::Sensor *temperature_sensor{nullptr};
  sensor::Sensor *voltage_sensor{nullptr};
  sensor::Sensor *current_sensor{nullptr};
  binary_sensor::BinarySensor *moving_sensor{nullptr};
  STS3215PositionNumber *target_position_number{nullptr};
  STS3215SpeedNumber *speed_limit_number{nullptr};
  STS3215AccelerationNumber *acceleration_number{nullptr};
  STS3215TorqueLimitNumber *torque_limit_number{nullptr};
  STS3215TorqueSwitch *torque_switch{nullptr};
};

class STS3215Component : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void add_servo(uint8_t servo_id, bool inverted);
  void set_position_sensor(uint8_t servo_id, sensor::Sensor *value);
  void set_position_raw_sensor(uint8_t servo_id, sensor::Sensor *value);
  void set_speed_sensor(uint8_t servo_id, sensor::Sensor *value);
  void set_load_sensor(uint8_t servo_id, sensor::Sensor *value);
  void set_temperature_sensor(uint8_t servo_id, sensor::Sensor *value);
  void set_voltage_sensor(uint8_t servo_id, sensor::Sensor *value);
  void set_current_sensor(uint8_t servo_id, sensor::Sensor *value);
  void set_moving_sensor(uint8_t servo_id, binary_sensor::BinarySensor *value);
  void set_target_position_number(uint8_t servo_id, STS3215PositionNumber *value);
  void set_speed_limit_number(uint8_t servo_id, STS3215SpeedNumber *value);
  void set_acceleration_number(uint8_t servo_id, STS3215AccelerationNumber *value);
  void set_torque_limit_number(uint8_t servo_id, STS3215TorqueLimitNumber *value);
  void set_torque_switch(uint8_t servo_id, STS3215TorqueSwitch *value);

  bool command_position(uint8_t servo_id, float degrees);
  bool step(uint8_t servo_id, float degrees);
  bool set_speed_limit(uint8_t servo_id, float degrees_per_second);
  bool set_acceleration(uint8_t servo_id, float value);
  bool set_torque_limit(uint8_t servo_id, float percent);
  bool set_torque_enabled(uint8_t servo_id, bool enabled);

 protected:
  static constexpr uint8_t INST_READ = 0x02;
  static constexpr uint8_t INST_WRITE = 0x03;
  static constexpr uint8_t REG_TORQUE_ENABLE = 40;
  static constexpr uint8_t REG_ACCELERATION = 41;
  static constexpr uint8_t REG_GOAL_SPEED = 46;
  static constexpr uint8_t REG_TORQUE_LIMIT = 48;
  static constexpr uint8_t REG_PRESENT_POSITION = 56;
  static constexpr float STEPS_PER_REVOLUTION = 4096.0f;
  static constexpr uint16_t MAX_POSITION = 4095;

  STS3215Servo *find_servo_(uint8_t servo_id);
  bool read_register_(uint8_t servo_id, uint8_t address, uint8_t *data, uint8_t length);
  bool write_register_(uint8_t servo_id, uint8_t address, const uint8_t *data, uint8_t length);
  bool read_status_packet_(uint8_t expected_id, uint8_t *data, uint8_t expected_length);
  bool read_byte_timeout_(uint8_t *data, uint32_t deadline);
  void clear_rx_();
  void poll_servo_(STS3215Servo &servo);

  static uint16_t decode_u16_(const uint8_t *data);
  static int16_t decode_signed_(uint16_t value, uint8_t sign_bit);
  static uint16_t encode_position_(float degrees, bool inverted);
  static float decode_position_degrees_(int16_t raw, bool inverted);
  static uint16_t speed_to_raw_(float degrees_per_second);
  static float speed_to_degrees_(int16_t raw, bool inverted);

  std::vector<STS3215Servo> servos_;
  uint32_t response_timeout_ms_{20};
};

template<typename... Ts> class STS3215StepAction : public Action<Ts...>, public Parented<STS3215Component> {
 public:
  TEMPLATABLE_VALUE(uint8_t, servo_id)
  TEMPLATABLE_VALUE(float, degrees)

  void play(Ts... x) override { this->parent_->step(this->servo_id_.value(x...), this->degrees_.value(x...)); }
};

}  // namespace sts3215
}  // namespace esphome

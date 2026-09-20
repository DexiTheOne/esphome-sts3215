#include "sts3215.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace sts3215 {

static const char *const TAG = "sts3215";

void STS3215Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up STS3215 bus with %u servo(s)...", static_cast<unsigned>(this->servos_.size()));
  this->clear_rx_();
}

void STS3215Component::dump_config() {
  ESP_LOGCONFIG(TAG, "STS3215:");
  LOG_UPDATE_INTERVAL(this);
  for (const auto &servo : this->servos_) {
    ESP_LOGCONFIG(TAG, "  Servo ID %u%s", servo.id, servo.inverted ? " (inverted)" : "");
  }
}

void STS3215Component::update() {
  for (auto &servo : this->servos_)
    this->poll_servo_(servo);
}

void STS3215Component::add_servo(uint8_t servo_id, bool inverted) {
  this->servos_.push_back({servo_id, inverted});
}

STS3215Servo *STS3215Component::find_servo_(uint8_t servo_id) {
  for (auto &servo : this->servos_) {
    if (servo.id == servo_id)
      return &servo;
  }
  ESP_LOGE(TAG, "Unknown servo ID %u", servo_id);
  return nullptr;
}

#define STS_SETTER(method, member, type) \
  void STS3215Component::method(uint8_t servo_id, type *value) { \
    auto *servo = this->find_servo_(servo_id); \
    if (servo != nullptr) \
      servo->member = value; \
  }

STS_SETTER(set_position_sensor, position_sensor, sensor::Sensor)
STS_SETTER(set_position_raw_sensor, position_raw_sensor, sensor::Sensor)
STS_SETTER(set_speed_sensor, speed_sensor, sensor::Sensor)
STS_SETTER(set_load_sensor, load_sensor, sensor::Sensor)
STS_SETTER(set_temperature_sensor, temperature_sensor, sensor::Sensor)
STS_SETTER(set_voltage_sensor, voltage_sensor, sensor::Sensor)
STS_SETTER(set_current_sensor, current_sensor, sensor::Sensor)
STS_SETTER(set_moving_sensor, moving_sensor, binary_sensor::BinarySensor)
STS_SETTER(set_target_position_number, target_position_number, STS3215PositionNumber)
STS_SETTER(set_speed_limit_number, speed_limit_number, STS3215SpeedNumber)
STS_SETTER(set_acceleration_number, acceleration_number, STS3215AccelerationNumber)
STS_SETTER(set_torque_limit_number, torque_limit_number, STS3215TorqueLimitNumber)
STS_SETTER(set_torque_switch, torque_switch, STS3215TorqueSwitch)

#undef STS_SETTER

uint16_t STS3215Component::decode_u16_(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

int16_t STS3215Component::decode_signed_(uint16_t value, uint8_t sign_bit) {
  const uint16_t mask = static_cast<uint16_t>(1U << sign_bit);
  const int16_t magnitude = static_cast<int16_t>(value & ~mask);
  return (value & mask) != 0 ? -magnitude : magnitude;
}

uint16_t STS3215Component::encode_position_(float degrees, bool inverted) {
  degrees = std::max(0.0f, std::min(360.0f, degrees));
  uint16_t raw = static_cast<uint16_t>(std::lround(degrees * MAX_POSITION / 360.0f));
  return inverted ? MAX_POSITION - raw : raw;
}

float STS3215Component::decode_position_degrees_(int16_t raw, bool inverted) {
  float degrees = static_cast<float>(raw) * 360.0f / STEPS_PER_REVOLUTION;
  if (inverted)
    degrees = 360.0f - degrees;
  return degrees;
}

uint16_t STS3215Component::speed_to_raw_(float degrees_per_second) {
  if (degrees_per_second <= 0.0f)
    return 0;  // STS convention: zero means unlimited speed.
  return static_cast<uint16_t>(std::min(4095.0f, std::round(degrees_per_second * STEPS_PER_REVOLUTION / 360.0f)));
}

float STS3215Component::speed_to_degrees_(int16_t raw, bool inverted) {
  float value = static_cast<float>(raw) * 360.0f / STEPS_PER_REVOLUTION;
  return inverted ? -value : value;
}

void STS3215Component::clear_rx_() {
  uint8_t ignored;
  while (this->available() && this->read_byte(&ignored)) {
  }
}

bool STS3215Component::read_byte_timeout_(uint8_t *data, uint32_t deadline) {
  while (static_cast<int32_t>(deadline - millis()) > 0) {
    if (this->available() && this->read_byte(data))
      return true;
    delay(0);
  }
  return false;
}

bool STS3215Component::read_status_packet_(uint8_t expected_id, uint8_t *data, uint8_t expected_length) {
  const uint32_t deadline = millis() + this->response_timeout_ms_;
  uint8_t byte = 0;
  uint8_t previous = 0;
  bool header_found = false;
  while (this->read_byte_timeout_(&byte, deadline)) {
    if (previous == 0xFF && byte == 0xFF) {
      header_found = true;
      break;
    }
    previous = byte;
  }
  if (!header_found)
    return false;

  uint8_t id;
  uint8_t packet_length;
  uint8_t error;
  if (!this->read_byte_timeout_(&id, deadline) || !this->read_byte_timeout_(&packet_length, deadline) ||
      !this->read_byte_timeout_(&error, deadline))
    return false;
  if (id != expected_id || packet_length != expected_length + 2) {
    ESP_LOGW(TAG, "Unexpected status packet (ID %u, length %u)", id, packet_length);
    return false;
  }

  uint8_t checksum_sum = id + packet_length + error;
  for (uint8_t i = 0; i < expected_length; i++) {
    if (!this->read_byte_timeout_(&data[i], deadline))
      return false;
    checksum_sum += data[i];
  }
  uint8_t received_checksum;
  if (!this->read_byte_timeout_(&received_checksum, deadline))
    return false;
  if (static_cast<uint8_t>(~checksum_sum) != received_checksum) {
    ESP_LOGW(TAG, "Checksum error in response from servo %u", expected_id);
    return false;
  }
  if (error != 0)
    ESP_LOGW(TAG, "Servo %u returned status flags 0x%02X", expected_id, error);
  return true;
}

bool STS3215Component::read_register_(uint8_t servo_id, uint8_t address, uint8_t *data, uint8_t length) {
  this->clear_rx_();
  const uint8_t packet_length = 4;
  const uint8_t checksum = static_cast<uint8_t>(~(servo_id + packet_length + INST_READ + address + length));
  const uint8_t packet[] = {0xFF, 0xFF, servo_id, packet_length, INST_READ, address, length, checksum};
  this->write_array(packet, sizeof(packet));
  this->flush();
  return this->read_status_packet_(servo_id, data, length);
}

bool STS3215Component::write_register_(uint8_t servo_id, uint8_t address, const uint8_t *data, uint8_t length) {
  this->clear_rx_();
  std::vector<uint8_t> packet;
  packet.reserve(length + 7);
  const uint8_t packet_length = length + 3;
  uint8_t checksum_sum = servo_id + packet_length + INST_WRITE + address;
  packet.insert(packet.end(), {0xFF, 0xFF, servo_id, packet_length, INST_WRITE, address});
  for (uint8_t i = 0; i < length; i++) {
    packet.push_back(data[i]);
    checksum_sum += data[i];
  }
  packet.push_back(static_cast<uint8_t>(~checksum_sum));
  this->write_array(packet.data(), packet.size());
  this->flush();
  return true;
}

void STS3215Component::poll_servo_(STS3215Servo &servo) {
  uint8_t settings[10];
  if (this->read_register_(servo.id, REG_TORQUE_ENABLE, settings, sizeof(settings))) {
    servo.acceleration_raw = settings[1];
    servo.speed_limit_raw = decode_u16_(&settings[6]);
    if (servo.torque_switch != nullptr)
      servo.torque_switch->publish_state(settings[0] != 0);
    if (servo.acceleration_number != nullptr)
      servo.acceleration_number->publish_state(servo.acceleration_raw);
    if (servo.speed_limit_number != nullptr)
      servo.speed_limit_number->publish_state(servo.speed_limit_raw * 360.0f / STEPS_PER_REVOLUTION);
    if (servo.torque_limit_number != nullptr)
      servo.torque_limit_number->publish_state(decode_u16_(&settings[8]) / 10.0f);
    if (servo.target_position_number != nullptr) {
      const int16_t target_raw = decode_signed_(decode_u16_(&settings[2]), 15);
      servo.target_position_number->publish_state(decode_position_degrees_(target_raw, servo.inverted));
    }
  } else {
    ESP_LOGW(TAG, "No settings response from servo %u", servo.id);
  }

  uint8_t feedback[15];
  if (!this->read_register_(servo.id, REG_PRESENT_POSITION, feedback, sizeof(feedback))) {
    ESP_LOGW(TAG, "No telemetry response from servo %u", servo.id);
    this->status_set_warning();
    return;
  }

  servo.position_raw = decode_signed_(decode_u16_(&feedback[0]), 15);
  servo.has_position = true;
  const int16_t speed_raw = decode_signed_(decode_u16_(&feedback[2]), 15);
  const int16_t load_raw = decode_signed_(decode_u16_(&feedback[4]), 10);
  const int16_t current_raw = decode_signed_(decode_u16_(&feedback[13]), 15);

  if (servo.position_sensor != nullptr)
    servo.position_sensor->publish_state(decode_position_degrees_(servo.position_raw, servo.inverted));
  if (servo.position_raw_sensor != nullptr)
    servo.position_raw_sensor->publish_state(servo.position_raw);
  if (servo.speed_sensor != nullptr)
    servo.speed_sensor->publish_state(speed_to_degrees_(speed_raw, servo.inverted));
  if (servo.load_sensor != nullptr)
    servo.load_sensor->publish_state((servo.inverted ? -load_raw : load_raw) / 10.0f);
  if (servo.voltage_sensor != nullptr)
    servo.voltage_sensor->publish_state(feedback[6] / 10.0f);
  if (servo.temperature_sensor != nullptr)
    servo.temperature_sensor->publish_state(feedback[7]);
  if (servo.moving_sensor != nullptr)
    servo.moving_sensor->publish_state(feedback[10] != 0);
  if (servo.current_sensor != nullptr)
    servo.current_sensor->publish_state(std::abs(current_raw) * 0.0065f);
  this->status_clear_warning();
}

bool STS3215Component::command_position(uint8_t servo_id, float degrees) {
  auto *servo = this->find_servo_(servo_id);
  if (servo == nullptr)
    return false;
  const uint16_t position = encode_position_(degrees, servo->inverted);
  const uint8_t data[] = {
      servo->acceleration_raw,
      static_cast<uint8_t>(position & 0xFF), static_cast<uint8_t>(position >> 8),
      0, 0,
      static_cast<uint8_t>(servo->speed_limit_raw & 0xFF), static_cast<uint8_t>(servo->speed_limit_raw >> 8),
  };
  return this->write_register_(servo_id, REG_ACCELERATION, data, sizeof(data));
}

bool STS3215Component::step(uint8_t servo_id, float degrees) {
  auto *servo = this->find_servo_(servo_id);
  if (servo == nullptr)
    return false;
  if (!servo->has_position) {
    uint8_t data[2];
    if (!this->read_register_(servo_id, REG_PRESENT_POSITION, data, sizeof(data))) {
      ESP_LOGE(TAG, "Cannot step servo %u before its position is known", servo_id);
      return false;
    }
    servo->position_raw = decode_signed_(decode_u16_(data), 15);
    servo->has_position = true;
  }
  float current = decode_position_degrees_(servo->position_raw, servo->inverted);
  return this->command_position(servo_id, current + degrees);
}

bool STS3215Component::set_speed_limit(uint8_t servo_id, float degrees_per_second) {
  auto *servo = this->find_servo_(servo_id);
  if (servo == nullptr)
    return false;
  servo->speed_limit_raw = speed_to_raw_(degrees_per_second);
  const uint8_t data[] = {static_cast<uint8_t>(servo->speed_limit_raw & 0xFF),
                          static_cast<uint8_t>(servo->speed_limit_raw >> 8)};
  return this->write_register_(servo_id, REG_GOAL_SPEED, data, sizeof(data));
}

bool STS3215Component::set_acceleration(uint8_t servo_id, float value) {
  auto *servo = this->find_servo_(servo_id);
  if (servo == nullptr)
    return false;
  servo->acceleration_raw = static_cast<uint8_t>(std::max(0.0f, std::min(254.0f, std::round(value))));
  return this->write_register_(servo_id, REG_ACCELERATION, &servo->acceleration_raw, 1);
}

bool STS3215Component::set_torque_limit(uint8_t servo_id, float percent) {
  percent = std::max(0.0f, std::min(100.0f, percent));
  const uint16_t raw = static_cast<uint16_t>(std::lround(percent * 10.0f));
  const uint8_t data[] = {static_cast<uint8_t>(raw & 0xFF), static_cast<uint8_t>(raw >> 8)};
  return this->write_register_(servo_id, REG_TORQUE_LIMIT, data, sizeof(data));
}

bool STS3215Component::set_torque_enabled(uint8_t servo_id, bool enabled) {
  if (this->find_servo_(servo_id) == nullptr)
    return false;
  const uint8_t value = enabled ? 1 : 0;
  return this->write_register_(servo_id, REG_TORQUE_ENABLE, &value, 1);
}

void STS3215PositionNumber::control(float value) {
  if (this->parent_ != nullptr && this->parent_->command_position(this->servo_id_, value))
    this->publish_state(value);
}

void STS3215SpeedNumber::control(float value) {
  if (this->parent_ != nullptr && this->parent_->set_speed_limit(this->servo_id_, value))
    this->publish_state(value);
}

void STS3215AccelerationNumber::control(float value) {
  if (this->parent_ != nullptr && this->parent_->set_acceleration(this->servo_id_, value))
    this->publish_state(value);
}

void STS3215TorqueLimitNumber::control(float value) {
  if (this->parent_ != nullptr && this->parent_->set_torque_limit(this->servo_id_, value))
    this->publish_state(value);
}

void STS3215TorqueSwitch::write_state(bool state) {
  if (this->parent_ != nullptr && this->parent_->set_torque_enabled(this->servo_id_, state))
    this->publish_state(state);
}

}  // namespace sts3215
}  // namespace esphome

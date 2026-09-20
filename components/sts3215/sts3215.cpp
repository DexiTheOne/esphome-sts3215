#include "sts3215.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace sts3215 {

static const char *const TAG = "sts3215";

void STS3215Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up STS3215 bus with %u servo(s)...", static_cast<unsigned>(servos_.size()));
  clear_rx_();
  for (auto &servo : servos_) {
    servo.preference = global_preferences->make_preference<STS3215PreferenceData>(servo.preference_key);
    load_preferences_(servo);
    update_commission_state_(servo, true);
    const uint8_t acceleration = servo.acceleration_raw;
    const uint8_t speed[] = {static_cast<uint8_t>(servo.speed_limit_raw),
                             static_cast<uint8_t>(servo.speed_limit_raw >> 8)};
    const uint8_t torque[] = {static_cast<uint8_t>(servo.torque_limit_raw),
                              static_cast<uint8_t>(servo.torque_limit_raw >> 8)};
    const uint8_t disabled = 0;
    write_register_(servo.id, REG_ACCELERATION, &acceleration, 1);
    write_register_(servo.id, REG_GOAL_SPEED, speed, 2);
    write_register_(servo.id, REG_TORQUE_LIMIT, torque, 2);
    write_register_(servo.id, REG_TORQUE_ENABLE, &disabled, 1);
    publish_settings_(servo);
  }
}

void STS3215Component::loop() {
  const uint32_t now = millis();
  if (!move_queue_.empty() &&
      (!has_started_move_ || static_cast<uint32_t>(now - last_move_started_) >= start_delay_ms_)) {
    const auto move = move_queue_.front();
    move_queue_.pop_front();
    auto *servo = find_servo_(move.servo_id);
    if (servo != nullptr)
      begin_move_(*servo, move.target_raw);
  }
}

void STS3215Component::dump_config() {
  ESP_LOGCONFIG(TAG, "STS3215:");
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Inter-motor start delay: %u ms", static_cast<unsigned>(start_delay_ms_));
  ESP_LOGCONFIG(TAG, "  Move timeout: %u ms", static_cast<unsigned>(move_timeout_ms_));
  for (const auto &servo : servos_)
    ESP_LOGCONFIG(TAG, "  Servo ID %u%s; calibration %s", servo.id,
                  servo.inverted ? " (inverted)" : "", calibrated_(servo) ? "complete" : "incomplete");
}

void STS3215Component::update() {
  for (auto &servo : servos_)
    poll_servo_(servo);
  update_group_cover_();
}

void STS3215Component::add_servo(uint8_t servo_id, bool inverted, uint32_t preference_key,
                                 float initial_speed, uint8_t initial_acceleration, float initial_torque) {
  servos_.push_back({servo_id, inverted, preference_key, initial_speed, initial_acceleration, initial_torque});
}

STS3215Servo *STS3215Component::find_servo_(uint8_t servo_id) {
  for (auto &servo : servos_) {
    if (servo.id == servo_id)
      return &servo;
  }
  ESP_LOGE(TAG, "Unknown servo ID %u", servo_id);
  return nullptr;
}

#define STS_SETTER(method, member, type) \
  void STS3215Component::method(uint8_t servo_id, type *value) { \
    auto *servo = find_servo_(servo_id); \
    if (servo != nullptr) servo->member = value; \
  }

STS_SETTER(set_position_sensor, position_sensor, sensor::Sensor)
STS_SETTER(set_position_raw_sensor, position_raw_sensor, sensor::Sensor)
STS_SETTER(set_speed_sensor, speed_sensor, sensor::Sensor)
STS_SETTER(set_load_sensor, load_sensor, sensor::Sensor)
STS_SETTER(set_temperature_sensor, temperature_sensor, sensor::Sensor)
STS_SETTER(set_voltage_sensor, voltage_sensor, sensor::Sensor)
STS_SETTER(set_current_sensor, current_sensor, sensor::Sensor)
STS_SETTER(set_status_sensor, status_sensor, sensor::Sensor)
STS_SETTER(set_moving_sensor, moving_sensor, binary_sensor::BinarySensor)
STS_SETTER(set_torque_sensor, torque_sensor, binary_sensor::BinarySensor)
STS_SETTER(set_multi_turn_sensor, multi_turn_sensor, binary_sensor::BinarySensor)
STS_SETTER(set_target_position_number, target_position_number, STS3215PositionNumber)
STS_SETTER(set_speed_limit_number, speed_limit_number, STS3215SpeedNumber)
STS_SETTER(set_acceleration_number, acceleration_number, STS3215AccelerationNumber)
STS_SETTER(set_torque_limit_number, torque_limit_number, STS3215TorqueLimitNumber)
STS_SETTER(set_jog_increment_number, jog_increment_number, STS3215JogIncrementNumber)
STS_SETTER(set_cover, cover, STS3215Cover)

#undef STS_SETTER

uint16_t STS3215Component::decode_u16_(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

int16_t STS3215Component::decode_signed_(uint16_t value, uint8_t sign_bit) {
  const uint16_t mask = static_cast<uint16_t>(1U << sign_bit);
  const int16_t magnitude = static_cast<int16_t>(value & ~mask);
  return (value & mask) ? -magnitude : magnitude;
}

uint16_t STS3215Component::encode_signed_(int32_t value) {
  value = std::max<int32_t>(-32767, std::min<int32_t>(32767, value));
  if (value < 0)
    return static_cast<uint16_t>((-value) | 0x8000);
  return static_cast<uint16_t>(value);
}

int32_t STS3215Component::degrees_to_raw_(float degrees, bool inverted) {
  int32_t raw = static_cast<int32_t>(std::lround(degrees * STEPS_PER_REVOLUTION / 360.0f));
  raw = std::max<int32_t>(-32767, std::min<int32_t>(32767, raw));
  return inverted ? -raw : raw;
}

float STS3215Component::raw_to_degrees_(int32_t raw, bool inverted) {
  const float degrees = static_cast<float>(raw) * 360.0f / STEPS_PER_REVOLUTION;
  return inverted ? -degrees : degrees;
}

uint16_t STS3215Component::speed_to_raw_(float degrees_per_second) {
  if (degrees_per_second <= 0.0f)
    return 0;
  return static_cast<uint16_t>(std::min(4095.0f,
      std::round(degrees_per_second * STEPS_PER_REVOLUTION / 360.0f)));
}

float STS3215Component::speed_to_degrees_(int16_t raw, bool inverted) {
  float value = static_cast<float>(raw) * 360.0f / STEPS_PER_REVOLUTION;
  return inverted ? -value : value;
}

void STS3215Component::clear_rx_() {
  uint8_t ignored;
  while (available() && read_byte(&ignored)) {}
}

bool STS3215Component::read_byte_timeout_(uint8_t *data, uint32_t deadline) {
  while (static_cast<int32_t>(deadline - millis()) > 0) {
    if (available() && read_byte(data))
      return true;
    delay(0);
  }
  return false;
}

bool STS3215Component::read_status_packet_(uint8_t expected_id, uint8_t *data, uint8_t expected_length) {
  const uint32_t deadline = millis() + response_timeout_ms_;
  uint8_t byte = 0, previous = 0;
  bool header_found = false;
  while (read_byte_timeout_(&byte, deadline)) {
    if (previous == 0xFF && byte == 0xFF) { header_found = true; break; }
    previous = byte;
  }
  if (!header_found)
    return false;
  uint8_t id, packet_length, error;
  if (!read_byte_timeout_(&id, deadline) || !read_byte_timeout_(&packet_length, deadline) ||
      !read_byte_timeout_(&error, deadline))
    return false;
  if (id != expected_id || packet_length != expected_length + 2) {
    ESP_LOGW(TAG, "Unexpected status packet (ID %u, length %u)", id, packet_length);
    return false;
  }
  uint8_t checksum_sum = id + packet_length + error;
  for (uint8_t i = 0; i < expected_length; i++) {
    if (!read_byte_timeout_(&data[i], deadline)) return false;
    checksum_sum += data[i];
  }
  uint8_t received_checksum;
  if (!read_byte_timeout_(&received_checksum, deadline)) return false;
  if (static_cast<uint8_t>(~checksum_sum) != received_checksum) {
    ESP_LOGW(TAG, "Checksum error in response from servo %u", expected_id);
    return false;
  }
  if (error != 0)
    ESP_LOGW(TAG, "Servo %u returned status flags 0x%02X", expected_id, error);
  return true;
}

bool STS3215Component::read_register_(uint8_t servo_id, uint8_t address, uint8_t *data, uint8_t length) {
  clear_rx_();
  const uint8_t packet_length = 4;
  const uint8_t checksum = static_cast<uint8_t>(~(servo_id + packet_length + INST_READ + address + length));
  const uint8_t packet[] = {0xFF, 0xFF, servo_id, packet_length, INST_READ, address, length, checksum};
  write_array(packet, sizeof(packet));
  flush();
  return read_status_packet_(servo_id, data, length);
}

bool STS3215Component::write_register_(uint8_t servo_id, uint8_t address,
                                       const uint8_t *data, uint8_t length) {
  clear_rx_();
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
  write_array(packet.data(), packet.size());
  flush();
  return true;
}

void STS3215Component::poll_servo_(STS3215Servo &servo) {
  uint8_t settings[10];
  if (read_register_(servo.id, REG_TORQUE_ENABLE, settings, sizeof(settings))) {
    if (servo.torque_sensor != nullptr)
      servo.torque_sensor->publish_state(settings[0] != 0);
  } else {
    ESP_LOGW(TAG, "No settings response from servo %u", servo.id);
  }

  uint8_t feedback[15];
  if (!read_register_(servo.id, REG_PRESENT_POSITION, feedback, sizeof(feedback))) {
    ESP_LOGW(TAG, "No telemetry response from servo %u", servo.id);
    status_set_warning();
    return;
  }

  set_hardware_position_(servo, decode_signed_(decode_u16_(&feedback[0]), 15));
  const int16_t speed_raw = decode_signed_(decode_u16_(&feedback[2]), 15);
  const int16_t load_raw = decode_signed_(decode_u16_(&feedback[4]), 10);
  const int16_t current_raw = decode_signed_(decode_u16_(&feedback[13]), 15);
  servo.moving = feedback[10] != 0;
  if (servo.command_active && servo.moving)
    servo.moving_seen = true;

  if (servo.position_sensor != nullptr)
    servo.position_sensor->publish_state(raw_to_degrees_(servo.position_raw, servo.inverted));
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
  if (servo.status_sensor != nullptr)
    servo.status_sensor->publish_state(feedback[9]);
  if (servo.moving_sensor != nullptr)
    servo.moving_sensor->publish_state(servo.moving);
  if (servo.current_sensor != nullptr)
    servo.current_sensor->publish_state(std::abs(current_raw) * 0.0065f);

  if (servo.command_active) {
    const uint32_t elapsed = millis() - servo.command_started;
    const bool arrived = std::abs(servo.position_raw - servo.target_raw) <= position_tolerance_;
    const bool stopped_after_motion = servo.moving_seen && !servo.moving;
    if ((elapsed >= 250 && arrived) || stopped_after_motion || elapsed >= move_timeout_ms_)
      finish_move_(servo, elapsed >= move_timeout_ms_ && !arrived);
  }
  update_cover_(servo);
  status_clear_warning();
}

void STS3215Component::begin_move_(STS3215Servo &servo, int32_t target_raw) {
  target_raw = std::max<int32_t>(-32767, std::min<int32_t>(32767, target_raw));
  int32_t hardware_target = target_raw - servo.position_offset;
  if (hardware_target < -32767 || hardware_target > 32767) {
    hardware_target = std::max<int32_t>(-32767, std::min<int32_t>(32767, hardware_target));
    target_raw = hardware_target + servo.position_offset;
    ESP_LOGW(TAG, "Servo %u target was clamped to its current multi-turn hardware window", servo.id);
  }
  servo.target_raw = target_raw;
  const uint8_t enabled = 1;
  write_register_(servo.id, REG_TORQUE_ENABLE, &enabled, 1);
  const uint16_t encoded = encode_signed_(hardware_target);
  const uint8_t data[] = {
      servo.acceleration_raw, static_cast<uint8_t>(encoded), static_cast<uint8_t>(encoded >> 8),
      0, 0, static_cast<uint8_t>(servo.speed_limit_raw), static_cast<uint8_t>(servo.speed_limit_raw >> 8)};
  write_register_(servo.id, REG_ACCELERATION, data, sizeof(data));
  servo.command_active = true;
  servo.moving_seen = false;
  servo.command_started = millis();
  last_move_started_ = servo.command_started;
  has_started_move_ = true;
  if (servo.torque_sensor != nullptr)
    servo.torque_sensor->publish_state(true);
  ESP_LOGD(TAG, "Servo %u moving to raw position %ld", servo.id, static_cast<long>(target_raw));
}

void STS3215Component::finish_move_(STS3215Servo &servo, bool timed_out) {
  const uint8_t disabled = 0;
  write_register_(servo.id, REG_TORQUE_ENABLE, &disabled, 1);
  servo.command_active = false;
  servo.moving_seen = false;
  if (servo.torque_sensor != nullptr)
    servo.torque_sensor->publish_state(false);
  if (timed_out)
    ESP_LOGW(TAG, "Servo %u move timed out; torque disabled", servo.id);
  if (servo.has_position)
    save_preferences_(servo);
  update_cover_(servo);
}

void STS3215Component::enqueue_move_(uint8_t servo_id, int32_t target_raw) {
  remove_queued_(servo_id);
  move_queue_.push_back({servo_id, std::max<int32_t>(-32767, std::min<int32_t>(32767, target_raw))});
}

void STS3215Component::remove_queued_(uint8_t servo_id) {
  move_queue_.erase(std::remove_if(move_queue_.begin(), move_queue_.end(),
      [servo_id](const STS3215QueuedMove &move) { return move.servo_id == servo_id; }), move_queue_.end());
}

bool STS3215Component::command_position(uint8_t servo_id, float degrees) {
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return false;
  enqueue_move_(servo_id, degrees_to_raw_(degrees, servo->inverted));
  return true;
}

bool STS3215Component::step(uint8_t servo_id, float degrees) {
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return false;
  if (!servo->has_position) {
    uint8_t data[2];
    if (!read_register_(servo_id, REG_PRESENT_POSITION, data, sizeof(data))) {
      ESP_LOGE(TAG, "Cannot jog servo %u before its position is known", servo_id);
      return false;
    }
    set_hardware_position_(*servo, decode_signed_(decode_u16_(data), 15));
  }
  const int32_t delta = degrees_to_raw_(degrees, servo->inverted);
  enqueue_move_(servo_id, servo->position_raw + delta);
  return true;
}

bool STS3215Component::set_speed_limit(uint8_t servo_id, float value) {
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return false;
  servo->speed_limit_display = std::round(std::max(0.0f, std::min(360.0f, value)));
  servo->speed_limit_raw = speed_to_raw_(servo->speed_limit_display);
  const uint8_t data[] = {static_cast<uint8_t>(servo->speed_limit_raw),
                          static_cast<uint8_t>(servo->speed_limit_raw >> 8)};
  write_register_(servo_id, REG_GOAL_SPEED, data, 2);
  save_preferences_(*servo);
  return true;
}

bool STS3215Component::set_acceleration(uint8_t servo_id, float value) {
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return false;
  servo->acceleration_raw = static_cast<uint8_t>(std::max(0.0f, std::min(254.0f, std::round(value))));
  write_register_(servo_id, REG_ACCELERATION, &servo->acceleration_raw, 1);
  save_preferences_(*servo);
  return true;
}

bool STS3215Component::set_torque_limit(uint8_t servo_id, float value) {
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return false;
  servo->torque_limit_display = std::round(std::max(0.0f, std::min(100.0f, value)));
  servo->torque_limit_raw = static_cast<uint16_t>(servo->torque_limit_display * 10.0f);
  const uint8_t data[] = {static_cast<uint8_t>(servo->torque_limit_raw),
                          static_cast<uint8_t>(servo->torque_limit_raw >> 8)};
  write_register_(servo_id, REG_TORQUE_LIMIT, data, 2);
  save_preferences_(*servo);
  return true;
}

bool STS3215Component::set_jog_increment(uint8_t servo_id, float value) {
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return false;
  servo->jog_increment = std::round(std::max(0.1f, std::min(2520.0f, value)) * 10.0f) / 10.0f;
  save_preferences_(*servo);
  return true;
}

void STS3215Component::commission_step_mode(uint8_t servo_id) {
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return;

  ESP_LOGW(TAG, "Servo %u multi-turn commissioning requested", servo_id);
  if (update_commission_state_(*servo, true)) {
    ESP_LOGW(TAG, "Servo %u is already fully commissioned; EEPROM was not written", servo_id);
    return;
  }

  // This is deliberately reachable only from an explicit user button. Never
  // perform EEPROM writes automatically during setup or polling.
  stop_servo(servo_id);
  const uint8_t torque_off = 0;
  const uint8_t unlocked = 0;
  const uint8_t step_mode = 3;
  const uint8_t locked = 1;
  uint8_t phase = 0;
  if (!read_register_(servo_id, REG_PHASE, &phase, 1)) {
    ESP_LOGE(TAG, "Cannot commission servo %u: Phase register read failed", servo_id);
    return;
  }
  phase |= 0x10;
  const uint8_t limits[] = {0, 0, static_cast<uint8_t>(MULTI_TURN_MAX_POSITION),
                            static_cast<uint8_t>(MULTI_TURN_MAX_POSITION >> 8)};
  write_register_(servo_id, REG_TORQUE_ENABLE, &torque_off, 1);
  if (servo->torque_sensor != nullptr)
    servo->torque_sensor->publish_state(false);
  write_register_(servo_id, REG_EEPROM_LOCK, &unlocked, 1);
  delay(20);
  uint8_t lock_state = 0xFF;
  if (!read_register_(servo_id, REG_EEPROM_LOCK, &lock_state, 1) || lock_state != 0) {
    ESP_LOGE(TAG, "Servo %u EEPROM did not unlock (lock=%u)", servo_id, lock_state);
    return;
  }
  write_register_(servo_id, REG_MIN_ANGLE_LIMIT, limits, sizeof(limits));
  delay(20);
  write_register_(servo_id, REG_PHASE, &phase, 1);
  delay(20);
  write_register_(servo_id, REG_MODE, &step_mode, 1);
  delay(50);
  write_register_(servo_id, REG_EEPROM_LOCK, &locked, 1);
  delay(50);

  if (update_commission_state_(*servo, true)) {
    ESP_LOGW(TAG, "Servo %u commissioned successfully; power-cycle the complete node", servo_id);
  } else {
    ESP_LOGE(TAG, "Servo %u multi-turn commissioning verification failed", servo_id);
  }
}

bool STS3215Component::update_commission_state_(STS3215Servo &servo, bool log_result) {
  // Read EEPROM registers 9..33 in one transaction. Relevant offsets are:
  // minimum=0, maximum=2, Phase=9, and Operating_Mode=24.
  uint8_t config[25];
  if (!read_register_(servo.id, REG_MIN_ANGLE_LIMIT, config, sizeof(config))) {
    if (servo.multi_turn_sensor != nullptr)
      servo.multi_turn_sensor->publish_state(false);
    if (log_result)
      ESP_LOGE(TAG, "Servo %u multi-turn configuration read failed", servo.id);
    return false;
  }
  const uint16_t minimum = decode_u16_(&config[0]);
  const uint16_t maximum = decode_u16_(&config[2]);
  const uint8_t phase = config[9];
  const uint8_t mode = config[24];
  const bool ready = mode == 3 && (phase & 0x10) != 0 && minimum == 0 &&
                     maximum >= MULTI_TURN_MAX_POSITION;
  if (servo.multi_turn_sensor != nullptr)
    servo.multi_turn_sensor->publish_state(ready);
  if (log_result) {
    if (ready) {
      ESP_LOGI(TAG, "Servo %u multi-turn configuration verified: mode=%u phase=0x%02X limits=%u..%u",
               servo.id, mode, phase, minimum, maximum);
    } else {
      ESP_LOGW(TAG, "Servo %u multi-turn configuration incomplete: mode=%u phase=0x%02X limits=%u..%u",
               servo.id, mode, phase, minimum, maximum);
    }
  }
  return ready;
}

void STS3215Component::calibration_action(uint8_t servo_id, uint8_t action) {
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return;
  if (action == 0) { step(servo_id, servo->jog_increment); return; }
  if (action == 1) { step(servo_id, -servo->jog_increment); return; }
  if (action == 5) { commission_step_mode(servo_id); return; }
  if (!servo->has_position) {
    ESP_LOGW(TAG, "Cannot save calibration for servo %u until position telemetry is available", servo_id);
    return;
  }
  if (action == 2) { servo->calibration_down = servo->position_raw; servo->calibration_mask |= 0x01; }
  if (action == 3) { servo->calibration_middle = servo->position_raw; servo->calibration_mask |= 0x02; }
  if (action == 4) { servo->calibration_up = servo->position_raw; servo->calibration_mask |= 0x04; }
  save_preferences_(*servo);
  ESP_LOGI(TAG, "Saved servo %u calibration point at raw position %ld", servo_id,
           static_cast<long>(servo->position_raw));
  if (servo->calibration_mask == 0x07 && !calibrated_(*servo)) {
    const int32_t span = servo->calibration_up - servo->calibration_down;
    const int32_t middle_offset = servo->calibration_middle - servo->calibration_down;
    if (span == 0 || (span > 0 && (middle_offset <= 0 || middle_offset >= span)) ||
        (span < 0 && (middle_offset >= 0 || middle_offset <= span))) {
      ESP_LOGE(TAG, "Servo %u calibration invalid: middle must be strictly between down and up", servo_id);
    }
  }
  update_cover_(*servo);
}

void STS3215Component::command_cover(uint8_t servo_id, float position) {
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return;
  if (!calibrated_(*servo)) {
    ESP_LOGW(TAG, "Servo %u cover ignored: down/middle/up calibration is incomplete", servo_id);
    return;
  }
  position = std::max(0.0f, std::min(1.0f, position));
  enqueue_move_(servo_id, raw_for_cover_position_(*servo, position));
  if (servo->cover != nullptr) {
    const float current = cover_position_for_raw_(*servo, servo->position_raw);
    servo->cover->update_from_parent(current,
        position >= current ? cover::COVER_OPERATION_OPENING : cover::COVER_OPERATION_CLOSING);
  }
}

void STS3215Component::command_all_covers(float position) {
  for (auto &servo : servos_)
    if (calibrated_(servo))
      command_cover(servo.id, position);
}

void STS3215Component::stop_servo(uint8_t servo_id) {
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return;
  remove_queued_(servo_id);
  if (servo->command_active)
    finish_move_(*servo, false);
  update_cover_(*servo);
}

void STS3215Component::stop_all() {
  move_queue_.clear();
  for (auto &servo : servos_)
    if (servo.command_active)
      finish_move_(servo, false);
}

void STS3215Component::load_preferences_(STS3215Servo &servo) {
  STS3215PreferenceData data{};
  if (servo.preference.load(&data) && data.version == PREFERENCE_VERSION) {
    servo.speed_limit_display = data.speed_limit;
    servo.torque_limit_display = data.torque_limit;
    servo.jog_increment = data.jog_increment;
    servo.acceleration_raw = data.acceleration;
    servo.calibration_down = data.down;
    servo.calibration_middle = data.middle;
    servo.calibration_up = data.up;
    servo.calibration_mask = data.calibration_mask;
    servo.saved_position = data.last_position;
    servo.saved_position_valid = data.last_position_valid != 0;
  } else {
    servo.speed_limit_display = std::round(servo.default_speed);
    servo.torque_limit_display = std::round(servo.default_torque);
    servo.acceleration_raw = servo.default_acceleration;
    servo.jog_increment = 10.0f;
  }
  servo.speed_limit_raw = speed_to_raw_(servo.speed_limit_display);
  servo.torque_limit_raw = static_cast<uint16_t>(servo.torque_limit_display * 10.0f);
}

void STS3215Component::save_preferences_(STS3215Servo &servo) {
  if (servo.has_position) {
    servo.saved_position = servo.position_raw;
    servo.saved_position_valid = true;
  }
  const STS3215PreferenceData data = {
      PREFERENCE_VERSION, servo.speed_limit_display, servo.torque_limit_display, servo.jog_increment,
      servo.calibration_down, servo.calibration_middle, servo.calibration_up,
      servo.saved_position, servo.acceleration_raw, servo.calibration_mask,
      static_cast<uint8_t>(servo.saved_position_valid), 0};
  if (!servo.preference.save(&data))
    ESP_LOGW(TAG, "Failed to save preferences for servo %u", servo.id);
  else
    global_preferences->sync();
}

void STS3215Component::set_hardware_position_(STS3215Servo &servo, int32_t hardware_position) {
  servo.hardware_position_raw = hardware_position;
  if (!servo.has_position) {
    // The servo's multi-turn counter can reset at power loss. The worm drive
    // cannot back-drive while off, so the last stopped position defines the
    // correct turn-number offset for this boot.
    servo.position_offset = servo.saved_position_valid
        ? static_cast<int32_t>(std::lround(static_cast<float>(servo.saved_position - hardware_position) /
                                           STEPS_PER_REVOLUTION)) * static_cast<int32_t>(STEPS_PER_REVOLUTION)
        : 0;
    servo.has_position = true;
  }
  servo.position_raw = hardware_position + servo.position_offset;
}

void STS3215Component::publish_settings_(STS3215Servo &servo) {
  if (servo.speed_limit_number != nullptr) servo.speed_limit_number->publish_state(servo.speed_limit_display);
  if (servo.acceleration_number != nullptr) servo.acceleration_number->publish_state(servo.acceleration_raw);
  if (servo.torque_limit_number != nullptr) servo.torque_limit_number->publish_state(servo.torque_limit_display);
  if (servo.jog_increment_number != nullptr) servo.jog_increment_number->publish_state(servo.jog_increment);
}

int32_t STS3215Component::raw_for_cover_position_(const STS3215Servo &servo, float position) const {
  if (position <= 0.5f)
    return static_cast<int32_t>(std::lround(servo.calibration_down +
        (servo.calibration_middle - servo.calibration_down) * (position * 2.0f)));
  return static_cast<int32_t>(std::lround(servo.calibration_middle +
      (servo.calibration_up - servo.calibration_middle) * ((position - 0.5f) * 2.0f)));
}

float STS3215Component::cover_position_for_raw_(const STS3215Servo &servo, int32_t raw) const {
  const int32_t first = servo.calibration_middle - servo.calibration_down;
  const int32_t second = servo.calibration_up - servo.calibration_middle;
  if (first == 0 || second == 0) return 0.0f;
  const bool before_middle = first > 0 ? raw <= servo.calibration_middle : raw >= servo.calibration_middle;
  float result;
  if (before_middle)
    result = 0.5f * static_cast<float>(raw - servo.calibration_down) / static_cast<float>(first);
  else
    result = 0.5f + 0.5f * static_cast<float>(raw - servo.calibration_middle) / static_cast<float>(second);
  return std::max(0.0f, std::min(1.0f, result));
}

void STS3215Component::update_cover_(STS3215Servo &servo) {
  if (servo.cover == nullptr || !servo.has_position || !calibrated_(servo)) return;
  bool pending = servo.command_active;
  int32_t pending_target = servo.target_raw;
  if (!pending) {
    const auto queued = std::find_if(move_queue_.begin(), move_queue_.end(),
        [&servo](const STS3215QueuedMove &move) { return move.servo_id == servo.id; });
    if (queued != move_queue_.end()) {
      pending = true;
      pending_target = queued->target_raw;
    }
  }
  cover::CoverOperation operation = cover::COVER_OPERATION_IDLE;
  if (pending) {
    const float current = cover_position_for_raw_(servo, servo.position_raw);
    const float target = cover_position_for_raw_(servo, pending_target);
    operation = target >= current ? cover::COVER_OPERATION_OPENING : cover::COVER_OPERATION_CLOSING;
  }
  servo.cover->update_from_parent(cover_position_for_raw_(servo, servo.position_raw), operation);
}

void STS3215Component::update_group_cover_() {
  if (group_cover_ == nullptr) return;
  float sum = 0.0f;
  size_t count = 0;
  bool opening = false, closing = false;
  for (const auto &servo : servos_) {
    if (!servo.has_position || !calibrated_(servo)) continue;
    sum += cover_position_for_raw_(servo, servo.position_raw);
    count++;
    int32_t target = servo.target_raw;
    bool pending = servo.command_active;
    if (!pending) {
      const auto queued = std::find_if(move_queue_.begin(), move_queue_.end(),
          [&servo](const STS3215QueuedMove &move) { return move.servo_id == servo.id; });
      if (queued != move_queue_.end()) {
        pending = true;
        target = queued->target_raw;
      }
    }
    if (pending) {
      const float current_position = cover_position_for_raw_(servo, servo.position_raw);
      const float target_position = cover_position_for_raw_(servo, target);
      opening |= target_position >= current_position;
      closing |= target_position < current_position;
    }
  }
  if (count == 0) return;
  const auto operation = opening ? cover::COVER_OPERATION_OPENING :
                         closing ? cover::COVER_OPERATION_CLOSING : cover::COVER_OPERATION_IDLE;
  group_cover_->update_from_parent(sum / count, operation);
}

cover::CoverTraits STS3215Cover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  traits.set_is_assumed_state(false);
  return traits;
}

void STS3215Cover::control(const cover::CoverCall &call) {
  if (call.get_stop()) parent_->stop_servo(servo_id_);
  if (call.get_position().has_value()) parent_->command_cover(servo_id_, *call.get_position());
}

void STS3215Cover::update_from_parent(float value, cover::CoverOperation operation) {
  position = value;
  current_operation = operation;
  publish_state(false);
}

cover::CoverTraits STS3215GroupCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  traits.set_is_assumed_state(false);
  return traits;
}

void STS3215GroupCover::control(const cover::CoverCall &call) {
  if (call.get_stop()) parent_->stop_all();
  if (call.get_position().has_value()) parent_->command_all_covers(*call.get_position());
}

void STS3215GroupCover::update_from_parent(float value, cover::CoverOperation operation) {
  position = value;
  current_operation = operation;
  publish_state(false);
}

void STS3215CalibrationButton::press_action() {
  if (parent_ != nullptr) parent_->calibration_action(servo_id_, action_);
}

void STS3215PositionNumber::control(float value) {
  const float clean = std::round(value * 10.0f) / 10.0f;
  if (parent_ != nullptr && parent_->command_position(servo_id_, clean)) publish_state(clean);
}

void STS3215SpeedNumber::control(float value) {
  const float clean = std::round(value);
  if (parent_ != nullptr && parent_->set_speed_limit(servo_id_, clean)) publish_state(clean);
}

void STS3215AccelerationNumber::control(float value) {
  const float clean = std::round(value);
  if (parent_ != nullptr && parent_->set_acceleration(servo_id_, clean)) publish_state(clean);
}

void STS3215TorqueLimitNumber::control(float value) {
  const float clean = std::round(value);
  if (parent_ != nullptr && parent_->set_torque_limit(servo_id_, clean)) publish_state(clean);
}

void STS3215JogIncrementNumber::control(float value) {
  const float clean = std::round(value * 10.0f) / 10.0f;
  if (parent_ != nullptr && parent_->set_jog_increment(servo_id_, clean)) publish_state(clean);
}

}  // namespace sts3215
}  // namespace esphome

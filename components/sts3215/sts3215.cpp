#include "sts3215.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"
#include "esphome/core/alloc_helpers.h"

namespace esphome {
namespace sts3215 {

static const char *const TAG = "sts3215";

static float tilt_openness(float tilt) {
  tilt = std::max(0.0f, std::min(1.0f, tilt));
  return 1.0f - std::abs(tilt * 2.0f - 1.0f);
}

static cover::CoverOperation operation_for_target(float tilt) {
  return tilt <= 0.0f || tilt >= 1.0f
      ? cover::COVER_OPERATION_CLOSING : cover::COVER_OPERATION_OPENING;
}

static float next_cover_quarter(float tilt, bool increase) {
  constexpr int QUARTER_COUNT = 4;
  const float scaled = std::max(0.0f, std::min(1.0f, tilt)) * QUARTER_COUNT;
  int quarter;
  if (increase)
    quarter = static_cast<int>(std::floor(scaled)) + 1;
  else
    quarter = static_cast<int>(std::ceil(scaled)) - 1;
  quarter = std::max(0, std::min(QUARTER_COUNT, quarter));
  return static_cast<float>(quarter) / QUARTER_COUNT;
}

void STS3215Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up STS3215 bus with %u servo(s)...", static_cast<unsigned>(servos_.size()));
  if (servos_.empty()) {
    if (current_id_sensor_ != nullptr) current_id_sensor_->publish_state(NAN);
    publish_id_status_("Detecting the connected servo; select only the new ID");
  }
  if (power_pin_ != nullptr) {
    power_pin_->setup();
    power_pin_->digital_write(false);
  }
  for (auto &servo : servos_) {
    servo.preference = global_preferences->make_preference<STS3215PreferenceData>(servo.preference_key);
    load_preferences_(servo);
    // Keep this separate from the existing position/calibration preference layout.
    servo.direction_preference = global_preferences->make_preference<bool>(servo.preference_key ^ 0xAC321500);
    servo.direction_preference.load(&servo.negative_is_down);
    if (servo.direction_select != nullptr)
      servo.direction_select->publish_state(servo.negative_is_down ? "Negative is down" : "Negative is up");
    if (servo.saved_position_valid) {
      servo.position_raw = servo.saved_position;
      servo.has_position = true;
    }
    publish_settings_(servo);
    publish_calibration_status_(servo);
  }
  if (power_pin_ != nullptr) {
    set_bus_power_(true);  // Verify EEPROM and restore volatile settings after the motors boot.
  } else {
    initialize_powered_bus_();
  }
}

void STS3215Component::set_bus_power_(bool on) {
  if (power_pin_ == nullptr || power_on_ == on) return;
  power_pin_->digital_write(on);
  power_on_ = on;
  power_ready_ = false;
  if (on) {
    power_on_at_ = millis();
    idle_timer_active_ = false;
  } else {
    idle_timer_active_ = false;
    invalidate_telemetry_();
  }
}

void STS3215Component::invalidate_telemetry_() {
  for (auto &servo : servos_) {
    if (servo.position_sensor != nullptr) servo.position_sensor->publish_state(NAN);
    if (servo.position_raw_sensor != nullptr) servo.position_raw_sensor->publish_state(NAN);
    if (servo.speed_sensor != nullptr) servo.speed_sensor->publish_state(NAN);
    if (servo.load_sensor != nullptr) servo.load_sensor->publish_state(NAN);
    if (servo.voltage_sensor != nullptr) servo.voltage_sensor->publish_state(NAN);
    if (servo.temperature_sensor != nullptr) servo.temperature_sensor->publish_state(NAN);
    if (servo.status_sensor != nullptr) servo.status_sensor->publish_state(NAN);
    if (servo.current_sensor != nullptr) servo.current_sensor->publish_state(NAN);
    if (servo.moving_sensor != nullptr) servo.moving_sensor->invalidate_state();
    if (servo.torque_sensor != nullptr) servo.torque_sensor->invalidate_state();
    if (servo.multi_turn_sensor != nullptr) servo.multi_turn_sensor->invalidate_state();
  }
}

void STS3215Component::initialize_powered_bus_() {
  clear_rx_();
  for (auto &servo : servos_) {
    bool config_read = false;
    servo.mode_ready = update_commission_state_(servo, true, &config_read);
    if (config_read && !servo.mode_ready && !servo.commission_attempted) {
      servo.commission_attempted = true;
      pending_commission_ids_.push_back(servo.id);
    }
    const uint8_t acceleration = servo.acceleration_raw;
    const uint8_t speed[] = {static_cast<uint8_t>(servo.speed_limit_raw),
                             static_cast<uint8_t>(servo.speed_limit_raw >> 8)};
    const uint8_t torque[] = {static_cast<uint8_t>(servo.torque_limit_raw),
                              static_cast<uint8_t>(servo.torque_limit_raw >> 8)};
    const uint8_t disabled = 0;
    // Disable torque first: a servo may restore volatile goal/limit registers
    // when torque is switched off.
    write_register_(servo.id, REG_TORQUE_ENABLE, &disabled, 1);
    write_register_(servo.id, REG_ACCELERATION, &acceleration, 1);
    write_register_(servo.id, REG_GOAL_SPEED, speed, 2);
    write_register_(servo.id, REG_TORQUE_LIMIT, torque, 2);
    if (servo.torque_sensor != nullptr) servo.torque_sensor->publish_state(false);
    uint8_t settings[10];
    bool settings_read = false;
    bool settings_ready = false;
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
      settings_read = read_register_(servo.id, REG_TORQUE_ENABLE, settings, sizeof(settings));
      settings_ready = settings_read && settings[0] == 0 &&
          decode_u16_(&settings[8]) == servo.torque_limit_raw;
      if (settings_ready) break;
      ESP_LOGW(TAG, "Servo %u power-up settings verification failed on attempt %u; restoring torque limit",
               servo.id, attempt + 1);
      if (attempt < 2) {
        delay(10);
        write_register_(servo.id, REG_TORQUE_LIMIT, torque, 2);
      }
    }
    // Goal speed and acceleration are written again in the seven-byte move
    // packet. The tested servo reported zero goal speed at idle after wake,
    // so this idle readback must not prevent that packet from being sent.
    if (!settings_read)
      ESP_LOGW(TAG, "Servo %u settings readback timed out after power-up", servo.id);
    else if (!settings_ready)
      ESP_LOGW(TAG, "Servo %u settings mismatch: torque=%u/%u accel=%u/%u speed=%u/%u limit=%u/%u",
               servo.id, static_cast<unsigned>(settings[0]), static_cast<unsigned>(disabled),
               static_cast<unsigned>(settings[1]), static_cast<unsigned>(acceleration),
               static_cast<unsigned>(decode_u16_(&settings[6])), static_cast<unsigned>(servo.speed_limit_raw),
               static_cast<unsigned>(decode_u16_(&settings[8])), static_cast<unsigned>(servo.torque_limit_raw));
    if (settings_read && settings[1] != acceleration) {
      ESP_LOGW(TAG, "Servo %u accepted acceleration %u instead of %u; using accepted value",
               servo.id, settings[1], acceleration);
      servo.acceleration_raw = settings[1];
      if (servo.acceleration_number != nullptr)
        servo.acceleration_number->publish_state(servo.acceleration_raw);
      save_preferences_(servo);
    }
    servo.mode_ready &= settings_ready;
    uint8_t position[2];
    if (read_register_(servo.id, REG_PRESENT_POSITION, position, sizeof(position)))
      set_hardware_position_(servo, decode_signed_(decode_u16_(position), 15));
  }
  power_ready_ = true;
}

void STS3215Component::loop() {
  if (power_pin_ != nullptr) {
    if (!power_on_ && (!move_queue_.empty() || !calibration_queue_.empty() ||
                       commission_state_ != COMMISSION_IDLE || id_change_state_ != ID_IDLE))
      set_bus_power_(true);
    if (power_on_ && !power_ready_) {
      if (static_cast<uint32_t>(millis() - power_on_at_) < power_on_delay_ms_) return;
      initialize_powered_bus_();
    }
    bool active = false;
    for (const auto &servo : servos_) active |= servo.command_active;
    const bool idle = auto_state_ == AUTO_IDLE && move_queue_.empty() && calibration_queue_.empty() && pending_commission_ids_.empty() &&
                      commission_state_ == COMMISSION_IDLE && id_change_state_ == ID_IDLE && !active &&
                      !(servos_.empty() && current_id_sensor_ != nullptr);
    if (power_on_ && idle) {
      if (!idle_timer_active_) {
        idle_since_ = millis();
        idle_timer_active_ = true;
      } else if (static_cast<uint32_t>(millis() - idle_since_) >= power_off_delay_ms_) {
        set_bus_power_(false);
        return;
      }
    } else {
      idle_timer_active_ = false;
    }
  }
  if (auto_state_ != AUTO_IDLE) {
    process_auto_calibration_();
    return;
  }
  if (id_change_state_ != ID_IDLE) {
    process_id_change_();
    return;
  }
  if (servos_.empty() && current_id_sensor_ != nullptr && !id_detection_paused_ &&
      (id_detection_pending_ || static_cast<int32_t>(millis() - next_id_detection_ms_) >= 0)) {
    detect_current_id_();
    return;
  }
  if (!calibration_queue_.empty()) {
    const auto action = calibration_queue_.front();
    calibration_queue_.pop_front();
    calibration_action(action.first, action.second);
    if (auto_state_ != AUTO_IDLE) return;
  }
  if (commission_state_ == COMMISSION_IDLE && !pending_commission_ids_.empty()) {
    const uint8_t servo_id = pending_commission_ids_.front();
    pending_commission_ids_.pop_front();
    commission_step_mode(servo_id);
  }
  process_commissioning_();
  // A short unloaded move can finish between normal update() calls. Poll
  // active moves frequently so the moving flag and relative encoder progress
  // are observed before the servo resets its Mode 3 counter to zero.
  for (auto &active_servo : servos_) {
    if (active_servo.command_active &&
        static_cast<uint32_t>(millis() - active_servo.last_motion_poll) >= 25)
      poll_servo_(active_servo);
  }
  if (move_queue_.empty()) return;
  if (commission_state_ != COMMISSION_IDLE || !pending_commission_ids_.empty()) return;

  const uint32_t now = millis();
  auto next = move_queue_.begin();
  if (overlapping_) {
    // Skip a busy motor's buffered steps without changing their order. This
    // lets the next blind start during the preceding gravity-return sequence.
    next = std::find_if(move_queue_.begin(), move_queue_.end(), [this](const STS3215QueuedMove &candidate) {
      const auto *candidate_servo = find_servo_(candidate.servo_id);
      return candidate_servo == nullptr || !candidate_servo->command_active;
    });
    if (next == move_queue_.end()) return;
  }
  // In staggered ripple mode, both members of the front pair may run, but
  // their remaining steps must finish before the next pair starts.
  if (!overlapping_ && next->batch != 0) {
    const uint32_t batch = next->batch;
    next = std::find_if(move_queue_.begin(), move_queue_.end(), [this, batch](const STS3215QueuedMove &candidate) {
      const auto *candidate_servo = find_servo_(candidate.servo_id);
      return candidate.batch == batch && (candidate_servo == nullptr || !candidate_servo->command_active);
    });
    if (next == move_queue_.end()) return;
  }
  const auto move = *next;
  if (!overlapping_ && std::any_of(servos_.begin(), servos_.end(), [&move](const STS3215Servo &active) {
        return active.command_active &&
               (move.batch == 0 || active.active_batch != move.batch);
      }))
    return;
  auto *servo = find_servo_(move.servo_id);
  if (servo == nullptr) {
    move_queue_.erase(next);
    return;
  }
  // Never interrupt an in-flight relative command for the same servo. A
  // buffered target starts as soon as that command completes. The configured
  // delay applies only when starting a different motor, which is what limits
  // the multi-blind startup surge.
  if (servo->command_active) return;
  if (power_pin_ != nullptr && !servo->mode_ready) {
    ESP_LOGW(TAG, "Servo %u is unavailable or not commissioned for mode 3; move discarded", servo->id);
    move_queue_.erase(next);
    update_cover_(*servo);
    update_group_cover_();
    return;
  }
  if (std::abs(move.target_raw - servo->position_raw) <= position_tolerance_) {
    move_queue_.erase(next);
    ESP_LOGD(TAG, "Servo %u skipped zero-distance move; %u queued move(s) remain",
             servo->id, static_cast<unsigned>(move_queue_.size()));
    update_cover_(*servo);
    update_group_cover_();
    return;
  }
  const bool same_servo = has_started_move_ && last_move_servo_id_ == move.servo_id;
  if (move.batch != 0) {
    // Pair members and later gravity-return steps share one launch interval.
    if (!move.batch_started && has_started_move_ &&
        static_cast<uint32_t>(now - last_batch_started_) < start_delay_ms_)
      return;
    if (!move.batch_started) {
      last_batch_started_ = now;
      for (auto &queued : move_queue_)
        if (queued.batch == move.batch) queued.batch_started = true;
    }
  } else {
    if (has_started_move_ && !same_servo &&
        static_cast<uint32_t>(now - last_move_started_) < start_delay_ms_)
      return;
    last_batch_started_ = now;
  }
  move_queue_.erase(next);
  begin_move_(*servo, move.target_raw);
  servo->active_batch = move.batch;
}

void STS3215Component::dump_config() {
  ESP_LOGCONFIG(TAG, "STS3215:");
  ESP_LOGCONFIG(TAG, "  UART packet trace: %s", YESNO(uart_trace_));
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Movement order: %s", ripple_ ? "ripple" : "listed");
  ESP_LOGCONFIG(TAG, "  Movement mode: %s", overlapping_ ? "overlapping" : "staggered");
  ESP_LOGCONFIG(TAG, "  Inter-motor start delay: %u ms", static_cast<unsigned>(start_delay_ms_));
  ESP_LOGCONFIG(TAG, "  Move timeout: %u ms", static_cast<unsigned>(move_timeout_ms_));
  if (power_pin_ != nullptr) {
    LOG_PIN("  Motor power pin: ", power_pin_);
    ESP_LOGCONFIG(TAG, "  Motor power-on delay: %u ms", static_cast<unsigned>(power_on_delay_ms_));
    ESP_LOGCONFIG(TAG, "  Motor power-off delay: %u ms", static_cast<unsigned>(power_off_delay_ms_));
  }
  for (const auto &servo : servos_)
    ESP_LOGCONFIG(TAG, "  Servo ID %u%s; calibration %s; gravity return %s", servo.id,
                  servo.inverted ? " (inverted)" : "", calibrated_(servo) ? "complete" : "incomplete",
                  YESNO(servo.gravity_return_to_zero));
}

void STS3215Component::update() {
  // Reannounce the flash-backed state independently of motor power and UART.
  // This also restores the API state if the startup publication was missed.
  for (auto &servo : servos_)
    publish_calibration_status_(servo);
  if (power_pin_ != nullptr && (!power_on_ || !power_ready_)) return;
  for (auto &servo : servos_)
    if (auto_state_ == AUTO_IDLE || servo.id != auto_servo_id_) poll_servo_(servo);
  update_group_cover_();
}

void STS3215Component::add_servo(uint8_t servo_id, bool inverted, uint32_t preference_key,
                                 float initial_speed, uint8_t initial_acceleration, float initial_torque,
                                 bool gravity_return_to_zero, uint8_t max_acceleration) {
  // Assign configuration explicitly so adding runtime fields cannot shift IDs
  // or preference keys through aggregate initialization.
  STS3215Servo servo{};
  servo.id = servo_id;
  servo.inverted = inverted;
  servo.preference_key = preference_key;
  servo.default_speed = initial_speed;
  servo.default_acceleration = initial_acceleration;
  servo.default_torque = initial_torque;
  servo.gravity_return_to_zero = gravity_return_to_zero;
  servo.max_acceleration = max_acceleration;
  servos_.push_back(servo);
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
STS_SETTER(set_calibration_status_sensor, calibration_status_sensor, text_sensor::TextSensor)
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
  uint8_t discarded[64];
  size_t count = 0;
  while (available() && read_byte(&ignored)) {
    if (uart_trace_) {
      discarded[count++] = ignored;
      if (count == sizeof(discarded)) {
        log_uart_bytes_("RX discarded", discarded, count);
        count = 0;
      }
    }
  }
  if (count != 0) log_uart_bytes_("RX discarded", discarded, count);
}

void STS3215Component::log_uart_bytes_(const char *label, const uint8_t *data, size_t length) {
  if (uart_trace_)
    ESP_LOGD(TAG, "UART %s (%u bytes): %s", label, static_cast<unsigned>(length),
             format_hex_pretty(data, length, ' ', false).c_str());
}

bool STS3215Component::read_byte_timeout_(uint8_t *data, uint32_t deadline) {
  while (static_cast<int32_t>(deadline - millis()) > 0) {
    if (available() && read_byte(data))
      return true;
    delay(0);
  }
  return false;
}

bool STS3215Component::read_status_packet_(uint8_t expected_id, uint8_t *data, uint8_t expected_length,
                                         uint8_t *received_id) {
  const uint32_t deadline = millis() + response_timeout_ms_;
  uint8_t received[128];
  size_t received_count = 0;
  auto read_traced = [&](uint8_t *value) {
    if (!read_byte_timeout_(value, deadline)) return false;
    if (uart_trace_) {
      if (received_count == sizeof(received)) {
        log_uart_bytes_("RX stream", received, received_count);
        received_count = 0;
      }
      received[received_count++] = *value;
    }
    return true;
  };
  auto log_received = [&]() {
    if (received_count != 0) log_uart_bytes_("RX stream", received, received_count);
  };
  uint8_t byte = 0, previous = 0;
  while (read_traced(&byte)) {
    if (previous != 0xFF || byte != 0xFF) {
      previous = byte;
      continue;
    }
    previous = 0;
    uint8_t id, packet_length, error;
    if (!read_traced(&id) || !read_traced(&packet_length) ||
        !read_traced(&error)) {
      log_received();
      return false;
    }
    if (packet_length < 2 || packet_length > 64) continue;

    const bool expected = (expected_id == 0xFE ? id <= 253 : id == expected_id) &&
                          packet_length == expected_length + 2;
    uint8_t checksum_sum = id + packet_length + error;
    // LENGTH counts the error byte, parameter bytes, and checksum. Consume
    // complete packets so a late write acknowledgement cannot be mistaken for
    // the read response that follows it.
    for (uint8_t i = 0; i < packet_length - 2; i++) {
      if (!read_traced(&byte)) {
        log_received();
        return false;
      }
      checksum_sum += byte;
      if (expected) data[i] = byte;
    }
    uint8_t received_checksum;
    if (!read_traced(&received_checksum)) {
      log_received();
      return false;
    }
    if (static_cast<uint8_t>(~checksum_sum) != received_checksum) {
      ESP_LOGW(TAG, "Checksum error in response from servo %u", id);
      continue;
    }
    if (!expected) {
      // A valid zero-parameter packet is a write acknowledgement. It can
      // arrive after clear_rx_() when the servo uses status-return level 2.
      if (packet_length != 2)
        ESP_LOGD(TAG, "Skipping status packet (ID %u, length %u)", id, packet_length);
      continue;
    }
    if (error != 0)
      ESP_LOGW(TAG, "Servo %u returned status flags 0x%02X", expected_id, error);
    if (received_id != nullptr) *received_id = id;
    log_received();
    return true;
  }
  log_received();
  return false;
}

bool STS3215Component::read_register_(uint8_t servo_id, uint8_t address, uint8_t *data, uint8_t length) {
  const uint8_t packet_length = 4;
  const uint8_t checksum = static_cast<uint8_t>(~(servo_id + packet_length + INST_READ + address + length));
  const uint8_t packet[] = {0xFF, 0xFF, servo_id, packet_length, INST_READ, address, length, checksum};
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    clear_rx_();
    log_uart_bytes_("TX read", packet, sizeof(packet));
    write_array(packet, sizeof(packet));
    flush();
    if (read_status_packet_(servo_id, data, length)) {
      if (attempt != 0)
        ESP_LOGD(TAG, "Servo %u register %u read succeeded on attempt %u", servo_id, address, attempt + 1);
      return true;
    }
    if (uart_trace_)
      ESP_LOGD(TAG, "UART RX timeout: ID %u register %u length %u attempt %u",
               servo_id, address, length, attempt + 1);
    delay(10);
  }
  return false;
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
  log_uart_bytes_("TX write", packet.data(), packet.size());
  write_array(packet.data(), packet.size());
  flush();
  // A status-return-level-2 servo replies to writes on this half-duplex
  // bus. Wait for that packet before transmitting the next command, rather
  // than colliding with its reply. Level-1 servos may omit the ACK; readback
  // still confirms their settings, so an absent ACK is not a write failure.
  read_status_packet_(servo_id, nullptr, 0);
  return true;
}

void STS3215Component::poll_servo_(STS3215Servo &servo) {
  servo.last_motion_poll = millis();
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

  const int32_t hardware_position = decode_signed_(decode_u16_(&feedback[0]), 15);
  set_hardware_position_(servo, hardware_position);
  const int16_t speed_raw = decode_signed_(decode_u16_(&feedback[2]), 15);
  const int16_t load_raw = decode_signed_(decode_u16_(&feedback[4]), 10);
  const int16_t current_raw = decode_signed_(decode_u16_(&feedback[13]), 15);
  servo.moving = feedback[10] != 0;
  if (servo.command_active) {
    if (servo.moving || hardware_position != 0) {
      // Mode 3 reports the remaining signed distance, not the distance
      // already traveled. It starts at the commanded delta and reaches zero.
      if (servo.moving || std::abs(hardware_position) < std::abs(servo.target_raw - servo.move_start_raw))
        servo.moving_seen = true;
      servo.position_raw = servo.target_raw - hardware_position;
    } else if (servo.moving_seen) {
      servo.position_raw = servo.target_raw;
    }
  }

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
    if (elapsed >= 5000 && !servo.moving_seen && !arrived) {
      ESP_LOGW(TAG, "Servo %u did not begin moving within 5 s; clearing queued moves", servo.id);
      remove_queued_(servo.id);
      finish_move_(servo, false);
    } else if ((elapsed >= 250 && arrived) || stopped_after_motion || elapsed >= move_timeout_ms_)
      finish_move_(servo, elapsed >= move_timeout_ms_ && !arrived);
  }
  update_cover_(servo);
  status_clear_warning();
}

void STS3215Component::begin_move_(STS3215Servo &servo, int32_t target_raw) {
  target_raw = std::max<int32_t>(-32767, std::min<int32_t>(32767, target_raw));
  int32_t move_delta = target_raw - servo.position_raw;
  if (move_delta < -32767 || move_delta > 32767) {
    move_delta = std::max<int32_t>(-32767, std::min<int32_t>(32767, move_delta));
    target_raw = servo.position_raw + move_delta;
    ESP_LOGW(TAG, "Servo %u move was clamped to the Mode 3 single-command step range", servo.id);
  }
  servo.move_start_raw = servo.position_raw;
  servo.target_raw = target_raw;
  const uint8_t enabled = 1;
  write_register_(servo.id, REG_TORQUE_ENABLE, &enabled, 1);
  const uint16_t encoded = encode_signed_(move_delta);
  const uint8_t data[] = {
      servo.acceleration_raw, static_cast<uint8_t>(encoded), static_cast<uint8_t>(encoded >> 8),
      0, 0, static_cast<uint8_t>(servo.speed_limit_raw), static_cast<uint8_t>(servo.speed_limit_raw >> 8)};
  write_register_(servo.id, REG_ACCELERATION, data, sizeof(data));
  servo.command_active = true;
  servo.moving_seen = false;
  servo.command_started = millis();
  servo.last_motion_poll = servo.command_started;
  last_move_started_ = servo.command_started;
  last_move_servo_id_ = servo.id;
  has_started_move_ = true;
  if (servo.torque_sensor != nullptr)
    servo.torque_sensor->publish_state(true);
  ESP_LOGD(TAG, "Servo %u moving by raw delta %ld to logical position %ld", servo.id,
           static_cast<long>(move_delta), static_cast<long>(target_raw));
}

void STS3215Component::finish_move_(STS3215Servo &servo, bool timed_out) {
  const uint8_t disabled = 0;
  write_register_(servo.id, REG_TORQUE_ENABLE, &disabled, 1);
  servo.command_active = false;
  servo.moving_seen = false;
  ESP_LOGD(TAG, "Servo %u move finished at raw %ld (target %ld); %u queued move(s) remain",
           servo.id, static_cast<long>(servo.position_raw), static_cast<long>(servo.target_raw),
           static_cast<unsigned>(move_queue_.size()));
  if (servo.torque_sensor != nullptr)
    servo.torque_sensor->publish_state(false);
  if (timed_out)
    ESP_LOGW(TAG, "Servo %u move timed out; torque disabled", servo.id);
  if (servo.has_position)
    save_preferences_(servo);
  update_cover_(servo);
}

void STS3215Component::enqueue_move_(uint8_t servo_id, int32_t target_raw) {
  target_raw = std::max<int32_t>(-32767, std::min<int32_t>(32767, target_raw));
  const auto existing = std::find_if(move_queue_.begin(), move_queue_.end(),
      [servo_id](const STS3215QueuedMove &move) { return move.servo_id == servo_id; });
  if (existing != move_queue_.end()) {
    existing->target_raw = target_raw;
    existing->batch = enqueue_batch_;
    existing->batch_started = false;
    move_queue_.erase(std::remove_if(std::next(existing), move_queue_.end(),
        [servo_id](const STS3215QueuedMove &move) { return move.servo_id == servo_id; }), move_queue_.end());
  } else {
    move_queue_.push_back({servo_id, target_raw, enqueue_batch_});
  }
}

void STS3215Component::enqueue_cover_sequence_(uint8_t servo_id, int32_t intermediate_raw,
                                               int32_t target_raw) {
  remove_queued_(servo_id);
  move_queue_.push_back({servo_id, intermediate_raw, enqueue_batch_});
  move_queue_.push_back({servo_id, target_raw, enqueue_batch_});
}

bool STS3215Component::pending_target_(const STS3215Servo &servo, int32_t &target_raw) const {
  bool pending = servo.command_active;
  target_raw = pending ? servo.target_raw : servo.position_raw;
  const auto queued = std::find_if(move_queue_.rbegin(), move_queue_.rend(),
      [&servo](const STS3215QueuedMove &move) { return move.servo_id == servo.id; });
  if (queued != move_queue_.rend()) {
    pending = true;
    target_raw = queued->target_raw;
  }
  return pending;
}

void STS3215Component::remove_queued_(uint8_t servo_id) {
  move_queue_.erase(std::remove_if(move_queue_.begin(), move_queue_.end(),
      [servo_id](const STS3215QueuedMove &move) { return move.servo_id == servo_id; }), move_queue_.end());
}

bool STS3215Component::command_position(uint8_t servo_id, float degrees) {
  if (auto_state_ != AUTO_IDLE) return false;
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return false;
  enqueue_move_(servo_id, degrees_to_raw_(degrees, servo->inverted));
  return true;
}

bool STS3215Component::step(uint8_t servo_id, float degrees) {
  if (auto_state_ != AUTO_IDLE) return false;
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return false;
  if (!servo->has_position) {
    if (power_pin_ != nullptr && !power_ready_) {
      // A new Mode 3 coordinate starts at zero; a previously saved coordinate
      // was restored in setup. Defer all bus communication until power is ready.
      servo->position_raw = 0;
      servo->has_position = true;
    } else {
      uint8_t data[2];
      if (!read_register_(servo_id, REG_PRESENT_POSITION, data, sizeof(data))) {
        ESP_LOGE(TAG, "Cannot jog servo %u before its position is known", servo_id);
        return false;
      }
      set_hardware_position_(*servo, decode_signed_(decode_u16_(data), 15));
    }
  }
  const int32_t delta = degrees_to_raw_(degrees, servo->inverted);
  int32_t planned_target;
  pending_target_(*servo, planned_target);
  enqueue_move_(servo_id, planned_target + delta);
  return true;
}

bool STS3215Component::set_speed_limit(uint8_t servo_id, float value) {
  if (auto_state_ != AUTO_IDLE) return false;
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return false;
  servo->speed_limit_display = std::round(std::max(0.0f, std::min(360.0f, value)));
  servo->speed_limit_raw = speed_to_raw_(servo->speed_limit_display);
  const uint8_t data[] = {static_cast<uint8_t>(servo->speed_limit_raw),
                          static_cast<uint8_t>(servo->speed_limit_raw >> 8)};
  if (power_pin_ == nullptr || (power_on_ && power_ready_))
    write_register_(servo_id, REG_GOAL_SPEED, data, 2);
  save_preferences_(*servo);
  return true;
}

bool STS3215Component::set_acceleration(uint8_t servo_id, float value) {
  if (auto_state_ != AUTO_IDLE) return false;
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return false;
  servo->acceleration_raw = static_cast<uint8_t>(
      std::max(0.0f, std::min(static_cast<float>(servo->max_acceleration), std::round(value))));
  if (power_pin_ == nullptr || (power_on_ && power_ready_)) {
    write_register_(servo_id, REG_ACCELERATION, &servo->acceleration_raw, 1);
    uint8_t accepted;
    if (read_register_(servo_id, REG_ACCELERATION, &accepted, 1) && accepted != servo->acceleration_raw) {
      ESP_LOGW(TAG, "Servo %u accepted acceleration %u instead of %u; using accepted value",
               servo_id, accepted, servo->acceleration_raw);
      servo->acceleration_raw = accepted;
    }
  }
  if (servo->acceleration_number != nullptr)
    servo->acceleration_number->publish_state(servo->acceleration_raw);
  save_preferences_(*servo);
  return true;
}

bool STS3215Component::set_torque_limit(uint8_t servo_id, float value) {
  if (auto_state_ != AUTO_IDLE) return false;
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return false;
  servo->torque_limit_display = std::round(std::max(0.0f, std::min(100.0f, value)));
  servo->torque_limit_raw = static_cast<uint16_t>(servo->torque_limit_display * 10.0f);
  const uint8_t data[] = {static_cast<uint8_t>(servo->torque_limit_raw),
                          static_cast<uint8_t>(servo->torque_limit_raw >> 8)};
  if (power_pin_ == nullptr || (power_on_ && power_ready_))
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

void STS3215Component::publish_id_status_(const char *status) {
  ESP_LOGI(TAG, "Servo ID provisioning: %s", status);
  if (id_status_sensor_ != nullptr) id_status_sensor_->publish_state(status);
}

void STS3215Component::publish_detected_id_(uint8_t value) {
  detected_id_ = value;
  detected_id_valid_ = true;
  next_id_detection_ms_ = millis() + 2000;
  if (current_id_sensor_ != nullptr) current_id_sensor_->publish_state(value);
}

void STS3215Component::request_id_detection() {
  if (!servos_.empty() || id_change_state_ != ID_IDLE) return;
  detected_id_valid_ = false;
  if (current_id_sensor_ != nullptr) current_id_sensor_->publish_state(NAN);
  id_detection_pending_ = true;
  id_detection_paused_ = false;
  publish_id_status_("Detecting the connected servo");
}

void STS3215Component::detect_current_id_() {
  // Feetech broadcast PING returns the ID of a single connected motor.
  // Confirm register 5 at that address before publishing or using it.
  const uint8_t ping[] = {0xFF, 0xFF, 0xFE, 2, 1, 0xFE};
  clear_rx_();
  log_uart_bytes_("TX identify", ping, sizeof(ping));
  write_array(ping, sizeof(ping));
  flush();
  uint8_t detected, stored_id;
  if (read_status_packet_(0xFE, nullptr, 0, &detected) &&
      read_register_(detected, REG_ID, &stored_id, 1) && stored_id == detected) {
    const bool changed = !detected_id_valid_ || detected_id_ != detected || id_detection_pending_;
    publish_detected_id_(detected);
    if (changed) publish_id_status_("Connected servo ID detected; ready to provision");
  } else {
    if (detected_id_valid_ || id_detection_pending_)
      publish_id_status_("No servo detected; connect one motor and check power/UART");
    detected_id_valid_ = false;
    if (current_id_sensor_ != nullptr) current_id_sensor_->publish_state(NAN);
  }
  id_detection_pending_ = false;
  next_id_detection_ms_ = millis() + 2000;
}

void STS3215Component::provision_selected_id() {
  if (!detected_id_valid_) {
    publish_id_status_("No verified current ID; identify the connected servo first");
    id_detection_pending_ = true;
    return;
  }
  set_servo_id(detected_id_, selected_new_id_);
}

void STS3215Component::set_servo_id(int32_t current_id, int32_t new_id) {
  if (current_id < 0 || current_id > 253 || new_id < 0 || new_id > 253) {
    publish_id_status_("Rejected: IDs must be integers from 0 to 253");
    return;
  }
  if (!servos_.empty()) {
    publish_id_status_("Rejected: use provisioning firmware without configured servos");
    return;
  }
  if (id_change_state_ != ID_IDLE) {
    ESP_LOGW(TAG, "Servo ID provisioning is already in progress; request ignored");
    return;
  }
  provisioning_current_id_ = static_cast<uint8_t>(current_id);
  provisioning_new_id_ = static_cast<uint8_t>(new_id);
  id_unlock_attempted_ = false;
  id_write_attempted_ = false;
  id_detection_paused_ = false;
  id_change_state_ = ID_WAIT_POWER;
  id_change_next_ms_ = millis();
  publish_id_status_("Provisioning: checking connected servo");
}

void STS3215Component::fail_id_change_(const char *reason) {
  // A write can take effect even when its response is lost. Relock both
  // possible addresses after an uncertain ID write, never broadcast.
  if (id_unlock_attempted_) {
    const uint8_t locked = 1;
    if (id_write_attempted_)
      write_register_(provisioning_new_id_, REG_EEPROM_LOCK, &locked, 1);
    write_register_(provisioning_current_id_, REG_EEPROM_LOCK, &locked, 1);
  }
  ESP_LOGE(TAG, "Servo ID %u -> %u failed: %s", provisioning_current_id_, provisioning_new_id_, reason);
  publish_id_status_(reason);
  detected_id_valid_ = false;
  if (current_id_sensor_ != nullptr) current_id_sensor_->publish_state(NAN);
  id_detection_pending_ = false;
  id_detection_paused_ = true;
  id_change_state_ = ID_IDLE;
}

void STS3215Component::process_id_change_() {
  if (id_change_state_ == ID_IDLE || static_cast<int32_t>(millis() - id_change_next_ms_) < 0) return;
  const uint8_t source = provisioning_current_id_;
  const uint8_t target = provisioning_new_id_;
  uint8_t value;
  switch (id_change_state_) {
    case ID_WAIT_POWER:
      if (power_pin_ != nullptr && !power_ready_) return;
      id_change_state_ = ID_CHECK;
      break;
    case ID_CHECK:
      if (!read_register_(source, REG_ID, &value, 1) || value != source) {
        fail_id_change_("Failed: current ID did not respond correctly; no EEPROM write");
        return;
      }
      if (source == target) {
        publish_detected_id_(target);
        publish_id_status_("Verified: current and new IDs match; EEPROM unchanged");
        id_change_state_ = ID_IDLE;
        return;
      }
      if (read_register_(target, REG_ID, &value, 1)) {
        fail_id_change_("Rejected: new ID is already in use; no EEPROM write");
        return;
      }
      if (!read_register_(source, REG_MOVING, &value, 1) || value != 0) {
        fail_id_change_("Rejected: servo must be stationary and readable; no EEPROM write");
        return;
      }
      value = 0;
      write_register_(source, REG_TORQUE_ENABLE, &value, 1);
      id_change_state_ = ID_TORQUE_OFF;
      break;
    case ID_TORQUE_OFF:
      if (!read_register_(source, REG_TORQUE_ENABLE, &value, 1) || value != 0) {
        fail_id_change_("Failed: torque-off verification failed; no EEPROM write");
        return;
      }
      value = 0;
      id_unlock_attempted_ = true;
      write_register_(source, REG_EEPROM_LOCK, &value, 1);
      id_change_state_ = ID_UNLOCK;
      break;
    case ID_UNLOCK:
      if (!read_register_(source, REG_EEPROM_LOCK, &value, 1) || value != 0) {
        fail_id_change_("Failed: EEPROM unlock could not be verified; relock attempted");
        return;
      }
      id_write_attempted_ = true;
      write_register_(source, REG_ID, &target, 1);
      id_change_state_ = ID_WRITE;
      break;
    case ID_WRITE:
      if (!read_register_(target, REG_ID, &value, 1) || value != target) {
        fail_id_change_("Failed: ID write unverified; relock attempted at both IDs; check current/new ID");
        return;
      }
      value = 1;
      write_register_(target, REG_EEPROM_LOCK, &value, 1);
      id_change_state_ = ID_RELOCK;
      break;
    case ID_RELOCK:
      if (!read_register_(target, REG_EEPROM_LOCK, &value, 1) || value != 1) {
        fail_id_change_("Failed: new ID responds but EEPROM lock unverified; relock attempted");
        return;
      }
      id_change_state_ = ID_VERIFY;
      break;
    case ID_VERIFY:
      if (!read_register_(target, REG_ID, &value, 1) || value != target) {
        fail_id_change_("Failed: final ID verification failed; check current/new ID");
        return;
      }
      ESP_LOGI(TAG, "Servo ID changed from %u to %u; EEPROM relocked, torque remains off", source, target);
      publish_detected_id_(target);
      publish_id_status_("Success: new ID verified and EEPROM locked; torque off");
      id_change_state_ = ID_IDLE;
      return;
    case ID_IDLE:
      return;
  }
  id_change_next_ms_ = millis() + 20;
}

void STS3215Component::commission_step_mode(uint8_t servo_id) {
  if (auto_state_ != AUTO_IDLE) return;
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return;

  if (commission_state_ != COMMISSION_IDLE) {
    ESP_LOGW(TAG, "A multi-turn commissioning operation is already in progress");
    return;
  }
  ESP_LOGW(TAG, "Servo %u multi-turn commissioning requested", servo_id);
  commissioning_servo_id_ = servo_id;
  commission_state_ = COMMISSION_CHECK;
  commission_next_ms_ = millis();
}

void STS3215Component::process_commissioning_() {
  if (commission_state_ == COMMISSION_IDLE ||
      static_cast<int32_t>(millis() - commission_next_ms_) < 0)
    return;

  auto *servo = find_servo_(commissioning_servo_id_);
  if (servo == nullptr) {
    commission_state_ = COMMISSION_IDLE;
    return;
  }

  // A successful configuration read must confirm a mismatch before EEPROM writes.
  switch (commission_state_) {
    case COMMISSION_CHECK: {
      bool config_read;
      if (update_commission_state_(*servo, true, &config_read)) {
        ESP_LOGW(TAG, "Servo %u is already fully commissioned; EEPROM was not written", servo->id);
        commission_state_ = COMMISSION_IDLE;
      } else if (!config_read) {
        ESP_LOGE(TAG, "Servo %u commissioning cancelled because configuration could not be read", servo->id);
        commission_state_ = COMMISSION_IDLE;
      } else {
        commission_state_ = COMMISSION_PREPARE;
        commission_next_ms_ = millis();
      }
      break;
    }

    case COMMISSION_PREPARE: {
      if (!read_register_(servo->id, REG_PHASE, &commissioning_phase_, 1)) {
        ESP_LOGE(TAG, "Cannot commission servo %u: Phase register read failed", servo->id);
        commission_state_ = COMMISSION_IDLE;
        break;
      }
      commissioning_phase_ |= 0x10;
      stop_servo(servo->id);
      const uint8_t torque_off = 0;
      write_register_(servo->id, REG_TORQUE_ENABLE, &torque_off, 1);
      if (servo->torque_sensor != nullptr)
        servo->torque_sensor->publish_state(false);
      commission_state_ = COMMISSION_UNLOCK;
      commission_next_ms_ = millis() + 20;
      break;
    }

    case COMMISSION_UNLOCK: {
      const uint8_t unlocked = 0;
      write_register_(servo->id, REG_EEPROM_LOCK, &unlocked, 1);
      // Some STS3215 firmware continues to report 1 from the lock register
      // after accepting this command. The configuration read-back below is
      // the authoritative success check, matching Feetech's official flow.
      commission_state_ = COMMISSION_WRITE_LIMITS;
      commission_next_ms_ = millis() + 20;
      break;
    }

    case COMMISSION_WRITE_LIMITS: {
      // Mode 3 requires both limits to be zero. A non-zero maximum creates a
      // hard stop at zero and prevents reverse jogging at the lower boundary.
      const uint8_t limits[] = {0, 0, 0, 0};
      write_register_(servo->id, REG_MIN_ANGLE_LIMIT, limits, sizeof(limits));
      commission_state_ = COMMISSION_WRITE_PHASE;
      commission_next_ms_ = millis() + 20;
      break;
    }

    case COMMISSION_WRITE_PHASE:
      write_register_(servo->id, REG_PHASE, &commissioning_phase_, 1);
      commission_state_ = COMMISSION_WRITE_MODE;
      commission_next_ms_ = millis() + 20;
      break;

    case COMMISSION_WRITE_MODE: {
      const uint8_t step_mode = 3;
      write_register_(servo->id, REG_MODE, &step_mode, 1);
      commission_state_ = COMMISSION_LOCK;
      commission_next_ms_ = millis() + 50;
      break;
    }

    case COMMISSION_LOCK: {
      const uint8_t locked = 1;
      write_register_(servo->id, REG_EEPROM_LOCK, &locked, 1);
      commission_state_ = COMMISSION_VERIFY;
      commission_next_ms_ = millis() + 50;
      break;
    }

    case COMMISSION_VERIFY:
      if (update_commission_state_(*servo, true)) {
        ESP_LOGW(TAG, "Servo %u commissioned successfully; power-cycle the complete node", servo->id);
      } else {
        ESP_LOGE(TAG, "Servo %u multi-turn commissioning verification failed", servo->id);
      }
      commission_state_ = COMMISSION_IDLE;
      break;

    case COMMISSION_IDLE:
      break;
  }
}

bool STS3215Component::update_commission_state_(STS3215Servo &servo, bool log_result, bool *read_success) {
  // Read EEPROM registers 9..33 in one transaction. Relevant offsets are:
  // minimum=0, maximum=2, Phase=9, and Operating_Mode=24.
  uint8_t config[25];
  if (read_success != nullptr) *read_success = false;
  if (!read_register_(servo.id, REG_MIN_ANGLE_LIMIT, config, sizeof(config))) {
    servo.mode_ready = false;
    if (servo.multi_turn_sensor != nullptr)
      servo.multi_turn_sensor->publish_state(false);
    if (log_result)
      ESP_LOGE(TAG, "Servo %u multi-turn configuration read failed", servo.id);
    return false;
  }
  if (read_success != nullptr) *read_success = true;
  const uint16_t minimum = decode_u16_(&config[0]);
  const uint16_t maximum = decode_u16_(&config[2]);
  const uint8_t phase = config[9];
  const uint8_t mode = config[24];
  const bool ready = mode == 3 && (phase & 0x10) != 0 && minimum == 0 && maximum == 0;
  servo.mode_ready = ready;
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

void STS3215Component::set_calibration_direction_select(uint8_t id, STS3215CalibrationDirectionSelect *value) {
  auto *servo = find_servo_(id);
  if (servo != nullptr) servo->direction_select = value;
}

bool STS3215Component::set_calibration_direction(uint8_t id, bool negative_is_down) {
  auto *servo = find_servo_(id);
  if (servo == nullptr || auto_state_ != AUTO_IDLE || calibrated_(*servo)) return false;
  servo->negative_is_down = negative_is_down;
  servo->direction_preference.save(&servo->negative_is_down);
  global_preferences->sync();
  return true;
}

void STS3215CalibrationDirectionSelect::control(const std::string &value) {
  if (parent_ != nullptr && parent_->set_calibration_direction(servo_id_, value == "Negative is down"))
    publish_state(value);
  else
    ESP_LOGW(TAG, "Calibration direction is locked; reset calibration first");
}

bool STS3215Component::read_auto_encoder_(STS3215Servo &servo, int32_t &encoder) {
  // Mode 3 clears the remaining-distance feedback when torque is released.
  // Read the physical encoder in position mode, with torque off throughout.
  // Feetech documents that writes with LOCK=1 affect RAM only. Never unlock
  // EEPROM here; restore Mode 3 before any subsequent motor command.
  uint8_t settings;
  if (!read_register_(servo.id, REG_EEPROM_LOCK, &settings, 1) || settings != 1 ||
      !read_register_(servo.id, REG_TORQUE_ENABLE, &settings, 1) || settings != 0)
    return false;
  const uint8_t position_mode = 0;
  write_register_(servo.id, REG_MODE, &position_mode, 1);
  bool success = read_register_(servo.id, REG_MODE, &settings, 1) && settings == position_mode;
  uint8_t feedback[11];
  if (success) {
    delay(5);  // Allow the firmware's feedback cache to reflect the mode change.
    success = read_register_(servo.id, REG_PRESENT_POSITION, feedback, sizeof(feedback)) &&
              feedback[9] == 0 && feedback[10] == 0;
  }
  const uint8_t step_mode = 3;
  write_register_(servo.id, REG_MODE, &step_mode, 1);
  const bool restored = read_register_(servo.id, REG_MODE, &settings, 1) && settings == step_mode;
  servo.mode_ready = restored;
  if (!success || !restored) return false;
  const int32_t raw = decode_signed_(decode_u16_(feedback), 15);
  encoder = ((raw % 4096) + 4096) % 4096;
  return true;
}

void STS3215Component::start_auto_step_(STS3215Servo &servo) {
  const int32_t delta = degrees_to_raw_(10.0f * auto_direction_, servo.inverted);
  auto_target_ = servo.position_raw + delta;
  if (auto_target_ < -32767 || auto_target_ > 32767) {
    ESP_LOGW(TAG, "Servo %u auto calibration reached the coordinate limit", servo.id);
    finish_auto_calibration_(false);
    return;
  }
  const uint8_t enabled = 1;
  write_register_(servo.id, REG_TORQUE_ENABLE, &enabled, 1);
  // Torque-off can restore volatile settings on some servos. Reapply and
  // verify the calibration limit on every step before commanding motion.
  const uint8_t torque[] = {250, 0};
  write_register_(servo.id, REG_TORQUE_LIMIT, torque, sizeof(torque));
  uint8_t accepted[2];
  if (!read_register_(servo.id, REG_TORQUE_LIMIT, accepted, sizeof(accepted)) ||
      decode_u16_(accepted) != 250) {
    finish_auto_calibration_(false);
    return;
  }
  const uint16_t encoded = encode_signed_(delta);
  const uint16_t speed = speed_to_raw_(100.0f);
  const uint8_t data[] = {static_cast<uint8_t>(std::min<uint8_t>(15, servo.max_acceleration)),
      static_cast<uint8_t>(encoded), static_cast<uint8_t>(encoded >> 8), 0, 0,
      static_cast<uint8_t>(speed), static_cast<uint8_t>(speed >> 8)};
  write_register_(servo.id, REG_ACCELERATION, data, sizeof(data));
  auto_state_ = AUTO_MOVE;
  auto_phase_started_ = millis();
  auto_last_poll_ = auto_phase_started_;
  servo.moving = true;
  if (servo.torque_sensor != nullptr) servo.torque_sensor->publish_state(true);
}

void STS3215Component::process_auto_calibration_() {
  auto *servo = find_servo_(auto_servo_id_);
  if (servo == nullptr) { auto_state_ = AUTO_IDLE; return; }
  const uint32_t now = millis();
  if (now - auto_started_ >= move_timeout_ms_) {
    ESP_LOGW(TAG, "Servo %u auto calibration search timed out", servo->id);
    finish_auto_calibration_(false);
    return;
  }
  // No position sample is used until the entire torque-off second has elapsed.
  if (auto_state_ == AUTO_SETTLE && now - auto_phase_started_ < 1000) return;
  if (now - auto_last_poll_ < 25) return;
  auto_last_poll_ = now;
  uint8_t feedback[11];
  if (!read_register_(servo->id, REG_PRESENT_POSITION, feedback, sizeof(feedback)) || feedback[9] != 0) {
    ESP_LOGW(TAG, "Servo %u auto calibration aborted: missing telemetry or servo fault", servo->id);
    finish_auto_calibration_(false);
    return;
  }
  servo->moving = feedback[10] != 0;
  if (auto_state_ == AUTO_MOVE) {
    // A blocked motor may keep MOVING asserted indefinitely. Give the small
    // move time to finish, then release it even if the endpoint prevents arrival.
    const uint32_t elapsed = now - auto_phase_started_;
    if (elapsed < 250 || (servo->moving && elapsed < 1500)) return;
    const uint8_t disabled = 0;
    write_register_(servo->id, REG_TORQUE_ENABLE, &disabled, 1);
    if (servo->torque_sensor != nullptr) servo->torque_sensor->publish_state(false);
    uint8_t torque;
    if (!read_register_(servo->id, REG_TORQUE_ENABLE, &torque, 1) || torque != 0) {
      finish_auto_calibration_(false);
      return;
    }
    auto_state_ = AUTO_SETTLE;
    auto_phase_started_ = millis();
    return;
  }
  int32_t encoder;
  if (!read_auto_encoder_(*servo, encoder)) {
    ESP_LOGW(TAG, "Servo %u cannot read the settled physical encoder", servo->id);
    finish_auto_calibration_(false);
    return;
  }
  // Each commanded step is only ten degrees, so the nearest single-turn
  // encoder difference unambiguously handles wraparound in either direction.
  int32_t traveled = encoder - auto_encoder_;
  if (traveled > 2048) traveled -= 4096;
  if (traveled < -2048) traveled += 4096;
  auto_encoder_ = encoder;
  const int32_t remaining = auto_target_ - (servo->position_raw + traveled);
  servo->position_raw += traveled;
  servo->has_position = true;
  if (servo->position_sensor != nullptr)
    servo->position_sensor->publish_state(raw_to_degrees_(servo->position_raw, servo->inverted));
  if (servo->position_raw_sensor != nullptr) servo->position_raw_sensor->publish_state(servo->position_raw);
  if (servo->moving) { finish_auto_calibration_(false); return; }
  // Five encoder counts (~0.44 degrees) distinguish bounce/short travel from
  // encoder quantization. Never use the requested target as an endpoint.
  const int32_t delta = degrees_to_raw_(10.0f * auto_direction_, servo->inverted);
  const bool endpoint = (delta > 0 ? remaining : -remaining) > 5;
  ESP_LOGI(TAG, "Servo %u auto calibration settled: target=%ld actual=%ld remaining=%ld encoder=%ld endpoint=%s",
           servo->id, static_cast<long>(auto_target_), static_cast<long>(servo->position_raw),
           static_cast<long>(remaining), static_cast<long>(encoder), YESNO(endpoint));
  if (endpoint && auto_direction_ < 0) {
    auto_first_endpoint_ = servo->position_raw;
    auto_direction_ = 1;
    auto_started_ = millis();
    publish_calibration_status_(*servo);
  } else if (endpoint) {
    if (std::abs(servo->position_raw - auto_first_endpoint_) <= 10) {
      finish_auto_calibration_(false);
      return;
    }
    const bool first_is_down = servo->negative_is_down;
    servo->calibration_down = first_is_down ? auto_first_endpoint_ : servo->position_raw;
    servo->calibration_up = first_is_down ? servo->position_raw : auto_first_endpoint_;
    servo->calibration_middle = servo->calibration_down +
        (servo->calibration_up - servo->calibration_down) / 2;
    servo->calibration_mask = 0x07;
    finish_auto_calibration_(true);
    return;
  }
  start_auto_step_(*servo);
}

void STS3215Component::finish_auto_calibration_(bool success) {
  auto *servo = find_servo_(auto_servo_id_);
  if (servo == nullptr) { auto_state_ = AUTO_IDLE; return; }
  const uint8_t disabled = 0;
  write_register_(servo->id, REG_TORQUE_ENABLE, &disabled, 1);
  const uint8_t torque[] = {static_cast<uint8_t>(servo->torque_limit_raw),
                          static_cast<uint8_t>(servo->torque_limit_raw >> 8)};
  const uint8_t speed[] = {static_cast<uint8_t>(servo->speed_limit_raw),
                          static_cast<uint8_t>(servo->speed_limit_raw >> 8)};
  write_register_(servo->id, REG_TORQUE_LIMIT, torque, sizeof(torque));
  write_register_(servo->id, REG_GOAL_SPEED, speed, sizeof(speed));
  write_register_(servo->id, REG_ACCELERATION, &servo->acceleration_raw, 1);
  auto_state_ = AUTO_IDLE;
  servo->moving = false;
  servo->target_raw = servo->position_raw;
  servo->calibration_error = !success;
  servo->calibration_unlocked = !success;
  if (servo->torque_sensor != nullptr) servo->torque_sensor->publish_state(false);
  save_preferences_(*servo);
  publish_calibration_status_(*servo);
  update_cover_(*servo);
  update_group_cover_();
  ESP_LOGI(TAG, "Servo %u auto calibration %s", servo->id, success ? "complete" : "aborted");
}

void STS3215Component::calibration_action(uint8_t servo_id, uint8_t action) {
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return;
  if (auto_state_ != AUTO_IDLE) {
    ESP_LOGW(TAG, "Auto calibration is active; stop it before using calibration controls");
    return;
  }
  if (action == 7 && calibrated_(*servo)) {
    ESP_LOGW(TAG, "Blind %u auto calibration is locked; reset calibration first", servo_id);
    return;
  }
  if (power_pin_ != nullptr && !power_ready_) {
    calibration_queue_.emplace_back(servo_id, action);
    return;
  }
  if (action == 6) {
    stop_servo(servo_id);
    servo->position_raw = 0;
    servo->hardware_position_raw = 0;
    servo->target_raw = 0;
    servo->calibration_down = 0;
    servo->calibration_middle = 0;
    servo->calibration_up = 0;
    servo->calibration_mask = 0;
    servo->calibration_unlocked = true;
    servo->calibration_error = false;
    servo->saved_position = 0;
    servo->saved_position_valid = true;
    save_preferences_(*servo);
    publish_calibration_status_(*servo);
    if (servo->target_position_number != nullptr)
      servo->target_position_number->publish_state(0);
    ESP_LOGW(TAG, "Blind %u calibration reset at logical position zero; calibration controls unlocked", servo_id);
    return;
  }
  if (action == 7) {
    if (commission_state_ != COMMISSION_IDLE || !pending_commission_ids_.empty() ||
        id_change_state_ != ID_IDLE || !move_queue_.empty() ||
        std::any_of(servos_.begin(), servos_.end(), [](const STS3215Servo &s) { return s.command_active; })) {
      ESP_LOGW(TAG, "Auto calibration requires an idle bus");
      return;
    }
    if (!update_commission_state_(*servo, true)) {
      servo->calibration_error = true;
      publish_calibration_status_(*servo);
      return;
    }
    auto_servo_id_ = servo_id;
    auto_direction_ = -1;
    auto_started_ = millis();
    servo->calibration_error = false;
    servo->calibration_unlocked = true;
    servo->calibration_mask = 0;
    save_preferences_(*servo);
    auto_state_ = AUTO_MOVE;
    const uint8_t disabled = 0;
    write_register_(servo_id, REG_TORQUE_ENABLE, &disabled, 1);
    if (!read_auto_encoder_(*servo, auto_encoder_)) {
      finish_auto_calibration_(false);
      return;
    }
    // Only volatile RAM registers are changed, leaving user settings intact.
    const uint8_t torque[] = {250, 0};
    write_register_(servo_id, REG_TORQUE_LIMIT, torque, sizeof(torque));
    uint8_t accepted[10];
    if (!read_register_(servo_id, REG_TORQUE_ENABLE, accepted, sizeof(accepted)) ||
        decode_u16_(&accepted[8]) != 250) {
      finish_auto_calibration_(false);
      return;
    }
    start_auto_step_(*servo);
    publish_calibration_status_(*servo);
    return;
  }
  if (!servo->calibration_unlocked) {
    ESP_LOGW(TAG, "Blind %u calibration controls are locked; press Reset Blind Calibration first", servo_id);
    return;
  }
  if (action == 0 || action == 1) {
    const float amount = action == 0 ? servo->jog_increment : -servo->jog_increment;
    if (!step(servo_id, amount)) {
      servo->calibration_error = true;
      publish_calibration_status_(*servo);
    }
    return;
  }
  // Capture fresh telemetry here instead of using the most recent 500 ms poll.
  // Otherwise a point pressed immediately after a jog can save the old value.
  uint8_t feedback[11];
  if (!read_register_(servo_id, REG_PRESENT_POSITION, feedback, sizeof(feedback))) {
    ESP_LOGW(TAG, "Cannot save calibration for servo %u: position read failed", servo_id);
    servo->calibration_error = true;
    publish_calibration_status_(*servo);
    return;
  }
  servo->hardware_position_raw = decode_signed_(decode_u16_(&feedback[0]), 15);
  servo->moving = feedback[10] != 0;
  if (servo->moving) {
    ESP_LOGW(TAG, "Cannot save calibration for servo %u while it is still moving", servo_id);
    servo->calibration_error = true;
    publish_calibration_status_(*servo);
    return;
  }
  if (servo->command_active) {
    if (millis() - servo->command_started < 250) {
      ESP_LOGW(TAG, "Cannot save calibration for servo %u until the current move completes", servo_id);
      servo->calibration_error = true;
      publish_calibration_status_(*servo);
      return;
    }
    servo->position_raw = servo->target_raw;
    finish_move_(*servo, false);
  }
  if (action == 2) { servo->calibration_down = servo->position_raw; servo->calibration_mask |= 0x01; }
  if (action == 3) { servo->calibration_middle = servo->position_raw; servo->calibration_mask |= 0x02; }
  if (action == 4) { servo->calibration_up = servo->position_raw; servo->calibration_mask |= 0x04; }
  servo->calibration_error = false;
  const char *point_name = action == 2 ? "down" : (action == 3 ? "middle" : "up");
  ESP_LOGI(TAG, "Saved servo %u %s calibration point: raw=%ld (%.1f degrees)", servo_id,
           point_name, static_cast<long>(servo->position_raw),
           raw_to_degrees_(servo->position_raw, servo->inverted));
  if (servo->calibration_mask == 0x07 && !calibrated_(*servo)) {
    const int32_t span = servo->calibration_up - servo->calibration_down;
    const int32_t middle_offset = servo->calibration_middle - servo->calibration_down;
    if (span == 0 || (span > 0 && (middle_offset <= 0 || middle_offset >= span)) ||
        (span < 0 && (middle_offset >= 0 || middle_offset <= span))) {
      servo->calibration_error = true;
      ESP_LOGE(TAG,
               "Servo %u calibration invalid: middle must be strictly between down and up "
               "(down=%ld middle=%ld up=%ld)",
               servo_id, static_cast<long>(servo->calibration_down),
               static_cast<long>(servo->calibration_middle), static_cast<long>(servo->calibration_up));
    }
  } else if (servo->calibration_mask == 0x07) {
    servo->calibration_unlocked = false;
    ESP_LOGI(TAG, "Servo %u blind calibration complete; point buttons locked and cover enabled", servo_id);
  }
  save_preferences_(*servo);
  publish_calibration_status_(*servo);
  update_cover_(*servo);
}

void STS3215Component::command_cover(uint8_t servo_id, float position) {
  if (auto_state_ != AUTO_IDLE) return;
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return;
  if (!calibrated_(*servo)) {
    ESP_LOGW(TAG, "Servo %u cover ignored: down/middle/up calibration is incomplete", servo_id);
    return;
  }
  position = std::max(0.0f, std::min(1.0f, position));
  int32_t previous_target;
  const bool pending = pending_target_(*servo, previous_target);
  if (pending && previous_target == raw_for_cover_position_(*servo, position)) return;
  command_cover_from_(servo_id, position, previous_target);
}

void STS3215Component::command_cover_from_(uint8_t servo_id, float position, int32_t previous_target) {
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return;
  const float previous_tilt = cover_position_for_raw_(*servo, previous_target);
  const int32_t target_raw = raw_for_cover_position_(*servo, position);
  ESP_LOGD(TAG, "Servo %u cover command %.0f%%: current raw %ld, target raw %ld, previous tilt %.0f%%",
           servo_id, position * 100.0f, static_cast<long>(servo->position_raw),
           static_cast<long>(target_raw), previous_tilt * 100.0f);
  if (servo->gravity_return_to_zero && position > 0.001f &&
      position < previous_tilt - 0.001f) {
    enqueue_cover_sequence_(servo_id, raw_for_cover_position_(*servo, 0.0f), target_raw);
    ESP_LOGD(TAG, "Servo %u gravity sequence queued: 0%% then %.0f%% tilt", servo_id, position * 100.0f);
  } else {
    enqueue_move_(servo_id, target_raw);
  }
  if (servo->cover != nullptr) {
    const float current_tilt = cover_position_for_raw_(*servo, servo->position_raw);
    const auto operation = operation_for_target(position);
    servo->cover->update_from_parent(position, tilt_openness(current_tilt), operation);
  }
}

void STS3215Component::command_ripple_(float position, bool stepping, bool increase) {
  // Snapshot every final target before rebuilding the queue. Gravity return
  // and repeated quarter-step commands must use the planned position, not
  // whichever intermediate step happens to be active when the request arrives.
  std::vector<int32_t> previous_targets;
  std::vector<float> targets;
  bool changed = false;
  for (auto &servo : servos_) {
    int32_t previous;
    const bool pending = pending_target_(servo, previous);
    const float target = stepping ? cover_step_target_(servo, previous, increase)
                                  : std::max(0.0f, std::min(1.0f, position));
    previous_targets.push_back(previous);
    targets.push_back(target);
    if (calibrated_(servo)) {
      const int32_t raw = raw_for_cover_position_(servo, target);
      changed |= pending ? raw != previous : std::abs(raw - previous) > position_tolerance_;
    }
  }
  if (!changed) return;  // Repeated group tilt commands preserve both gravity legs.
  for (auto &servo : servos_)
    if (calibrated_(servo)) remove_queued_(servo.id);
  for (size_t outer = 0; outer < (servos_.size() + 1) / 2; outer++) {
    if (++next_batch_ == 0) ++next_batch_;
    enqueue_batch_ = next_batch_;
    const size_t inner = servos_.size() - 1 - outer;
    for (const size_t index : {outer, inner}) {
      auto &servo = servos_[index];
      if (calibrated_(servo)) {
        command_cover_from_(servo.id, targets[index], previous_targets[index]);
      }
      if (outer == inner) break;  // An odd-sized list has a single center blind.
    }
  }
  enqueue_batch_ = 0;
  update_group_cover_();
}

void STS3215Component::command_all_covers(float position) {
  if (auto_state_ != AUTO_IDLE) return;
  if (ripple_) {
    command_ripple_(position, false, false);
    return;
  }
  for (auto &servo : servos_)
    if (calibrated_(servo))
      command_cover(servo.id, position);
  update_group_cover_();
}

void STS3215Component::step_cover(uint8_t servo_id, bool increase) {
  if (auto_state_ != AUTO_IDLE) return;
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return;
  if (!calibrated_(*servo)) {
    ESP_LOGW(TAG, "Servo %u cover step ignored: down/middle/up calibration is incomplete", servo_id);
    return;
  }
  int32_t planned_target;
  pending_target_(*servo, planned_target);
  command_cover(servo_id, cover_step_target_(*servo, planned_target, increase));
}

float STS3215Component::cover_step_target_(const STS3215Servo &servo, int32_t planned_target, bool increase) const {
  float target_tilt = 0.0f;
  bool target_set = false;
  for (int quarter = 0; quarter <= 4; quarter++) {
    const float tilt = static_cast<float>(quarter) / 4.0f;
    if (planned_target == raw_for_cover_position_(servo, tilt)) {
      const int next_quarter = std::max(0, std::min(4, quarter + (increase ? 1 : -1)));
      target_tilt = static_cast<float>(next_quarter) / 4.0f;
      target_set = true;
      break;
    }
  }
  if (!target_set)
    target_tilt = next_cover_quarter(cover_position_for_raw_(servo, planned_target), increase);
  return target_tilt;
}

void STS3215Component::step_all_covers(bool increase) {
  if (auto_state_ != AUTO_IDLE) return;
  if (ripple_) {
    command_ripple_(0.0f, true, increase);
    return;
  }
  for (auto &servo : servos_)
    if (calibrated_(servo))
      step_cover(servo.id, increase);
  update_group_cover_();
}

void STS3215Component::stop_servo(uint8_t servo_id) {
  calibration_queue_.erase(std::remove_if(calibration_queue_.begin(), calibration_queue_.end(),
      [servo_id](const std::pair<uint8_t, uint8_t> &action) {
        return action.first == servo_id && action.second == 7;
      }), calibration_queue_.end());
  if (auto_state_ != AUTO_IDLE && auto_servo_id_ == servo_id) {
    finish_auto_calibration_(false);
    return;
  }
  auto *servo = find_servo_(servo_id);
  if (servo == nullptr) return;
  ESP_LOGW(TAG, "Servo %u stop requested; active=%s, %u queued move(s)", servo_id,
           YESNO(servo->command_active), static_cast<unsigned>(move_queue_.size()));
  remove_queued_(servo_id);
  if (servo->command_active)
    finish_move_(*servo, false);
  update_cover_(*servo);
}

void STS3215Component::stop_all() {
  calibration_queue_.erase(std::remove_if(calibration_queue_.begin(), calibration_queue_.end(),
      [](const std::pair<uint8_t, uint8_t> &action) { return action.second == 7; }), calibration_queue_.end());
  if (auto_state_ != AUTO_IDLE) finish_auto_calibration_(false);
  move_queue_.clear();
  for (auto &servo : servos_)
    if (servo.command_active)
      finish_move_(servo, false);
}

void STS3215Component::load_preferences_(STS3215Servo &servo) {
  STS3215PreferenceData data{};
  const bool loaded = servo.preference.load(&data);
  const bool legacy = loaded && data.version == 2;
  if (loaded && (data.version == PREFERENCE_VERSION || legacy)) {
    servo.speed_limit_display = data.speed_limit;
    servo.torque_limit_display = data.torque_limit;
    servo.jog_increment = data.jog_increment;
    servo.acceleration_raw = data.acceleration;
    if (!legacy) {
      servo.calibration_down = data.down;
      servo.calibration_middle = data.middle;
      servo.calibration_up = data.up;
      servo.calibration_mask = data.calibration_mask;
      servo.calibration_unlocked = data.reserved != 0;
      servo.saved_position = data.last_position;
      servo.saved_position_valid = data.last_position_valid != 0;
    } else {
      ESP_LOGW(TAG, "Servo %u calibration and saved position cleared after Mode 3 feedback correction; recalibrate",
               servo.id);
    }
  } else {
    servo.speed_limit_display = std::round(servo.default_speed);
    servo.torque_limit_display = std::round(servo.default_torque);
    servo.acceleration_raw = servo.default_acceleration;
    servo.jog_increment = 10.0f;
  }
  servo.acceleration_raw = std::min(servo.acceleration_raw, servo.max_acceleration);
  servo.speed_limit_raw = speed_to_raw_(servo.speed_limit_display);
  servo.torque_limit_raw = static_cast<uint16_t>(servo.torque_limit_display * 10.0f);
  if (legacy) save_preferences_(servo);
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
      static_cast<uint8_t>(servo.saved_position_valid), static_cast<uint8_t>(servo.calibration_unlocked)};
  if (!servo.preference.save(&data))
    ESP_LOGW(TAG, "Failed to save preferences for servo %u", servo.id);
  else
    global_preferences->sync();
}

void STS3215Component::set_hardware_position_(STS3215Servo &servo, int32_t hardware_position) {
  servo.hardware_position_raw = hardware_position;
  if (!servo.has_position) {
    // Mode 3 position feedback is relative to the active move and returns to
    // zero when it finishes. The worm drive cannot back-drive while powered
    // off, so the last saved logical position is authoritative after reboot.
    servo.position_raw = servo.saved_position_valid ? servo.saved_position : 0;
    servo.has_position = true;
  }
}

void STS3215Component::publish_settings_(STS3215Servo &servo) {
  if (servo.speed_limit_number != nullptr) servo.speed_limit_number->publish_state(servo.speed_limit_display);
  if (servo.acceleration_number != nullptr) servo.acceleration_number->publish_state(servo.acceleration_raw);
  if (servo.torque_limit_number != nullptr) servo.torque_limit_number->publish_state(servo.torque_limit_display);
  if (servo.jog_increment_number != nullptr) servo.jog_increment_number->publish_state(servo.jog_increment);
}

void STS3215Component::publish_calibration_status_(STS3215Servo &servo) {
  if (servo.calibration_status_sensor == nullptr) return;
  const char *status = "None";
  if (auto_state_ != AUTO_IDLE && auto_servo_id_ == servo.id)
    status = auto_direction_ < 0 ? "Auto calibrating: negative search" : "Auto calibrating: positive search";
  else if (servo.calibration_error || (servo.calibration_mask == 0x07 && !calibrated_(servo)))
    status = "Error";
  else if (servo.calibration_unlocked)
    status = "Active";
  else if (calibrated_(servo))
    status = "Ok";
  if (!servo.calibration_status_sensor->has_state() || servo.calibration_status_sensor->state != status)
    servo.calibration_status_sensor->publish_state(status);
}

int32_t STS3215Component::raw_for_cover_position_(const STS3215Servo &servo, float position) const {
  if (position <= 0.5f)
    return static_cast<int32_t>(std::lround(servo.calibration_down +
        (servo.calibration_middle - servo.calibration_down) * (position * 2.0f)));
  return static_cast<int32_t>(std::lround(servo.calibration_middle +
      (servo.calibration_up - servo.calibration_middle) * ((position - 0.5f) * 2.0f)));
}

float STS3215Component::cover_position_for_raw_(const STS3215Servo &servo, int32_t raw) const {
  // Arrival allows a small encoder error. HA requires exactly zero openness
  // for Closed, so use the same tolerance at both closed endpoints.
  if (std::abs(raw - servo.calibration_down) <= position_tolerance_) return 0.0f;
  if (std::abs(raw - servo.calibration_up) <= position_tolerance_) return 1.0f;
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
  int32_t pending_target;
  const bool pending = pending_target_(servo, pending_target);
  cover::CoverOperation operation = cover::COVER_OPERATION_IDLE;
  const float current_tilt = cover_position_for_raw_(servo, servo.position_raw);
  float reported_tilt = current_tilt;
  if (pending) {
    reported_tilt = cover_position_for_raw_(servo, pending_target);
    operation = operation_for_target(reported_tilt);
  }
  servo.cover->update_from_parent(reported_tilt, tilt_openness(current_tilt), operation);
}

void STS3215Component::update_group_cover_() {
  if (group_cover_ == nullptr) return;
  float tilt_sum = 0.0f;
  float openness_sum = 0.0f;
  size_t count = 0;
  bool opening = false, closing = false;
  for (const auto &servo : servos_) {
    if (!servo.has_position || !calibrated_(servo)) continue;
    const float current_tilt = cover_position_for_raw_(servo, servo.position_raw);
    float reported_tilt = current_tilt;
    int32_t target;
    const bool pending = pending_target_(servo, target);
    if (pending) reported_tilt = cover_position_for_raw_(servo, target);
    tilt_sum += reported_tilt;
    openness_sum += tilt_openness(current_tilt);
    count++;
    if (pending) {
      const float target_tilt = cover_position_for_raw_(servo, target);
      opening |= operation_for_target(target_tilt) == cover::COVER_OPERATION_OPENING;
      closing |= operation_for_target(target_tilt) == cover::COVER_OPERATION_CLOSING;
    }
  }
  if (count == 0) return;
  const auto operation = opening ? cover::COVER_OPERATION_OPENING :
                         closing ? cover::COVER_OPERATION_CLOSING : cover::COVER_OPERATION_IDLE;
  group_cover_->update_from_parent(tilt_sum / count, openness_sum / count, operation);
}

cover::CoverTraits STS3215Cover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_position(false);
  traits.set_supports_tilt(true);
  traits.set_supports_stop(true);
  // Home Assistant models open/close as terminal commands. These blinds use
  // them as repeatable 25% directional steps, so keep both controls enabled.
  traits.set_is_assumed_state(true);
  return traits;
}

void STS3215Cover::control(const cover::CoverCall &call) {
  if (call.get_stop()) parent_->stop_servo(servo_id_);
  if (call.get_position().has_value())
    parent_->step_cover(servo_id_, *call.get_position() > 0.5f);
  if (call.get_tilt().has_value()) parent_->command_cover(servo_id_, *call.get_tilt());
}

void STS3215Cover::update_from_parent(float tilt_value, float openness,
                                      cover::CoverOperation operation) {
  tilt = tilt_value;
  position = openness;
  current_operation = operation;
  publish_state(false);
}

cover::CoverTraits STS3215GroupCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_position(false);
  traits.set_supports_tilt(true);
  traits.set_supports_stop(true);
  traits.set_is_assumed_state(true);
  return traits;
}

void STS3215GroupCover::control(const cover::CoverCall &call) {
  if (call.get_stop()) parent_->stop_all();
  if (call.get_position().has_value())
    parent_->step_all_covers(*call.get_position() > 0.5f);
  if (call.get_tilt().has_value()) parent_->command_all_covers(*call.get_tilt());
}

void STS3215GroupCover::update_from_parent(float tilt_value, float openness,
                                           cover::CoverOperation operation) {
  tilt = tilt_value;
  position = openness;
  current_operation = operation;
  publish_state(false);
}

void STS3215CalibrationButton::press_action() {
  if (parent_ != nullptr) parent_->calibration_action(servo_id_, action_);
}

void STS3215IDNumber::control(float value) {
  if (parent_ == nullptr || !std::isfinite(value) || value < 0 || value > 253 || value != std::floor(value)) return;
  parent_->set_selected_id(static_cast<uint8_t>(value));
  publish_state(value);
}

void STS3215SetIDButton::press_action() {
  if (parent_ == nullptr) return;
  if (discovery_) parent_->request_id_detection();
  else parent_->provision_selected_id();
}

void STS3215PresetButton::press_action() {
  if (parent_ != nullptr) parent_->command_cover(servo_id_, tilt_);
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
  if (parent_ != nullptr) parent_->set_acceleration(servo_id_, clean);
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

#pragma once

#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/gpio.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace sts3215 {

class STS3215Component;

class STS3215PositionNumber : public number::Number {
 public:
  void set_parent(STS3215Component *parent) { parent_ = parent; }
  void set_servo_id(uint8_t value) { servo_id_ = value; }
 protected:
  void control(float value) override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
};

class STS3215SpeedNumber : public number::Number {
 public:
  void set_parent(STS3215Component *parent) { parent_ = parent; }
  void set_servo_id(uint8_t value) { servo_id_ = value; }
 protected:
  void control(float value) override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
};

class STS3215AccelerationNumber : public number::Number {
 public:
  void set_parent(STS3215Component *parent) { parent_ = parent; }
  void set_servo_id(uint8_t value) { servo_id_ = value; }
 protected:
  void control(float value) override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
};

class STS3215TorqueLimitNumber : public number::Number {
 public:
  void set_parent(STS3215Component *parent) { parent_ = parent; }
  void set_servo_id(uint8_t value) { servo_id_ = value; }
 protected:
  void control(float value) override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
};

class STS3215JogIncrementNumber : public number::Number {
 public:
  void set_parent(STS3215Component *parent) { parent_ = parent; }
  void set_servo_id(uint8_t value) { servo_id_ = value; }
 protected:
  void control(float value) override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
};

class STS3215Cover : public cover::Cover {
 public:
  void set_parent(STS3215Component *parent) { parent_ = parent; }
  void set_servo_id(uint8_t value) { servo_id_ = value; }
  cover::CoverTraits get_traits() override;
  void update_from_parent(float tilt, float openness, cover::CoverOperation operation);
 protected:
  void control(const cover::CoverCall &call) override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
};

class STS3215GroupCover : public cover::Cover {
 public:
  void set_parent(STS3215Component *parent) { parent_ = parent; }
  cover::CoverTraits get_traits() override;
  void update_from_parent(float tilt, float openness, cover::CoverOperation operation);
 protected:
  void control(const cover::CoverCall &call) override;
  STS3215Component *parent_{nullptr};
};

class STS3215CalibrationButton : public button::Button {
 public:
  void set_parent(STS3215Component *parent) { parent_ = parent; }
  void set_servo_id(uint8_t value) { servo_id_ = value; }
  void set_action(uint8_t value) { action_ = value; }
 protected:
  void press_action() override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
  uint8_t action_{0};
};

class STS3215PresetButton : public button::Button {
 public:
  void set_parent(STS3215Component *parent) { parent_ = parent; }
  void set_servo_id(uint8_t value) { servo_id_ = value; }
  void set_tilt(float value) { tilt_ = value; }
 protected:
  void press_action() override;
  STS3215Component *parent_{nullptr};
  uint8_t servo_id_{0};
  float tilt_{0.5f};
};

struct STS3215PreferenceData {
  uint32_t version;
  float speed_limit;
  float torque_limit;
  float jog_increment;
  int32_t down;
  int32_t middle;
  int32_t up;
  int32_t last_position;
  uint8_t acceleration;
  uint8_t calibration_mask;
  uint8_t last_position_valid;
  uint8_t reserved;
};

struct STS3215Servo {
  uint8_t id;
  bool inverted;
  uint32_t preference_key;
  float default_speed;
  uint8_t default_acceleration;
  float default_torque;
  bool gravity_return_to_zero;
  bool has_position{false};
  bool mode_ready{false};
  int32_t position_raw{0};
  int32_t hardware_position_raw{0};
  int32_t saved_position{0};
  bool saved_position_valid{false};
  int32_t target_raw{0};
  int32_t move_start_raw{0};
  uint16_t speed_limit_raw{0};
  uint8_t acceleration_raw{0};
  uint16_t torque_limit_raw{0};
  float speed_limit_display{0.0f};
  float torque_limit_display{0.0f};
  float jog_increment{10.0f};
  int32_t calibration_down{0};
  int32_t calibration_middle{0};
  int32_t calibration_up{0};
  uint8_t calibration_mask{0};
  bool calibration_unlocked{false};
  bool calibration_error{false};
  bool moving{false};
  bool moving_seen{false};
  bool command_active{false};
  uint32_t command_started{0};
  uint32_t last_motion_poll{0};
  ESPPreferenceObject preference;

  sensor::Sensor *position_sensor{nullptr};
  sensor::Sensor *position_raw_sensor{nullptr};
  sensor::Sensor *speed_sensor{nullptr};
  sensor::Sensor *load_sensor{nullptr};
  sensor::Sensor *temperature_sensor{nullptr};
  sensor::Sensor *voltage_sensor{nullptr};
  sensor::Sensor *current_sensor{nullptr};
  sensor::Sensor *status_sensor{nullptr};
  binary_sensor::BinarySensor *moving_sensor{nullptr};
  binary_sensor::BinarySensor *torque_sensor{nullptr};
  binary_sensor::BinarySensor *multi_turn_sensor{nullptr};
  STS3215PositionNumber *target_position_number{nullptr};
  STS3215SpeedNumber *speed_limit_number{nullptr};
  STS3215AccelerationNumber *acceleration_number{nullptr};
  STS3215TorqueLimitNumber *torque_limit_number{nullptr};
  STS3215JogIncrementNumber *jog_increment_number{nullptr};
  text_sensor::TextSensor *calibration_status_sensor{nullptr};
  STS3215Cover *cover{nullptr};
};

struct STS3215QueuedMove {
  uint8_t servo_id;
  int32_t target_raw;
};

class STS3215Component : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_start_delay(uint32_t value) { start_delay_ms_ = value; }
  void set_move_timeout(uint32_t value) { move_timeout_ms_ = value; }
  void set_position_tolerance(uint16_t value) { position_tolerance_ = value; }
  void set_power_pin(GPIOPin *pin) { power_pin_ = pin; }
  void set_power_on_delay(uint32_t value) { power_on_delay_ms_ = value; }
  void set_uart_trace(bool value) { uart_trace_ = value; }
  void add_servo(uint8_t servo_id, bool inverted, uint32_t preference_key,
                 float initial_speed, uint8_t initial_acceleration, float initial_torque,
                 bool gravity_return_to_zero);
  void set_position_sensor(uint8_t, sensor::Sensor *);
  void set_position_raw_sensor(uint8_t, sensor::Sensor *);
  void set_speed_sensor(uint8_t, sensor::Sensor *);
  void set_load_sensor(uint8_t, sensor::Sensor *);
  void set_temperature_sensor(uint8_t, sensor::Sensor *);
  void set_voltage_sensor(uint8_t, sensor::Sensor *);
  void set_current_sensor(uint8_t, sensor::Sensor *);
  void set_status_sensor(uint8_t, sensor::Sensor *);
  void set_moving_sensor(uint8_t, binary_sensor::BinarySensor *);
  void set_torque_sensor(uint8_t, binary_sensor::BinarySensor *);
  void set_multi_turn_sensor(uint8_t, binary_sensor::BinarySensor *);
  void set_target_position_number(uint8_t, STS3215PositionNumber *);
  void set_speed_limit_number(uint8_t, STS3215SpeedNumber *);
  void set_acceleration_number(uint8_t, STS3215AccelerationNumber *);
  void set_torque_limit_number(uint8_t, STS3215TorqueLimitNumber *);
  void set_jog_increment_number(uint8_t, STS3215JogIncrementNumber *);
  void set_calibration_status_sensor(uint8_t, text_sensor::TextSensor *);
  void set_cover(uint8_t, STS3215Cover *);
  void set_group_cover(STS3215GroupCover *value) { group_cover_ = value; }

  bool command_position(uint8_t servo_id, float degrees);
  bool step(uint8_t servo_id, float degrees);
  bool set_speed_limit(uint8_t servo_id, float degrees_per_second);
  bool set_acceleration(uint8_t servo_id, float value);
  bool set_torque_limit(uint8_t servo_id, float percent);
  bool set_jog_increment(uint8_t servo_id, float degrees);
  void commission_step_mode(uint8_t servo_id);
  void calibration_action(uint8_t servo_id, uint8_t action);
  void command_cover(uint8_t servo_id, float position);
  void command_all_covers(float position);
  void step_cover(uint8_t servo_id, bool increase);
  void step_all_covers(bool increase);
  void stop_servo(uint8_t servo_id);
  void stop_all();

 protected:
  static constexpr uint8_t INST_READ = 0x02;
  static constexpr uint8_t INST_WRITE = 0x03;
  static constexpr uint8_t REG_MIN_ANGLE_LIMIT = 9;
  static constexpr uint8_t REG_MAX_ANGLE_LIMIT = 11;
  static constexpr uint8_t REG_PHASE = 18;
  static constexpr uint8_t REG_TORQUE_ENABLE = 40;
  static constexpr uint8_t REG_MODE = 33;
  static constexpr uint8_t REG_ACCELERATION = 41;
  static constexpr uint8_t REG_GOAL_SPEED = 46;
  static constexpr uint8_t REG_TORQUE_LIMIT = 48;
  static constexpr uint8_t REG_EEPROM_LOCK = 55;
  static constexpr uint8_t REG_PRESENT_POSITION = 56;
  static constexpr float STEPS_PER_REVOLUTION = 4096.0f;
  static constexpr uint32_t PREFERENCE_VERSION = 2;

  STS3215Servo *find_servo_(uint8_t servo_id);
  bool read_register_(uint8_t, uint8_t, uint8_t *, uint8_t);
  bool write_register_(uint8_t, uint8_t, const uint8_t *, uint8_t);
  bool read_status_packet_(uint8_t, uint8_t *, uint8_t);
  bool read_byte_timeout_(uint8_t *, uint32_t);
  void clear_rx_();
  void log_uart_bytes_(const char *label, const uint8_t *data, size_t length);
  void set_bus_power_(bool on);
  void initialize_powered_bus_();
  void invalidate_telemetry_();
  void poll_servo_(STS3215Servo &servo);
  void begin_move_(STS3215Servo &servo, int32_t target_raw);
  void finish_move_(STS3215Servo &servo, bool timed_out);
  void enqueue_move_(uint8_t servo_id, int32_t target_raw);
  void enqueue_cover_sequence_(uint8_t servo_id, int32_t intermediate_raw, int32_t target_raw);
  void remove_queued_(uint8_t servo_id);
  bool pending_target_(const STS3215Servo &servo, int32_t &target_raw) const;
  void load_preferences_(STS3215Servo &servo);
  void save_preferences_(STS3215Servo &servo);
  void publish_settings_(STS3215Servo &servo);
  void publish_calibration_status_(STS3215Servo &servo);
  void update_cover_(STS3215Servo &servo);
  void update_group_cover_();
  void process_commissioning_();
  bool update_commission_state_(STS3215Servo &servo, bool log_result);
  void set_hardware_position_(STS3215Servo &servo, int32_t hardware_position);
  bool calibrated_(const STS3215Servo &servo) const {
    if (servo.calibration_mask != 0x07) return false;
    const int32_t span = servo.calibration_up - servo.calibration_down;
    const int32_t middle = servo.calibration_middle - servo.calibration_down;
    return span != 0 && ((span > 0 && middle > 0 && middle < span) ||
                         (span < 0 && middle < 0 && middle > span));
  }
  int32_t raw_for_cover_position_(const STS3215Servo &servo, float position) const;
  float cover_position_for_raw_(const STS3215Servo &servo, int32_t raw) const;

  static uint16_t decode_u16_(const uint8_t *data);
  static int16_t decode_signed_(uint16_t value, uint8_t sign_bit);
  static uint16_t encode_signed_(int32_t value);
  static int32_t degrees_to_raw_(float degrees, bool inverted);
  static float raw_to_degrees_(int32_t raw, bool inverted);
  static uint16_t speed_to_raw_(float degrees_per_second);
  static float speed_to_degrees_(int16_t raw, bool inverted);

  std::vector<STS3215Servo> servos_;
  std::deque<STS3215QueuedMove> move_queue_;
  std::deque<std::pair<uint8_t, uint8_t>> calibration_queue_;
  STS3215GroupCover *group_cover_{nullptr};
  uint32_t response_timeout_ms_{50};
  uint32_t start_delay_ms_{2000};
  uint32_t move_timeout_ms_{120000};
  uint16_t position_tolerance_{5};
  uint32_t last_move_started_{0};
  uint8_t last_move_servo_id_{0};
  bool has_started_move_{false};
  GPIOPin *power_pin_{nullptr};
  uint32_t power_on_delay_ms_{1000};
  uint32_t power_on_at_{0};
  bool power_on_{false};
  bool power_ready_{false};
  bool uart_trace_{false};

  enum CommissionState : uint8_t {
    COMMISSION_IDLE,
    COMMISSION_CHECK,
    COMMISSION_PREPARE,
    COMMISSION_UNLOCK,
    COMMISSION_WRITE_LIMITS,
    COMMISSION_WRITE_PHASE,
    COMMISSION_WRITE_MODE,
    COMMISSION_LOCK,
    COMMISSION_VERIFY,
  };
  CommissionState commission_state_{COMMISSION_IDLE};
  uint8_t commissioning_servo_id_{0};
  uint8_t commissioning_phase_{0};
  uint32_t commission_next_ms_{0};
};

template<typename... Ts> class STS3215StepAction : public Action<Ts...>, public Parented<STS3215Component> {
 public:
  TEMPLATABLE_VALUE(uint8_t, servo_id)
  TEMPLATABLE_VALUE(float, degrees)
  void play(Ts... x) override { parent_->step(servo_id_.value(x...), degrees_.value(x...)); }
};

}  // namespace sts3215
}  // namespace esphome

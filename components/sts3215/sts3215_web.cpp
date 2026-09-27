#include "sts3215.h"

#ifdef USE_STS3215_WEB_UI

#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

#include "web_dashboard.h"

namespace esphome {
namespace sts3215 {

namespace {
void append_number(std::string &json, const char *key, float value) {
  json += '"';
  json += key;
  json += "\":";
  json += std::isfinite(value) ? std::to_string(value) : "null";
  json += ',';
}

void append_int(std::string &json, const char *key, int32_t value) {
  json += '"';
  json += key;
  json += "\":";
  json += std::to_string(value);
  json += ',';
}

void append_bool(std::string &json, const char *key, bool value) {
  json += '"';
  json += key;
  json += "\":";
  json += value ? "true" : "false";
  json += ',';
}

float sensor_value(sensor::Sensor *sensor) {
  return sensor != nullptr && sensor->has_state() ? sensor->state : NAN;
}
bool parse_int(const std::string &text, int32_t &value) {
  if (text.empty()) return false;
  const char *start = text.c_str();
  if (*start == '+' || *start == '-') ++start;
  if (*start == '\0') return false;
  for (const char *p = start; *p; ++p) if (*p < '0' || *p > '9') return false;
  errno = 0;
  char *end;
  const long parsed = strtol(text.c_str(), &end, 10);
  if (errno != 0 || *end != '\0' || parsed < std::numeric_limits<int32_t>::min() ||
      parsed > std::numeric_limits<int32_t>::max()) return false;
  value = static_cast<int32_t>(parsed);
  return true;
}
}  // namespace

class STS3215WebHandler : public AsyncWebHandler {
 public:
  explicit STS3215WebHandler(STS3215Component *parent) : parent_(parent) {}

  bool canHandle(AsyncWebServerRequest *request) const override {
    char url_buffer[AsyncWebServerRequest::URL_BUF_SIZE];
    auto url = request->url_to(url_buffer);
    return (request->method() == HTTP_GET && (url == "/" || url == "/sts3215/state")) ||
        (request->method() == HTTP_POST && url == "/sts3215/control");
  }

  void handleRequest(AsyncWebServerRequest *request) override {
    char url_buffer[AsyncWebServerRequest::URL_BUF_SIZE];
    auto url = request->url_to(url_buffer);
    if (url == "/") {
      auto *response = request->beginResponse(
          200, "text/html; charset=utf-8", reinterpret_cast<const uint8_t *>(STS3215_DASHBOARD_HTML),
          sizeof(STS3215_DASHBOARD_HTML) - 1);
      request->send(response);
      return;
    }
    if (url == "/sts3215/control") {
      auto *id_arg = request->getParam("id");
      auto *action_arg = request->getParam("action");
      int32_t id;
      if (id_arg == nullptr || action_arg == nullptr || !parse_int(id_arg->value(), id) || id < 1 || id > 253) {
        request->send(400, "text/plain", "Invalid control request");
        return;
      }
      const std::string action = action_arg->value();
      bool success = false;
      if (action == "manual" || action == "edit") {
        auto *value_arg = request->getParam("value");
        if (value_arg != nullptr && (value_arg->value() == "0" || value_arg->value() == "1")) {
          const bool enabled = value_arg->value() == "1";
          success = action == "manual" ? parent_->set_manual_control(id, enabled) :
              parent_->set_edit_positions(id, enabled);
        }
      } else if (action == "positions") {
        int32_t down, middle, up;
        auto *d = request->getParam("down");
        auto *m = request->getParam("middle");
        auto *u = request->getParam("up");
        if (d != nullptr && m != nullptr && u != nullptr && parse_int(d->value(), down) &&
            parse_int(m->value(), middle) && parse_int(u->value(), up))
          success = parent_->set_manual_positions(id, down, middle, up);
      }
      request->send(success ? 200 : 400, "text/plain", success ? "OK" : "Invalid or unavailable control");
      return;
    }

    std::string json;
    json.reserve(2400);
    json += "{\"servos\":[";
    bool first = true;
    for (const auto &servo : parent_->servos_) {
      if (!first) json += ',';
      first = false;
      json += '{';
      append_int(json, "id", servo.id);
      append_number(json, "position", servo.has_position ? static_cast<float>(servo.position_raw) : NAN);
      int32_t target = servo.position_raw;
      parent_->pending_target_(servo, target);
      append_number(json, "target", servo.has_position ? static_cast<float>(target) : NAN);
      append_int(json, "down", servo.calibration_down);
      append_int(json, "middle", servo.calibration_middle);
      append_int(json, "up", servo.calibration_up);
      append_int(json, "mask", servo.calibration_mask);
      append_bool(json, "calibrated", parent_->calibrated_(servo));
      append_bool(json, "unlocked", servo.calibration_unlocked);
      append_bool(json, "manual_control", servo.manual_control);
      append_bool(json, "edit_positions", servo.edit_positions);
      append_bool(json, "positions_manual", servo.positions_manual);
      append_bool(json, "middle_calculated", servo.middle_calculated);
      append_bool(json, "negative_is_down", servo.negative_is_down);
      append_bool(json, "auto_active", parent_->auto_state_ != STS3215Component::AUTO_IDLE &&
                  parent_->auto_servo_id_ == servo.id);
      append_bool(json, "moving", servo.moving || servo.command_active);
      append_bool(json, "torque_enabled", servo.torque_sensor != nullptr && servo.torque_sensor->state);
      append_number(json, "jog", servo.jog_increment);
      append_number(json, "speed_limit", servo.speed_limit_display);
      append_int(json, "acceleration", servo.acceleration_raw);
      append_number(json, "torque_limit", servo.torque_limit_display);
      append_number(json, "tilt", servo.cover != nullptr ? servo.cover->tilt : NAN);
      append_number(json, "speed", sensor_value(servo.speed_sensor));
      append_number(json, "load", sensor_value(servo.load_sensor));
      append_number(json, "voltage", sensor_value(servo.voltage_sensor));
      append_number(json, "current", sensor_value(servo.current_sensor));
      append_number(json, "temperature", sensor_value(servo.temperature_sensor));
      append_number(json, "servo_status", sensor_value(servo.status_sensor));
      const char *status = "None";
      if (parent_->auto_state_ != STS3215Component::AUTO_IDLE && parent_->auto_servo_id_ == servo.id)
        status = parent_->auto_direction_ < 0 ? "Auto: negative search" : "Auto: positive search";
      else if (servo.calibration_error || (servo.calibration_mask == 0x07 && !parent_->calibrated_(servo)))
        status = "Error";
      else if (servo.calibration_unlocked)
        status = "Active";
      else if (parent_->calibrated_(servo))
        status = "Ok";
      json += "\"status\":\"";
      json += status;
      json += "\"}";
    }
    json += "],";
    append_number(json, "all_tilt", parent_->group_cover_ != nullptr ? parent_->group_cover_->tilt : NAN);
    json.back() = '}';  // Replace the trailing comma from append_number.
    request->send(200, "application/json", json.c_str());
  }

  bool isRequestHandlerTrivial() const override { return false; }

 protected:
  STS3215Component *parent_;
};

void STS3215Component::register_web_ui(web_server_base::WebServerBase *base) {
  if (this->web_handler_ != nullptr) return;
  this->web_handler_ = new STS3215WebHandler(this);  // WebServerBase keeps handlers for firmware lifetime.
  base->add_handler(this->web_handler_);
}

}  // namespace sts3215
}  // namespace esphome

#endif  // USE_STS3215_WEB_UI

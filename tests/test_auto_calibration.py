"""Exercise production auto-calibration against a fake clock and Mode 3 bus."""

from pathlib import Path
import subprocess
import tempfile

from test_id_provisioning import ROOT, function


def main():
    source = (ROOT / "components/sts3215/sts3215.cpp").read_text()
    runtime = "\n".join(function(source, name) for name in (
        "read_auto_encoder_", "start_auto_step_", "process_auto_calibration_", "finish_auto_calibration_"))
    harness = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define YESNO(x) (x)
uint32_t clock_ms=0;
uint32_t millis() { return clock_ms; }
void delay(uint32_t value) { clock_ms+=value; }
struct Sensor { void publish_state(float) {} };
struct STS3215Servo {
  uint8_t id=6, max_acceleration=254, acceleration_raw=20, calibration_mask=0;
  bool inverted=false, moving=false, has_position=true, negative_is_down=true, mode_ready=true;
  bool calibration_error=false, calibration_unlocked=true;
  int32_t position_raw=0, hardware_position_raw=0, target_raw=0;
  int32_t calibration_down=0, calibration_up=0, calibration_middle=0;
  uint16_t torque_limit_raw=300, speed_limit_raw=1024;
  Sensor *torque_sensor=nullptr, *position_sensor=nullptr, *position_raw_sensor=nullptr;
};
struct Write { uint8_t reg; std::vector<uint8_t> data; };
struct STS3215Component {
  enum AutoCalibrationState { AUTO_IDLE, AUTO_MOVE, AUTO_SETTLE };
  AutoCalibrationState auto_state_=AUTO_MOVE;
  uint8_t auto_servo_id_=6;
  int8_t auto_direction_=-1;
  int32_t auto_target_=0, auto_first_endpoint_=0;
  int32_t auto_encoder_=2000, physical_encoder=2000;
  uint32_t auto_started_=0, auto_phase_started_=0, auto_last_poll_=0, move_timeout_ms_=120000;
  static constexpr uint8_t REG_TORQUE_ENABLE=40, REG_TORQUE_LIMIT=48;
  static constexpr uint8_t REG_ACCELERATION=41, REG_GOAL_SPEED=46, REG_PRESENT_POSITION=56;
  static constexpr uint8_t REG_MODE=33, REG_EEPROM_LOCK=55;
  STS3215Servo servo;
  std::vector<Write> writes;
  int32_t residual=0;
  bool physical_moving=false, healthy=true;
  uint8_t fault=0, torque=1;
  uint8_t mode=3, eeprom_lock=1;
  uint16_t torque_limit=300;
  int read_count=0, save_count=0;
  STS3215Servo *find_servo_(uint8_t id) { return id==servo.id ? &servo : nullptr; }
  int32_t degrees_to_raw_(float d, bool inverted) { return std::lround(d*4096/360)*(inverted ? -1 : 1); }
  float raw_to_degrees_(int32_t r, bool inverted) { return r*360.0f/4096*(inverted ? -1 : 1); }
  uint16_t speed_to_raw_(float speed) { return std::lround(speed*4096/360); }
  uint16_t encode_signed_(int32_t r) { return r<0 ? -r|0x8000 : r; }
  uint16_t decode_u16_(uint8_t *p) { return p[0]|p[1]<<8; }
  int32_t decode_signed_(uint16_t r, int) { return r&0x8000 ? -(r&0x7fff) : r; }
  bool write_register_(uint8_t, uint8_t reg, const uint8_t *p, uint8_t size) {
    writes.push_back({reg,{p,p+size}});
    if (reg==40) torque=p[0];
    if (reg==48) torque_limit=p[0]|p[1]<<8;
    if (reg==33) { assert(torque==0 && eeprom_lock==1); mode=p[0]; }
    assert(reg>=40 || reg==33); // Mode writes require locked EEPROM and torque off.
    assert(reg!=55); // Never unlock EEPROM.
    return true;
  }
  bool read_register_(uint8_t, uint8_t reg, uint8_t *p, uint8_t size) {
    read_count++;
    if (!healthy) return false;
    std::fill(p,p+size,0);
    if (reg==40) p[0]=torque;
    if (reg==48) { p[0]=torque_limit; p[1]=torque_limit>>8; }
    if (reg==33) p[0]=mode;
    if (reg==55) p[0]=eeprom_lock;
    if (reg==56) {
      uint16_t r=encode_signed_(mode==0 ? physical_encoder : residual);
      p[0]=r; p[1]=r>>8; p[9]=fault; p[10]=physical_moving;
    }
    return true;
  }
  void save_preferences_(STS3215Servo &) { save_count++; }
  void publish_calibration_status_(STS3215Servo &) {}
  void update_cover_(STS3215Servo &) {}
  void update_group_cover_() {}
  void start_auto_step_(STS3215Servo &);
  bool read_auto_encoder_(STS3215Servo &, int32_t &);
  void process_auto_calibration_();
  void finish_auto_calibration_(bool);
  void advance(uint32_t dt) { clock_ms+=dt; process_auto_calibration_(); }
};
RUNTIME
void step_and_settle(STS3215Component &c, int32_t settled_residual) {
  c.residual=0;
  c.advance(250);
  assert(c.auto_state_==c.AUTO_SETTLE && c.torque==0);
  const auto previous_position=c.servo.position_raw;
  const int reads=c.read_count;
  c.residual=settled_residual;
  c.advance(999);
  assert(c.read_count==reads && c.servo.position_raw==previous_position);
  const int delta=c.degrees_to_raw_(10.0f*c.auto_direction_,c.servo.inverted);
  c.physical_encoder=(c.physical_encoder+delta-settled_residual+4096)%4096;
  c.residual=0; // Real Mode 3 clears feedback after torque is released.
  c.advance(1);
}
int main() {
  for (bool inverted : {false,true}) for (bool negative_down : {false,true}) {
    clock_ms=0;
    STS3215Component c;
    c.servo.inverted=inverted;
    c.servo.negative_is_down=negative_down;
    const int sign=inverted ? -1 : 1;
    c.start_auto_step_(c.servo);
    const auto move=c.writes.back();
    assert(move.reg==41 && move.data[0]==15 && move.data[3]==0 && move.data[4]==0);
    assert((move.data[5]|move.data[6]<<8)==1138); // 100 degrees/s.
    step_and_settle(c,0); // Full negative 10 degrees; continue searching.
    assert(c.servo.position_raw==-114*sign && c.auto_direction_==-1);
    step_and_settle(c,-40*sign); // Bounce/shortfall only visible after release.
    assert(c.auto_direction_==1 && c.auto_first_endpoint_==-188*sign);
    step_and_settle(c,0);
    assert(c.servo.position_raw==-74*sign && c.auto_state_==c.AUTO_MOVE);
    step_and_settle(c,20*sign);
    assert(c.auto_state_==c.AUTO_IDLE && c.torque==0);
    assert(c.mode==3 && c.eeprom_lock==1);
    assert(c.servo.calibration_mask==7 && !c.servo.calibration_unlocked && !c.servo.calibration_error);
    assert(c.servo.position_raw==20*sign);
    assert(c.servo.calibration_down==(negative_down ? -188 : 20)*sign);
    assert(c.servo.calibration_up==(negative_down ? 20 : -188)*sign);
    assert(c.servo.calibration_middle==-84*sign);
    assert(c.save_count==1);
    assert(c.writes[c.writes.size()-3].reg==48);
    assert(c.writes[c.writes.size()-3].data[0]==44 && c.writes[c.writes.size()-3].data[1]==1);
    assert(c.writes.back().reg==41 && c.writes.back().data[0]==20);
  }
  { // Blocked MOVING flag must release torque before evaluating its endpoint.
    clock_ms=0; STS3215Component c; c.start_auto_step_(c.servo);
    c.physical_moving=true; c.advance(1499);
    assert(c.auto_state_==c.AUTO_MOVE);
    c.advance(25); assert(c.auto_state_==c.AUTO_SETTLE && c.torque==0);
    c.physical_moving=false; c.residual=-114; c.advance(1000);
    assert(c.auto_direction_==1 && c.auto_first_endpoint_==0);
  }
  for (int failure=0; failure<4; failure++) {
    clock_ms=0; STS3215Component c; c.start_auto_step_(c.servo);
    if (failure==0) c.healthy=false;
    if (failure==1) c.fault=32;
    if (failure==2) c.move_timeout_ms_=100;
    if (failure==3) { c.finish_auto_calibration_(false); }
    else c.advance(250);
    assert(c.auto_state_==c.AUTO_IDLE && c.torque==0 && c.servo.calibration_error);
    assert(c.servo.calibration_mask==0 && c.servo.calibration_unlocked);
  }
  { // A jam in both directions must not become a valid zero-span calibration.
    clock_ms=0; STS3215Component c; c.start_auto_step_(c.servo);
    step_and_settle(c,-114); step_and_settle(c,114);
    assert(c.auto_state_==c.AUTO_IDLE && c.servo.calibration_error && c.servo.calibration_mask==0);
  }
  { // Coordinate exhaustion must abort instead of saving a false endpoint.
    clock_ms=0; STS3215Component c; c.servo.position_raw=-32700;
    c.start_auto_step_(c.servo);
    assert(c.auto_state_==c.AUTO_IDLE && c.servo.calibration_error);
  }
  { // Physical encoder wrapping cannot turn a full negative step into an endpoint.
    clock_ms=0; STS3215Component c; c.auto_encoder_=c.physical_encoder=50;
    c.start_auto_step_(c.servo); step_and_settle(c,0);
    assert(c.servo.position_raw==-114 && c.auto_direction_==-1 && c.physical_encoder==4032);
  }
  { // Reject unlocked EEPROM before any mode changes or reads are attempted.
    clock_ms=0; STS3215Component c; c.eeprom_lock=0; c.torque=0;
    int32_t encoder=0; assert(!c.read_auto_encoder_(c.servo,encoder) && c.writes.empty());
  }
}
'''.replace("RUNTIME", runtime)
    with tempfile.TemporaryDirectory(prefix="sts3215-auto-calibration-") as directory:
        cpp = Path(directory) / "auto.cpp"
        exe = Path(directory) / "auto.exe"
        cpp.write_text(harness)
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", str(cpp), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    print("Auto calibration regression tests passed")


if __name__ == "__main__":
    main()

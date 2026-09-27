"""Host regression checks for the production ID transaction (requires g++).

Run: python tests/test_id_provisioning.py
The mock UART registers exercise success and uncertain-write recovery without
connecting to or changing any physical servo.
"""

from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    start = re.search(rf"(?:void|bool|float) STS3215Component::{name}\(", source).start()
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    source = (ROOT / "components/sts3215/sts3215.cpp").read_text()
    header = (ROOT / "components/sts3215/sts3215.h").read_text()
    enum = re.search(r"enum IDChangeState.*?};", header, re.S).group()
    runtime = "\n".join(function(source, name) for name in (
        "set_servo_id", "fail_id_change_", "process_id_change_", "publish_detected_id_",
        "detect_current_id_", "request_id_detection", "provision_selected_id"))
    harness = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <vector>
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
uint32_t now = 100;
uint32_t millis() { return now; }
struct Readout {
  float state=NAN;
  void publish_state(float value) { state=value; }
};
struct STS3215Component {
  ENUM
  IDChangeState id_change_state_{ID_IDLE};
  std::vector<int> servos_;
  void *power_pin_{nullptr};
  bool power_ready_{true};
  uint8_t provisioning_current_id_{0}, provisioning_new_id_{0};
  uint32_t id_change_next_ms_{0};
  bool id_unlock_attempted_{false}, id_write_attempted_{false};
  uint8_t detected_id_{0}, selected_new_id_{2};
  bool detected_id_valid_{false}, id_detection_pending_{true}, id_detection_paused_{false};
  uint32_t next_id_detection_ms_{0};
  Readout *current_id_sensor_{nullptr};
  static constexpr uint8_t REG_ID=5, REG_MOVING=66,
      REG_TORQUE_ENABLE=40, REG_EEPROM_LOCK=55;
  std::map<int, std::map<int,int>> hardware;
  std::vector<std::tuple<int,int,int>> writes;
  std::string status;
  bool ignore_torque_off=false, fail_unlock_read=false,
      fail_new_id_read=false, ignore_id_write=false, ignore_relock=false;
  bool ping_fails=false, mismatch_stored_id=false;
  std::vector<uint8_t> ping;
  void add(int id, int moving=0) { hardware[id]={{5,id},{66,moving},{40,1},{55,1}}; }
  void publish_id_status_(const char *value) {
    status=value;
    if(status.find("Success:")==0 && current_id_sensor_!=nullptr) {
      assert(current_id_sensor_->state==provisioning_new_id_);
      assert(detected_id_valid_ && hardware[provisioning_new_id_][55]==1);
    }
  }
  void set_servo_id(int32_t,int32_t);
  void fail_id_change_(const char *);
  void process_id_change_();
  void publish_detected_id_(uint8_t);
  void detect_current_id_();
  void request_id_detection();
  void provision_selected_id();
  void clear_rx_() {}
  void log_uart_bytes_(const char *,const uint8_t *,size_t) {}
  void write_array(const uint8_t *data,size_t size) { ping.assign(data,data+size); }
  void flush() {}
  bool read_status_packet_(uint8_t,uint8_t *,uint8_t,uint8_t *received) {
    if (ping_fails || hardware.empty()) return false;
    *received=hardware.begin()->first; return true;
  }
  bool read_register_(uint8_t id,uint8_t address,uint8_t *value,uint8_t) {
    if (!hardware.count(id)) return false;
    if (fail_unlock_read && id_unlock_attempted_ && address==55) return false;
    if (fail_new_id_read && id_write_attempted_ && address==5 && id==provisioning_new_id_) return false;
    *value=hardware[id][address];
    if(mismatch_stored_id && address==5) ++*value;
    return true;
  }
  bool write_register_(uint8_t id,uint8_t address,const uint8_t *value,uint8_t) {
    writes.emplace_back(id,address,*value);
    if (!hardware.count(id)) return true;
    if (address==40 && ignore_torque_off) return true;
    if (address==55 && *value==1 && ignore_relock) return true;
    if (address==5) {
      if (ignore_id_write || hardware[id][55]!=0) return true;
      auto registers=hardware[id]; hardware.erase(id);
      registers[5]=*value; hardware[*value]=registers;
    } else { hardware[id][address]=*value; }
    return true;
  }
  void run() {
    for(int i=0; i<30 && id_change_state_!=ID_IDLE; ++i) { now+=25; process_id_change_(); }
    assert(id_change_state_==ID_IDLE);
  }
};
RUNTIME
int main() {
  {
    STS3215Component c; Readout selector; c.current_id_sensor_=&selector;
    c.add(1); c.set_servo_id(1,2); c.run();
    assert(c.hardware.count(2) && !c.hardware.count(1));
    assert(c.hardware[2][55]==1 && c.hardware[2][40]==0);
    assert(c.status.find("Success:")==0);
    assert(selector.state==2 && c.detected_id_==2 && c.detected_id_valid_);
    const std::vector<std::tuple<int,int,int>> expected{{1,40,0},{1,55,0},{1,5,2},{2,55,1}};
    assert(c.writes==expected);
    c.writes.clear(); c.set_servo_id(2,2); c.run(); assert(c.writes.empty());
    c.selected_new_id_=3; c.provision_selected_id(); c.run();
    assert(selector.state==3 && c.detected_id_==3 && c.hardware.count(3));
  }
  {
    STS3215Component c; Readout selector; c.current_id_sensor_=&selector;
    c.add(2); c.set_servo_id(2,2); c.run();
    assert(selector.state==2 && c.detected_id_==2 && c.writes.empty());
  }
  for(int target : {-1,254,255,256,100000}) {
    STS3215Component c; c.add(1); c.set_servo_id(1,target);
    assert(c.id_change_state_==c.ID_IDLE && c.writes.empty());
  }
  for(int source : {-1,254,255,256}) {
    STS3215Component c; c.set_servo_id(source,2);
    assert(c.id_change_state_==c.ID_IDLE && c.writes.empty());
  }
  {
    STS3215Component c; c.add(1); c.servos_.push_back(1); c.set_servo_id(1,2);
    assert(c.id_change_state_==c.ID_IDLE && c.writes.empty());
  }
  {
    STS3215Component c; c.add(1); c.add(2); c.set_servo_id(1,2); c.run();
    assert(c.writes.empty() && c.status.find("Rejected:")==0);
  }
  {
    STS3215Component c; c.set_servo_id(1,2); c.run(); assert(c.writes.empty());
  }
  {
    STS3215Component c; c.add(1,1); c.set_servo_id(1,2); c.run(); assert(c.writes.empty());
  }
  {
    STS3215Component c; c.add(1); c.ignore_torque_off=true; c.set_servo_id(1,2); c.run();
    assert(c.writes.size()==1 && c.hardware[1][55]==1);
  }
  {
    STS3215Component c; c.add(1); c.fail_unlock_read=true; c.set_servo_id(1,2); c.run();
    assert(c.hardware[1][55]==1 && !c.hardware.count(2));
    assert(c.writes.size()==3); // torque off, unlock, recovery relock
  }
  for(bool applied : {false,true}) {
    STS3215Component c; Readout selector; c.current_id_sensor_=&selector;
    c.add(1); c.ignore_id_write=!applied; c.fail_new_id_read=true;
    c.set_servo_id(1,2); c.run();
    assert(c.hardware[applied ? 2:1][55]==1);
    assert(c.status.find("Failed:")==0);
    assert(std::isnan(selector.state) && !c.detected_id_valid_);
    assert(c.id_detection_paused_ && !c.id_detection_pending_);
    assert(c.writes.size()==5); // recovery tries both possible IDs
    const auto count=c.writes.size(); c.process_id_change_(); assert(c.writes.size()==count);
  }
  {
    STS3215Component c; c.add(1); c.ignore_relock=true; c.set_servo_id(1,2); c.run();
    assert(c.status.find("Failed:")==0); // never reports success without lock verification
  }
  {
    STS3215Component c; c.add(1); c.power_pin_=&c; c.power_ready_=false;
    c.set_servo_id(1,2); c.set_servo_id(1,3);
    now+=25; c.process_id_change_(); assert(c.writes.empty());
    assert(c.provisioning_new_id_==2);
    c.power_ready_=true; c.run(); assert(c.hardware.count(2));
  }
  {
    STS3215Component c; c.add(0); now=UINT32_MAX-10;
    c.set_servo_id(0,253); c.run(); assert(c.hardware.count(253));
  }
  {
    STS3215Component c; Readout readout; c.current_id_sensor_=&readout;
    c.provision_selected_id(); assert(c.writes.empty() && c.id_change_state_==c.ID_IDLE);
    c.add(17); c.detect_current_id_();
    assert(readout.state==17 && c.detected_id_valid_ && c.writes.empty());
    const std::vector<uint8_t> expected{0xFF,0xFF,0xFE,2,1,0xFE};
    assert(c.ping==expected);
    c.hardware.clear(); c.detect_current_id_();
    assert(std::isnan(readout.state) && !c.detected_id_valid_);
    c.add(42); c.request_id_detection(); c.detect_current_id_();
    assert(readout.state==42 && c.detected_id_valid_ && c.writes.empty());
    c.mismatch_stored_id=true; c.detect_current_id_();
    assert(std::isnan(readout.state) && !c.detected_id_valid_);
  }
  {
    STS3215Component c; Readout readout; c.current_id_sensor_=&readout;
    c.add(1); c.detect_current_id_(); c.provision_selected_id();
    while(c.id_change_state_!=c.ID_VERIFY) {
      now+=25; c.process_id_change_();
      assert(readout.state==1); // readout stays old throughout write and relock
      assert(c.status.find("Success:")!=0);
    }
    now+=25; c.process_id_change_();
    assert(readout.state==2 && c.status.find("Success:")==0);
  }
  std::cout << "ID provisioning transaction regression checks passed\n";
}
'''.replace("ENUM", enum).replace("RUNTIME", runtime)
    with tempfile.TemporaryDirectory(prefix="sts3215-id-test-") as directory:
        cpp = Path(directory) / "provision.cpp"
        exe = Path(directory) / "provision.exe"
        cpp.write_text(harness)
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", str(cpp), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()

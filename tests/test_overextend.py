"""Exercise production endpoint tug and settling with a fake bus and clock."""
import subprocess
import tempfile
from pathlib import Path
from test_id_provisioning import ROOT, function

def main():
    source = (ROOT / "components/sts3215/sts3215.cpp").read_text()
    runtime = "\n".join(function(source, name) for name in ("finish_move_", "process_overextend_"))
    harness = r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cmath>
#include <vector>
#define ESP_LOGD(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
uint32_t clock_ms=0;
uint32_t millis() { return clock_ms; }
struct Sensor { void publish_state(bool) {} };
struct STS3215Servo {
 uint8_t id=6, overextend_state=0;
 bool overextend_failed=false, overextend=true, command_active=true, moving_seen=true, moving=false, has_position=true, mode_ready=true;
 int32_t position_raw=1000, target_raw=1000, calibration_down=0, calibration_middle=500, calibration_up=1000;
 int32_t overextend_endpoint=0, overextend_encoder=0, overextend_start=0;
 uint32_t overextend_released=0;
 uint16_t torque_limit_raw=400;
 Sensor *torque_sensor=nullptr;
};
struct STS3215QueuedMove { uint8_t servo_id; };
struct STS3215Component {
 std::vector<STS3215QueuedMove> move_queue_;
 int position_tolerance_=5, encoder=4000, limit=400, begins=0, saves=0;
 bool encoder_ok=true;
 static constexpr int REG_TORQUE_ENABLE=40, REG_TORQUE_LIMIT=48;
 void write_register_(uint8_t,int reg,const uint8_t *data,int size) { if(reg==48 && size==2) limit=data[0]+256*data[1]; }
 bool read_register_(uint8_t,int,uint8_t *data,int) { data[0]=limit;data[1]=limit>>8;return true; }
 int decode_u16_(uint8_t *data) { return data[0]+256*data[1]; }
 bool calibrated_(STS3215Servo &) { return true; }
 bool read_auto_encoder_(STS3215Servo &,int32_t &value) {value=encoder;return encoder_ok;}
 void begin_move_(STS3215Servo &s,int32_t target) { ++begins;s.target_raw=target;s.command_active=true; }
 void remove_queued_(uint8_t) {move_queue_.clear();}
 void update_cover_(STS3215Servo &) {}
 void save_preferences_(STS3215Servo &) {++saves;}
 void finish_move_(STS3215Servo &,bool);
 void process_overextend_(STS3215Servo &);
};
RUNTIME
int main() {
 STS3215Component c; STS3215Servo s;
 c.finish_move_(s,false);
 assert(c.begins==1 && s.target_raw==1228 && c.limit==250 && s.overextend_state==1);
 s.position_raw=1228; c.finish_move_(s,false);
 assert(s.command_active && s.overextend_state==2);
 clock_ms=999;c.process_overextend_(s); assert(s.command_active && c.saves==0);
 clock_ms=1000;c.encoder=104;c.process_overextend_(s);
 assert(!s.command_active && s.position_raw==1200 && c.limit==400 && c.saves==1);
 s=STS3215Servo{};s.position_raw=s.target_raw=0;c.encoder=100;
 c.finish_move_(s,false);assert(s.target_raw==-228);
 s.position_raw=-228;c.finish_move_(s,false);clock_ms=2000;c.encoder=4090;c.process_overextend_(s);
 assert(s.position_raw==-106);
 s=STS3215Servo{};s.position_raw=992;c.finish_move_(s,false);assert(s.overextend_start==992 && s.target_raw==1220);
 s=STS3215Servo{};c.move_queue_.push_back({6});int starts=c.begins;c.finish_move_(s,false);
 assert(c.begins==starts && !s.command_active);
 c.move_queue_.clear();s=STS3215Servo{};c.finish_move_(s,true);assert(c.begins==starts);
 s=STS3215Servo{};c.finish_move_(s,false);c.finish_move_(s,true);clock_ms=3000;c.encoder_ok=false;c.process_overextend_(s);
 assert(!s.mode_ready && !s.command_active && c.limit==400);
}
'''.replace("RUNTIME",runtime)
    with tempfile.TemporaryDirectory() as directory:
        cpp=Path(directory)/"test.cpp";exe=Path(directory)/"test.exe"
        cpp.write_text(harness)
        subprocess.run(["g++","-std=c++17","-Wall","-Wextra","-Werror",str(cpp),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
    print("Overextend regression tests passed")
if __name__ == "__main__": main()

"""Exercise production queue scheduling with a fake clock and active motors."""

from pathlib import Path
import subprocess
import tempfile

from test_id_provisioning import ROOT, function


def main():
    loop = function((ROOT / "components/sts3215/sts3215.cpp").read_text(), "loop")
    scheduler = loop[loop.index("  if (move_queue_.empty()) return;"):]
    harness = r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <deque>
#include <vector>
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGD(...) ((void)0)
uint32_t now_ms = 0;
uint32_t millis() { return now_ms; }
struct STS3215Servo {
  uint8_t id;
  bool command_active=false, mode_ready=true;
  int32_t position_raw=0;
};
struct STS3215QueuedMove { uint8_t servo_id; int32_t target_raw; };
struct STS3215Component {
  std::vector<STS3215Servo> servos_{{1},{2},{3}};
  std::deque<STS3215QueuedMove> move_queue_;
  std::deque<uint8_t> pending_commission_ids_;
  int commission_state_=0;
  static constexpr int COMMISSION_IDLE=0;
  void *power_pin_=nullptr;
  bool overlapping_=false, has_started_move_=false;
  uint8_t last_move_servo_id_=0;
  uint32_t last_move_started_=0, start_delay_ms_=2000;
  std::vector<uint8_t> starts;
  STS3215Servo *find_servo_(uint8_t id) {
    for (auto &servo: servos_) if (servo.id==id) return &servo;
    return nullptr;
  }
  void update_cover_(STS3215Servo &) {}
  void update_group_cover_() {}
  void begin_move_(STS3215Servo &servo, int32_t) {
    servo.command_active=true;
    has_started_move_=true;
    last_move_started_=millis();
    last_move_servo_id_=servo.id;
    starts.push_back(servo.id);
  }
  void schedule();
};
void STS3215Component::schedule() {
SCHEDULER
int main() {
  // A gravity sequence must not block starts of the other motors in overlap mode.
  STS3215Component c;
  c.overlapping_=true;
  c.move_queue_={{1,100},{1,200},{2,100},{2,200},{3,100}};
  c.schedule(); assert(c.starts==std::vector<uint8_t>({1}));
  now_ms=1999; c.schedule(); assert(c.starts.size()==1);
  now_ms=2000; c.schedule(); assert(c.starts==std::vector<uint8_t>({1,2}));
  now_ms=3999; c.schedule(); assert(c.starts.size()==2);
  now_ms=4000; c.schedule(); assert(c.starts==std::vector<uint8_t>({1,2,3}));
  c.schedule(); assert(c.starts.size()==3); // All remaining steps are busy.
  c.servos_[0].command_active=false;
  now_ms=6000; c.schedule(); assert(c.starts.back()==1);
  assert(c.move_queue_.size()==1 && c.move_queue_.front().servo_id==2);
  // The original FIFO behavior remains the default.
  STS3215Component fifo; now_ms=0;
  fifo.move_queue_={{1,100},{1,200},{2,100}};
  fifo.schedule(); now_ms=5000; fifo.schedule(); assert(fifo.starts.size()==1);
  fifo.servos_[0].command_active=false; fifo.schedule();
  assert(fifo.starts==std::vector<uint8_t>({1,1}));
  // No-op and missing servos do not consume start intervals.
  STS3215Component skip; skip.overlapping_=true; now_ms=0;
  skip.move_queue_={{99,100},{1,0},{2,100}};
  skip.schedule(); skip.schedule(); skip.schedule();
  assert(skip.starts==std::vector<uint8_t>({2}));
  // Unsigned subtraction preserves the start delay across millis() rollover.
  STS3215Component wrap; wrap.overlapping_=true;
  wrap.move_queue_={{1,100},{1,200},{2,100}};
  now_ms=UINT32_MAX-999; wrap.schedule();
  now_ms=999; wrap.schedule(); assert(wrap.starts.size()==1);
  now_ms=1000; wrap.schedule(); assert(wrap.starts.size()==2);
}
'''.replace("SCHEDULER", scheduler)
    with tempfile.TemporaryDirectory(prefix="sts3215-queue-test-") as directory:
        cpp = Path(directory) / "queue.cpp"
        exe = Path(directory) / "queue.exe"
        cpp.write_text(harness)
        subprocess.run(["g++", "-std=c++17", str(cpp), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    print("Movement queue tests passed")


if __name__ == "__main__":
    main()

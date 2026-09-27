"""Exercise production queue scheduling with a fake clock and active motors."""

from pathlib import Path
import subprocess
import tempfile

from test_id_provisioning import ROOT, function


def main():
    loop = function((ROOT / "components/sts3215/sts3215.cpp").read_text(), "loop")
    ripple = function((ROOT / "components/sts3215/sts3215.cpp").read_text(), "command_ripple_")
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
  uint32_t active_batch=0;
  bool calibrated=true;
};
struct STS3215QueuedMove { uint8_t servo_id; int32_t target_raw; uint32_t batch=0; bool batch_started=false; };
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
  uint32_t next_batch_=0, enqueue_batch_=0, last_batch_started_=0;
  std::vector<uint8_t> starts;
  bool calibrated_(const STS3215Servo &servo) { return servo.calibrated; }
  void command_cover(uint8_t id, float) { move_queue_.push_back({id,100,enqueue_batch_}); }
  void step_cover(uint8_t id, bool) { move_queue_.push_back({id,200,enqueue_batch_}); }
  void command_ripple_(float, bool, bool);
  void remove_queued_(uint8_t id) {
    move_queue_.erase(std::remove_if(move_queue_.begin(), move_queue_.end(),
      [id](const STS3215QueuedMove &move) { return move.servo_id==id; }), move_queue_.end());
  }
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
RIPPLE
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
  // Ripple uses physical list order, pairing outer IDs even for odd counts.
  STS3215Component ripple; ripple.servos_={{6},{5},{4}};
  ripple.command_ripple_(0.5f,false,false);
  assert(ripple.move_queue_.size()==3);
  assert(ripple.move_queue_[0].servo_id==6 && ripple.move_queue_[1].servo_id==4);
  assert(ripple.move_queue_[0].batch==ripple.move_queue_[1].batch);
  assert(ripple.move_queue_[2].servo_id==5 && ripple.move_queue_[2].batch!=ripple.move_queue_[0].batch);
  now_ms=0; ripple.schedule(); ripple.schedule();
  assert(ripple.starts==std::vector<uint8_t>({6,4}));
  now_ms=10000; ripple.schedule(); assert(ripple.starts.size()==2);
  ripple.servos_[0].command_active=false; ripple.schedule(); assert(ripple.starts.size()==2);
  ripple.servos_[2].command_active=false; ripple.schedule();
  assert(ripple.starts==std::vector<uint8_t>({6,4,5}));
  // Replacing a partially consumed command restores the outside-in order.
  STS3215Component replace; replace.servos_={{6},{5},{4}};
  replace.move_queue_={{5,100,8},{4,100,9}};
  replace.command_ripple_(0.5f,false,false);
  assert(replace.move_queue_.size()==3 && replace.move_queue_[0].servo_id==6);
  assert(replace.move_queue_[1].servo_id==4 && replace.move_queue_[2].servo_id==5);
  // Even lists pair inward; group open/close steps use the same order.
  STS3215Component even; even.servos_={{6},{5},{4},{3},{2},{1}};
  even.command_ripple_(0.0f,true,true);
  for (size_t i=0; i<6; i++) assert(even.move_queue_[i].servo_id==std::vector<uint8_t>({6,1,5,2,4,3})[i]);
  // Overlapping ripple delays pair launches, not the two members of a pair.
  STS3215Component overlap; overlap.overlapping_=true;
  overlap.move_queue_={{1,100,1},{1,200,1},{3,100,1},{3,200,1},{2,100,2}};
  now_ms=0; overlap.schedule(); overlap.schedule();
  assert(overlap.starts==std::vector<uint8_t>({1,3}));
  now_ms=1999; overlap.schedule(); assert(overlap.starts.size()==2);
  now_ms=2000; overlap.schedule(); assert(overlap.starts==std::vector<uint8_t>({1,3,2}));
  // Staggered ripple waits through both members' gravity-return sequences.
  STS3215Component gravity;
  gravity.move_queue_={{1,100,1},{1,200,1},{3,100,1},{3,200,1},{2,100,2}};
  now_ms=0; gravity.schedule(); gravity.schedule();
  gravity.servos_[0].command_active=false; now_ms=500;
  gravity.schedule(); assert(gravity.starts==std::vector<uint8_t>({1,3,1}));
  gravity.servos_[0].command_active=false; now_ms=3000;
  gravity.schedule(); assert(gravity.starts.size()==3);
  gravity.servos_[2].command_active=false; gravity.schedule();
  assert(gravity.starts==std::vector<uint8_t>({1,3,1,3}));
  gravity.servos_[2].command_active=false; gravity.schedule();
  assert(gravity.starts.back()==2);
  // Uncalibrated members are skipped without changing physical pairing.
  STS3215Component partial; partial.servos_={{6},{5},{4}};
  partial.servos_[0].calibrated=false; partial.command_ripple_(0.5f,false,false);
  assert(partial.move_queue_.size()==2 && partial.move_queue_[0].servo_id==4);
  assert(partial.move_queue_[1].servo_id==5);
  // Unsigned subtraction preserves the start delay across millis() rollover.
  STS3215Component wrap; wrap.overlapping_=true;
  wrap.move_queue_={{1,100},{1,200},{2,100}};
  now_ms=UINT32_MAX-999; wrap.schedule();
  now_ms=999; wrap.schedule(); assert(wrap.starts.size()==1);
  now_ms=1000; wrap.schedule(); assert(wrap.starts.size()==2);
}
'''.replace("SCHEDULER", scheduler).replace("RIPPLE", ripple)
    with tempfile.TemporaryDirectory(prefix="sts3215-queue-test-") as directory:
        cpp = Path(directory) / "queue.cpp"
        exe = Path(directory) / "queue.exe"
        cpp.write_text(harness)
        subprocess.run(["g++", "-std=c++17", str(cpp), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    print("Movement queue tests passed")


if __name__ == "__main__":
    main()

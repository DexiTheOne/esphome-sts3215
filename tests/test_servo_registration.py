"""Compile the actual servo structure and registration code to check config fields."""

from pathlib import Path
import re
import subprocess
import tempfile

from test_id_provisioning import ROOT


def main():
    header = (ROOT / "components/sts3215/sts3215.h").read_text()
    source = (ROOT / "components/sts3215/sts3215.cpp").read_text()
    structure = re.search(r"struct STS3215Servo \{.*?\n\};", header, re.S).group()
    registration = re.search(r"void STS3215Component::add_servo\(.*?\n\}", source, re.S).group()
    harness = r'''
#include <cassert>
#include <cstdint>
#include <vector>
struct ESPPreferenceObject {};
namespace sensor { struct Sensor {}; }
namespace binary_sensor { struct BinarySensor {}; }
namespace text_sensor { struct TextSensor {}; }
struct STS3215PositionNumber {};
struct STS3215SpeedNumber {};
struct STS3215AccelerationNumber {};
struct STS3215TorqueLimitNumber {};
struct STS3215JogIncrementNumber {};
struct STS3215Cover {};
STRUCTURE
struct STS3215Component {
  std::vector<STS3215Servo> servos_;
  void add_servo(uint8_t,bool,uint32_t,float,uint8_t,float,bool,uint8_t);
};
REGISTRATION
int main() {
  STS3215Component c;
  for (uint8_t id : {6,5,4}) {
    c.add_servo(id, true, 0x5A321500U+id, 90.0f, 20, 30.0f, true, 50);
    const auto &s = c.servos_.back();
    assert(s.id==id && s.inverted);
    assert(s.preference_key==0x5A321500U+id);
    assert(s.default_speed==90.0f && s.default_acceleration==20);
    assert(s.default_torque==30.0f && s.gravity_return_to_zero && s.max_acceleration==50);
    assert(s.active_batch==0 && !s.command_active && !s.has_position);
    assert(s.position_sensor==nullptr && s.cover==nullptr);
  }
  assert(c.servos_[0].id==6 && c.servos_[1].id==5 && c.servos_[2].id==4);
}
'''.replace("STRUCTURE", structure).replace("REGISTRATION", registration)
    with tempfile.TemporaryDirectory(prefix="sts3215-registration-test-") as directory:
        cpp = Path(directory) / "registration.cpp"
        exe = Path(directory) / "registration.exe"
        cpp.write_text(harness)
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", str(cpp), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    print("Servo registration tests passed")


if __name__ == "__main__":
    main()

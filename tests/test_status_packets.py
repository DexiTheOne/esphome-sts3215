"""Exercise the real packet reader for discovery IDs and delayed write ACKs."""

from pathlib import Path
import subprocess
import tempfile

from test_id_provisioning import ROOT, function


def main():
    reader = function((ROOT / "components/sts3215/sts3215.cpp").read_text(), "read_status_packet_")
    harness = r'''
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGD(...) ((void)0)
uint32_t millis() { return 100; }
struct STS3215Component {
  uint32_t response_timeout_ms_=50;
  bool uart_trace_=false;
  std::vector<uint8_t> stream;
  size_t cursor=0;
  void log_uart_bytes_(const char *,const uint8_t *,size_t) {}
  bool read_byte_timeout_(uint8_t *value,uint32_t) {
    if(cursor==stream.size()) return false;
    *value=stream[cursor++]; return true;
  }
  bool read_status_packet_(uint8_t,uint8_t *,uint8_t,uint8_t *received_id=nullptr);
  void packet(uint8_t id,const std::vector<uint8_t> &parameters, bool corrupt=false) {
    const uint8_t length=parameters.size()+2;
    uint8_t sum=id+length;
    stream.insert(stream.end(),{0xFF,0xFF,id,length,0});
    for(auto value: parameters) { stream.push_back(value); sum+=value; }
    stream.push_back(static_cast<uint8_t>(~sum)+(corrupt ? 1:0));
  }
};
READER
int main() {
  for(uint8_t id: {0,1,42,253}) {
    STS3215Component c; c.packet(id,{}); uint8_t detected=99;
    assert(c.read_status_packet_(0xFE,nullptr,0,&detected) && detected==id);
  }
  for(uint8_t id: {254,255}) {
    STS3215Component c; c.packet(id,{}); uint8_t detected=99;
    assert(!c.read_status_packet_(0xFE,nullptr,0,&detected) && detected==99);
  }
  {
    STS3215Component c; c.packet(42,{},true); uint8_t detected=99;
    assert(!c.read_status_packet_(0xFE,nullptr,0,&detected) && detected==99);
  }
  {
    STS3215Component c; c.packet(42,{},true); c.packet(7,{}); uint8_t detected=99;
    assert(c.read_status_packet_(0xFE,nullptr,0,&detected) && detected==7);
  }
  {
    STS3215Component c; c.packet(1,{}); c.packet(2,{}); c.packet(2,{2});
    uint8_t value=99;
    assert(c.read_status_packet_(2,&value,1) && value==2);
  }
  {
    STS3215Component c; c.packet(1,{1}); uint8_t value=99;
    assert(!c.read_status_packet_(2,&value,1) && value==99);
  }
}
'''.replace("READER", reader)
    with tempfile.TemporaryDirectory(prefix="sts3215-reader-test-") as directory:
        cpp = Path(directory) / "reader.cpp"
        exe = Path(directory) / "reader.exe"
        cpp.write_text(harness)
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", str(cpp), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    print("Discovery packet and delayed-ACK checks passed")


if __name__ == "__main__":
    main()

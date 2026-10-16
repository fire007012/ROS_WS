#include "can_motor_interface/link_health.h"
#include <cstdio>
#include <cstring>
using namespace can_motor_interface;
void dump(const char* label, const can_frame& frame) {
  std::printf("%s %08x DLC=%u struct=", label, frame.can_id, frame.can_dlc);
  const auto* raw = reinterpret_cast<const unsigned char*>(&frame);
  for (size_t i = 0; i < sizeof(frame); ++i) std::printf("%02x", raw[i]);
  std::puts("");
}
int main() {
  dump("heartbeat", stm32Command(0x100, false, 9, 0xff, 0));
  dump("report-mask", stm32Command(0x100, false, 7, 0xff, 7));
  dump("stop", stm32Command(0x100, false, 3, 0xff, 0));
  for (int wheel : {1, 0, 3, 2}) {
    dump("read-speed", driverRead(wheel + 1, 0x35));
    dump("read-status", driverRead(wheel + 1, 0x3a));
  }
}

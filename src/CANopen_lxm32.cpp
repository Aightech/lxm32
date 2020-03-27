#include "CANopen_lxm32.h"

CANopen::LXM32::LXM32(const char *ifname, uint16_t can_id, bool verbose)
  : Driver(ifname,can_id,verbose) {

}

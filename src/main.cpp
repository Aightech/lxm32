#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <cstdint>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "Lexium32A_canopen.h"

#include "joystick.h"



int
main(int argc, char** argv)
{

  LXM32 lxm32("can0", 4, true);
  cJoystick js;
  printf("%d\n",js.joystickValue(1));

  //lxm32.start();
  // Canopen_socket can("can0",true);

  // uint16_t t = 0x1234;

  // uint16_t index = (uint16_t)strtol(argv[1],NULL,16);
  // uint8_t  subindex = (uint8_t)strtol(argv[2],NULL,16);

  // can.send_SDO(1, FRAME_SIZE, SDO_R, index, subindex);

  

	
  return 0;
}

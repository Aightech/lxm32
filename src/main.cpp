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

  
  cJoystick js;
  printf("%d\n",js.joystickValue(1));
  LXM32 lxm32_4("can0", 4,true);
  lxm32_4.init();
  lxm32_4.start();
  LXM32 lxm32_3("can0", 3,true);
  lxm32_3.init();
  lxm32_3.start();

  //lxm32_4.new_pos(50000);

  int alpha4=0;
  int alpha3=0;
  int inc4=0;
  int inc3=0;
  

  while(1)
    {
      inc4=js.joystickValue(1)-alpha4;
      alpha4+=inc4;
      printf("%d\n",alpha4);
      inc3=js.joystickValue(4)-alpha3;
      alpha3+=inc3;
      lxm32_4.new_pos(inc4);
      lxm32_3.new_pos(inc3);
      usleep(100000);
    }

  //lxm32.start();
  // Canopen_socket can("can0",true);

  // uint16_t t = 0x1234;

  // uint16_t index = (uint16_t)strtol(argv[1],NULL,16);
  // uint8_t  subindex = (uint8_t)strtol(argv[2],NULL,16);

  // can.send_SDO(1, FRAME_SIZE, SDO_R, index, subindex);

  

	
  return 0;
}

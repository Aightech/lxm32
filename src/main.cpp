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


  std::vector<LXM32*> motors;
  int32_t m_motor_pos[6];
  int32_t m_inc[6];
  double m_conv=50000.0f/24.0f*180.0f/M_PI;

  
  int m_motor_lookup[6] = {5,2,0,1,4,3};
  
  for(int i=0; i<6; i++)
    {
      motors.push_back(new LXM32("can0", i+1,false));
      motors.back()->init();
      motors.back()->start();
      m_motor_pos[i]=0;
      m_inc[i]=0;
    }
  
  cJoystick js;

  int c=0;
  int i_n;
  int i=0;
  i_n = m_motor_lookup[i];
  while(1)
    {
      if(js.buttonPressed(0)>0 && c==0)
	{
	  i=(i+1)%6;
	  if(i==0)
	      for(int i=0; i<6; i++)
		m_motor_pos[i]=0;
	  printf("motor %d\n",i);
	  c++;
	  i_n = m_motor_lookup[i];
	}
      if(js.buttonPressed(0)==0)
	c=0;

      int target = js.joystickValue(1)*m_conv*3.14/32000/2;
      m_inc[i_n]=target-m_motor_pos[i_n];
      m_motor_pos[i_n]+=m_inc[i_n];
      motors[i_n]->new_pos(m_inc[i_n]);
      printf("val %d\n",target);
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

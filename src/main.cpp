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

#include "CANopen_lxm32.h"

#include "joystick.h"



int
main(int argc, char** argv)
{


  std::vector<CANopen::LXM32*> motors;
  int32_t m_motor_pos[6];
  int32_t m_inc[6];
  double m_conv=50000.0f/24.0f*180.0f/M_PI;

  int m_motor_lookup[6] = {5,2,0,1,4,3};
  
  for(int i=0; i<6; i++)
    {
      //std::cout << CANopen::LXM32::OPmode::ProfilePosition << std::endl;
      motors.push_back(new CANopen::LXM32("vcan0", i+1,false));
      int a = 1;
      int b = 2;
      motors.back()->pdo_watchdog(CANopen::Driver::PDOFunctionCode::PDO1Receive,&a);

      std::cout << a << " " << b << std::endl;
      //motors.back()->start(MODE_ProfilePosition, PPctrl_RELATIVE| PPctrl_ON_DIRECT);
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
      //motors[i_n]->fa_pos(m_inc[i_n]);
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

#include "canopen_socket.h"


Canopen_socket::Canopen_socket(const char* ifname, bool verbose) : m_verbose(verbose)
{
  //Get the can network interface.
  strcpy(m_ifname, ifname);

  //Open a can socket
  if((m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    perror("Error while opening socket");
    

  
  strcpy(m_ifr.ifr_name, ifname);
  ioctl(m_socket, SIOCGIFINDEX, &m_ifr);
	
  m_addr.can_family  = AF_CAN;
  m_addr.can_ifindex = m_ifr.ifr_ifindex;

  if(verbose)
    printf("%s at index %d\n", m_ifname, m_ifr.ifr_ifindex);

  if(bind(m_socket, (struct sockaddr *)&m_addr, sizeof(m_addr)) < 0) 
    perror("Error in socket bind");
  
}


void
Canopen_socket::print_frame(struct can_frame frame)
{
  
  if(frame.can_id > 0x580 && frame.can_id < 0x700)
    {
      if(frame.can_id > 0x600 )
	  printf("SDO message to [%X]: (",frame.can_id-0x600);
      else
	  printf("SDO message fr [%X]: (",frame.can_id-0x580);
      for(int i = 0 ; i < 8; i++)
	printf(" %02X",frame.data[i]);

      int sd = 4-((frame.data[0]&0x0f)>>2);
      printf(" ) \t %s\tIndex:0x%04X\tSubindex:0x%02X\tData:%d (0x%04X)\n",
	     ((frame.data[0]&0xf0)==0x40)?"READ":"WRITE",
      	     *(uint16_t*)(frame.data+1),
      	     *(uint8_t*)(frame.data+3),
      	     (sd==1)?*(uint8_t*)(frame.data+4):(sd==2)?*(uint16_t*)(frame.data+4):*(uint32_t*)(frame.data+4),
	     (sd==1)?*(uint8_t*)(frame.data+4):(sd==2)?*(uint16_t*)(frame.data+4):*(uint32_t*)(frame.data+4)
      	     );
    }
}



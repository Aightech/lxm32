#ifndef _CANOPEN_H_
#define _CANOPEN_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstdint>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define FRAME_SIZE 8
#define SDO_W 1
#define SDO_R 0


class Canopen_socket 
{
 public:
  Canopen_socket(const char* ifname,bool verbose = false);

  template <typename T=uint8_t>
  int send_SDO(uint8_t nodeID, uint8_t dlc, bool w, uint16_t index, uint8_t subindex, T data=0)
  {
    m_frame.can_id  = 0x600+nodeID;
    m_frame.can_dlc = dlc;
    memset( m_frame.data , 0, 8 );
    
    long unsigned i,sd = sizeof(T);//data size 

    m_frame.data[0] = !w?0x40:(0x23| ((4-sd)<<2));//1b:F 2b:B 4b:3
    m_frame.data[1] = index;
    m_frame.data[2] = index>>8;
    m_frame.data[3] = subindex;
    for(i =0; i<sd; i++)
      m_frame.data[4+i] = data>>(8*i);

    if(m_verbose)
      print_frame(m_frame);
    
    write(m_socket, &m_frame, sizeof(struct can_frame));

    recv_SDO();
  }

  int recv_SDO()
  { 
    int n = read(m_socket, &m_frame, sizeof(struct can_frame));
    if(m_verbose)
      print_frame(m_frame);
    
  }

  void print_frame(struct can_frame m_frame);
  

  
 private:
  int m_socket;
  char m_ifname[20];

  
  struct sockaddr_can m_addr;
  struct can_frame m_frame;
  struct ifreq m_ifr;
  
  bool m_verbose;
  
};

#endif

#ifndef _LEXIUM32A_CANOPEN_H_
#define _LEXIUM32A_CANOPEN_H_

#include "canopen_socket.cpp"

class LXM32 
{
 public:
  LXM32(const char* ifname,bool verbose = false);

  void init()
  {};


  void start()
  {};


  void stop()
  {};
  

  
 private:
  Canopen_socket m_can;
  bool m_verbose;
  
};

#endif

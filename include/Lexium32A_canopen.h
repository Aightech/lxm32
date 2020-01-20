#ifndef _LEXIUM32A_CANOPEN_H_
#define _LEXIUM32A_CANOPEN_H_

#include "canopen_socket.h"
#include "LXM32A_CANopen_register.h"
#include <string>

#define NB_PARAM 5

#define IND_CANaddress          0x30410002
#define IND_CANbaud             0x30410003
#define IND__CanDiag            0x30410006
#define IND__ManuSdoAbort       0x3041000A
#define IND_CANpdo1Event        0x3041000B
#define IND_CANpdo2Event        0x3041000C
#define IND_CANpdo3Event        0x3041000D
#define IND_CANpdo4Event        0x3041000E

#define IND__CTRL_KPid          0x30110001
#define IND__CTRL_TNid          0x30110002
#define IND__CTRL_KPiq          0x30110003
#define IND__CTRL_TNiq          0x30110004
#define IND_CTRL_vPIDDTime      0x30110005
#define IND_CTRL_vPIDDPart      0x30110006
#define IND_CTRL_TAUnact        0x30110008
#define IND_CTRL_SpdFric        0x30110009
#define IND_CTRL_KFAcc          0x3011000A
#define IND_CTRL_I_max          0x3011000C
#define IND_LIM_I_maxQSTP       0x3011000D
#define IND_LIM_I_maxHalt       0x3011000E
#define IND_CTRL_I_max_fw       0x3011000F
#define IND_CTRL_v_max          0x30110010
#define IND_CTRL_ParChgTime     0x30110014
#define IND_CTRL_GlobGain       0x30110015
#define IND_CTRL_ParSetCopy     0x30110016
#define IND__CTRL_ActParSet     0x30110017
#define IND_CTRL_PwrUpParSet    0x30110018
#define IND_CTRL_SelParSet      0x30110019
#define IND_CLSET_ParSwiCond    0x3011001A
#define IND_CLSET_winTime       0x3011001B
#define IND_CLSET_p_DiffWin     0x3011001C
#define IND_CLSET_v_Threshol    0x3011001D
#define IND_CTRL_VelObsActiv    0x30110022
#define IND_CTRL_VelObsDyn      0x30110023
#define IND_CTRL_VelObsInert    0x30110024
#define IND_CLSET_p_DiffWin_usr 0x30110025
#define IND_CTRL_SmoothCurr     0x30110026

#define IND__IO_act             0x30080001
#define IND__IO_DI_act          0x3008000F
#define IND_BRK_release         0x3008000A
#define IND__IO_DQ_act          0x30080010
#define IND_IO_DQ_set           0x30080011
#define IND_DI_0_Debounce       0x30080020
#define IND_DI_1_Debounce       0x30080021
#define IND_DI_2_Debounce       0x30080022
#define IND_DI_3_Debounce       0x30080023
#define IND__IO_STO_act         0x30080026
#define IND_IOsigVelLim         0x30080027
#define IND_IOsigCurrLim        0x30080028

#define PARAM_NAME {"hey", "hoy"}





class LXM32 
{
 public:
 LXM32(const char* ifname,uint16_t can_address, bool verbose = false): m_can(ifname,verbose), m_can_address(can_address), m_verbose(verbose)
    {
      
    };

  

  void init()
  {};


  void start()
  {
    uint16_t t = 0x1234;

    uint16_t index = 0x1000;
    uint8_t  subindex = 0;
    get_param();
};


  void stop()
  {};

  void get_param()
  {
    m_can.send_SDO(m_can_address , FRAME_SIZE, SDO_R, IND_CANaddress);
    m_can.send_SDO(m_can_address , FRAME_SIZE, SDO_R, IND_CANbaud);
    
  }

  
 private:
  Canopen_socket m_can;
  bool m_verbose;
  uint16_t m_can_address;
  uint32_t    m_param_val[NB_PARAM];
  std::string m_param_nam[2]=PARAM_NAME;
  uint32_t    m_param_reg[NB_PARAM];
  
  
};

#endif

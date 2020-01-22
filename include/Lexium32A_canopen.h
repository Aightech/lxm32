#ifndef _LEXIUM32A_CANOPEN_H_
#define _LEXIUM32A_CANOPEN_H_

#include "canopen_socket.h"
//#include "LXM32A_CANopen_register.h"
#include <string>
#include <unistd.h>


#define REG_CANaddress          0x30410002
#define REG_CANbaud             0x30410003

#define REG__DCOMstatus         0x60410000
#define REG_DCOMopmode          0x60600000
#define REG_DCOMcontrol         0x60400000

#define REG_PPp_target          0x607A0000
#define REG_PPv_target          0x60810000
#define REG_PPoption            0x60F20000

#define REG_JOGactivate         0x301B0009


#define MODE_ProfilePosition 1

#define OP_SHUTDOWN     0x0006
#define OP_SWITCHON     0x0007
#define OP_DISABLEVOL   0x0000
#define OP_QUICKSTEP    0x0002
#define OP_DISABLEOP    0x0007
#define OP_ENABLEOP     0x000F
#define OP_FAULTRESEST  0x0080





class LXM32 
{
 public:
 LXM32(const char* ifname,uint16_t can_id, bool verbose = false):
  m_verbose(verbose),
    m_can(ifname,verbose),
    m_can_PDO1(ifname, 0x180 +can_id, verbose),
    m_can_PDO2(ifname, 0x280 +can_id, verbose),
    m_can_PDO3(ifname, 0x380 +can_id, verbose),
    m_can_PDO4(ifname, 0x480 +can_id, verbose),
    m_can_SDO(ifname,  0x580 +can_id, verbose),
    m_can_id(can_id)
    {
      
      /* get_param(); */
      
      /* init(); */
      
      /* print_param(); */
      
      /* new_pos(30000, 0, 60); */
      
      
      m_can.set_PDO<1, R_PDO>(m_can_id);
      //m_can.send_PDO<1>( m_can_id,(uint16_t)OP_DISABLEVOL);
      

      
    };

  

  void init()
  {

    m_can_SDO.set_PDO<2,R_PDO>(m_can_id);
    m_can_SDO.set_PDO<2,T_PDO>(m_can_id);
    
    m_can.send_NMT(NMT_START);
    
    m_can.send_PDO(PDO_2, m_can_id,(uint16_t)OP_DISABLEVOL, (int32_t)0x00);
    m_can.send_PDO(PDO_2, m_can_id,(uint16_t)OP_SHUTDOWN,   (int32_t)0x00);
    m_can.send_PDO(PDO_2, m_can_id,(uint16_t)OP_ENABLEOP,   (int32_t)0x00);

    
    m_can.send_SDO(m_can_id , SDO_W, 0x60830000, 2000); 
    m_can.send_SDO(m_can_id , SDO_W, 0x60840000, 4000); 
    m_can.send_SDO(m_can_id , SDO_W, 0x60810000, 4000);
    usleep(10000);

  };


  void start()
  {
    
  };

  void new_pos(int32_t pos, bool abs, uint32_t spd=0)
  {
    m_PPp_target = pos;
    m_PPv_target = (spd!=0)?spd:m_PPv_target;
    m_PPoption = (abs)?2:0;

    if(m_m_dcom_mode == MODE_ProfilePosition)
      {
	m_dcom_mode = MODE_ProfilePosition;
	m_can.send_SDO(m_can_id , SDO_W, REG_DCOMopmode, m_dcom_mode);
      }

    
    m_can.send_PDO(PDO_2, m_can_id,(uint16_t)0x4F,(int32_t)m_PPp_target);
    m_can.send_PDO(PDO_2, m_can_id,(uint16_t)0x5F,(int32_t)m_PPp_target);
  };


  void stop()
  {};

  void get_param()
  {
    m_can_id   = (uint16_t)m_can.send_SDO(m_can_id, SDO_R, REG_CANaddress);
    m_can_baud = (uint16_t)m_can.send_SDO(m_can_id, SDO_R, REG_CANbaud);
    m_dcom_status = (uint16_t)m_can.send_SDO(m_can_id, SDO_R, REG__DCOMstatus);
  };

  void print_param()
  {
    
    printf("> LXM32A Parameters\n");
    printf("\t>> CANopen bus info:\n");
    printf("\t\t >>> Node_id: %03d\n", m_can_id);
    printf("\t\t >>> Baud: %d kps\n",  m_can_baud);
    printf("\t>> Operating State info:\n");
    
    
    if(m_dcom_status & 0x0008)//FAULT
      {
	m_op_state=6;
	if(m_dcom_status & 0x0007)
	  m_op_state=7;
      }
    else if(m_dcom_status & 0x0040)//DISABLE
      m_op_state=0;
    else if((m_dcom_status&0x0020)==0)
      m_op_state=5;
    else
      {
	m_op_state= (m_dcom_status&0x0007);
	m_op_state= (m_op_state==0x00)?0:(m_op_state==0x01)?1:(m_op_state==0x03)?2:3;
	m_op_state+=1;
      }
    printf("\t\t>>> [ %s ]\n",m_op_state_str[m_op_state]);

    printf("\t\t>>> Voltage: %s\n",(m_dcom_status&0x10)?"ON":"OFF");
    printf("\t\t>>> Error Msg: %s\n",(m_dcom_status&0x80)?"YES":"NO");

    printf("\t\t>>> Halt: %s\n",(m_dcom_status&0x0100)?"ACTIVATED":"DESACTIVATED");

    printf("\t\t>>> Field bus: %s\n",(m_dcom_status&0x0200)?"ACTIVATED":"DESACTIVATED");
    printf("\t\t>>> Error : %s\n",(m_dcom_status&0x2000)?"YES":"NO");

    printf("\t\t>>> Operating : %s\n",(m_dcom_status&0x4000)?"NO":"YES");
    printf("\t\t>>> Valid Zero Point : %s\n",(m_dcom_status&0x8000)?"YES":"NO");
    
    
  };

  
 private:
  
  bool m_verbose;
  
  Canopen_socket m_can;
  
  Canopen_socket m_can_PDO1;
  Canopen_socket m_can_PDO2;
  Canopen_socket m_can_PDO3;
  Canopen_socket m_can_PDO4;
  
  Canopen_socket m_can_SDO;

  uint16_t m_can_id;
  uint16_t m_can_baud;
  uint16_t m_dcom_status;

  int8_t m_dcom_mode; 

  //Position profil param
  int32_t  m_PPp_target;
  int32_t m_PPv_target=60;
  uint16_t m_PPoption;

  uint8_t m_op_state;
  const char* m_op_state_str[8]={"DISABLED",
				 "NOT READY",
				 "READY",
				 "ON",
				 "ENABLED",
				 "QUICK STOP",
				 "FAULT",
				 "FAULT, REACTION ACTIVE"};
  
  
};

#endif

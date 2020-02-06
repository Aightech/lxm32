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
#define PPctrl_SET_POINT  0x0010
#define PPctrl_ON_TARGET  0x0040
#define PPctrl_ON_DIRECT  0x0020
#define PPctrl_RELATIVE     0x0040
#define PPctrl_ABSOLUTE     0x0000

#define OP_SHUTDOWN     0x0006
#define OP_SWITCHON     0x0007
#define OP_DISABLEVOL   0x0000
#define OP_QUICKSTOP    0x0002
#define OP_DISABLEOP    0x0007
#define OP_ENABLEOP     0x000F
#define OP_FAULTRESEST  0x0080





class LXM32 
{
 public:

 /*!
   *  \brief Constructor
   *  \param ifname : Name of the CAN interface. 
   *  \param can_id : Node CAN ID of the driver.
   */
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
      m_ifavailable = m_can.is_available();
    };

  
    /*!
     *  \brief return true if the can interface is available
     */
    bool
    is_available()
    {
        return m_ifavailable;
    };
  

  void init()
  {

    m_can_id   = (uint16_t)m_can_SDO.send_SDO(m_can_id, SDO_R, REG_CANaddress);
    m_can_baud = (uint16_t)m_can_SDO.send_SDO(m_can_id, SDO_R, REG_CANbaud);
    m_dcom_status = (uint16_t)m_can_SDO.send_SDO(m_can_id, SDO_R, REG__DCOMstatus);
    if(m_verbose)
      print_status();

    //init R_PDO2 and T_PDO 
    m_can_SDO.set_PDO<2,R_PDO>(m_can_id);
    m_can_SDO.set_PDO<2,T_PDO>(m_can_id);
    

	

    
    m_can_SDO.send_SDO(m_can_id , SDO_W, 0x60830000, 2000);
    m_can_SDO.send_SDO(m_can_id , SDO_W, 0x60840000, 4000);
    m_can_SDO.send_SDO(m_can_id , SDO_W, 0x60810000, 4000);
    usleep(10000);

  };


  void start()
  {
    m_can.send_NMT(NMT_START);
    
    //set control to put in operating mode
    
    m_can.send_PDO<2>(m_can_id,(uint16_t)OP_DISABLEVOL,   (int32_t)0x00);
    //m_can_PDO1.recv(m_dcom_status);

    m_dcom_control = OP_ENABLEOP | PPctrl_RELATIVE| PPctrl_ON_DIRECT;
    print_control();
    m_can.send_PDO<2>(m_can_id,(uint16_t)m_dcom_control,(int32_t)m_PPp_target);
    //m_can_PDO1.recv(m_dcom_status);
    usleep(100000);
  };

  

  void stop()
  {
    m_can.send_NMT(NMT_STOP);
  };

  void new_pos(int32_t pos)
  {
    m_PPp_target = pos;

    set_mode(MODE_ProfilePosition);
    
    if(m_verbose)
      print_control();
    m_can.send_PDO<2>(m_can_id,(uint16_t)(m_dcom_control),(int32_t)m_PPp_target);
    m_can.send_PDO<2>(m_can_id,(uint16_t)(m_dcom_control|PPctrl_SET_POINT),(int32_t)m_PPp_target);
  };

  void set_mode(int8_t mode)
  {
     if(m_dcom_mode != mode)
      {
	m_dcom_mode = MODE_ProfilePosition;
	m_can.send_SDO(m_can_id , SDO_W, REG_DCOMopmode, m_dcom_mode);
      }
  }


  void get_param()
  {
    m_can_id   = (uint16_t)m_can.send_SDO(m_can_id, SDO_R, REG_CANaddress);
    m_can_baud = (uint16_t)m_can.send_SDO(m_can_id, SDO_R, REG_CANbaud);
  };

  void print_status()
  {
    
    printf("> LXM32A Status\n");
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

    if(m_dcom_mode==1)
      {
	printf("\t>> Position Profile:\n");
	printf("\t\t>>> Target : %s\n",((m_dcom_status&0x40)==0x40)?"NOT REACHED":"REACHED");
	printf("\t\t>>> Position : %s\n",((m_dcom_status&0x40)==0x40)?"POSSIBLE":"IMPOSSIBLE");
      }
    
  };

  void print_control()
  {
    printf("> LXM32A Control s\n");
    printf("\t>> Operating Control info:\n");

    if((m_dcom_control&0x87)==OP_SHUTDOWN)//FAULT
      m_ctrl_state=0;
    else if((m_dcom_control&0x8F)==OP_SWITCHON)
      m_ctrl_state=1;
    else if((m_dcom_control&0x82)==OP_DISABLEVOL)
      m_ctrl_state=2;
    else if((m_dcom_control&0x86)==OP_QUICKSTOP)
      m_ctrl_state=3;
    else if((m_dcom_control&0x8F)==OP_DISABLEOP)
      m_ctrl_state=4;
    else if((m_dcom_control&0x8F)==OP_ENABLEOP)
      m_ctrl_state=5;
    else if((m_dcom_control&0x80)==OP_FAULTRESEST)
      m_ctrl_state=6;
    printf("\t\t>>> [ %s ]\n",m_op_control_str[m_ctrl_state]);

    if(m_dcom_mode==1)
      {
	printf("\t>> Position Profile:\n");
	printf("\t\t>>> Set point: %s\n",((m_dcom_control&0x0010)==0x0010)?"SET": "UNSET");
	       
	if((m_dcom_control&0x0030)==0x0000)
	  printf("\t\t>>> MODE: on set point \n");
	if((m_dcom_control&0x0030)==0x0020)
	  printf("\t\t>>> MODE: on target reached \n");
	if((m_dcom_control&0x0010)==0x0010)
	  printf("\t\t>>> MODE: immediately \n");       
	printf("\t\t>>> Movement : %s\n",((m_dcom_status&0x40)==0x40)?"ABSOLUTE":"RELATIVE");
      }
    
    
  };

  
 private:
  
  bool m_verbose;
  bool m_ifavailable;
  
  Canopen_socket m_can;
  
  Canopen_socket m_can_PDO1;
  Canopen_socket m_can_PDO2;
  Canopen_socket m_can_PDO3;
  Canopen_socket m_can_PDO4;
  
  Canopen_socket m_can_SDO;

  uint16_t m_can_id;
  uint16_t m_can_baud;
  uint16_t m_dcom_status;
  uint16_t m_dcom_control;

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

  uint8_t m_ctrl_state;
  const char* m_op_control_str[8]={"SHUTDOWN",
				   "SWITCH ON",
				   "DISABLE VOLTAGE",
				   "QUICK STOP",
				   "DISABLE OPERATION",
				   "ENABLE OPERATION",
				   "FAULT RESET"};
  
  
};

#endif

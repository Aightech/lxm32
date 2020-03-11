#ifndef _LEXIUM32A_CANOPEN_H_
#define _LEXIUM32A_CANOPEN_H_

#include "canopen_socket.h"
//#include "LXM32A_CANopen_register.h"
#include <string>
#include <unistd.h>

#define REG_CANaddress 0x30410002
#define REG_CANbaud 0x30410003

#define REG__DCOMstatus 0x60410000
#define REG_DCOMopmode 0x60600000
#define REG_DCOMcontrol 0x60400000

#define REG_PPp_target 0x607A0000
#define REG_PPv_target 0x60810000
#define REG_RAMP_v_acc 0x60830000
#define REG_RAMP_v_dec 0x60840000

#define REG_PPoption 0x60F20000

#define REG_JOGactivate 0x301B0009

#define MODE_ProfilePosition 1
#define PPctrl_SET_POINT 0x0010
#define PPctrl_ON_TARGET 0x0040
#define PPctrl_ON_DIRECT 0x0020
#define PPctrl_RELATIVE 0x0040
#define PPctrl_ABSOLUTE 0x0000

#define OP_SHUTDOWN 0x0006
#define OP_SWITCHON 0x0007
#define OP_DISABLEVOL 0x0000
#define OP_QUICKSTOP 0x0002
#define OP_DISABLEOP 0x0007
#define OP_ENABLEOP 0x000F
#define OP_FAULTRESEST 0x0080

class LXM32
{
    public:
    /*!
     *  \brief Constructor
     *  \param ifname : Name of the CAN interface.
     *  \param can_id : Node CAN ID of the driver.
     */
    LXM32(const char *ifname, uint16_t can_id, bool verbose = false);

    /*!
     *  \brief return true if the can interface is available
     */
    bool
    is_available()
    {
        return m_available;
    };

    int32_t
    init();

    void
    start(int8_t mode, uint16_t control=0);

    void
    stop();

    void
    set_mode(int8_t mode);

    //Motion profile
    void
    setSpeed(uint32_t speed);
    void
    setAccel(uint32_t acc);
    void
    setDecel(uint32_t dec);
    void
    new_pos(int32_t pos);
    void
    new_spd(int32_t spd);

    void
    get_param();

    void
    print_status();

    void
    print_control();

    private:
    bool m_verbose;
    bool m_available;

    Canopen_socket m_can;

    Canopen_socket m_can_PDO1;
    Canopen_socket m_can_PDO2;
    Canopen_socket m_can_PDO3;
    Canopen_socket m_can_PDO4;

    Canopen_socket m_can_SDO;

    uint8_t m_can_id;
    uint16_t m_can_baud;
    uint16_t m_dcom_status;
    uint16_t m_dcom_control;

    int8_t m_dcom_mode;

    //Position profil param
    int32_t m_PPp_target;
    int32_t m_PPv_target = 60;

    uint8_t m_op_state;
    const char *m_op_state_str[8] = {
        "DISABLED", "NOT READY",  "READY", "ON",
        "ENABLED",  "QUICK STOP", "FAULT", "FAULT, REACTION ACTIVE"};

    uint8_t m_ctrl_state;
    const char *m_op_control_str[8] = {"SHUTDOWN",          "SWITCH ON",
                                       "DISABLE VOLTAGE",   "QUICK STOP",
                                       "DISABLE OPERATION", "ENABLE OPERATION",
                                       "FAULT RESET"};
};

#endif

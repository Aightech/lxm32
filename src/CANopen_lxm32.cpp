#include "CANopen_lxm32.h"

LXM32::LXM32(const char *ifname, uint16_t can_id, bool verbose)
    : m_verbose(verbose), m_available(true), m_socket(ifname, verbose), m_node_id(can_id) {

    if(!m_socket.bind())
        m_available = false;

    //init();
    //start(MODE_ProfilePosition, PPctrl_RELATIVE| PPctrl_ON_DIRECT);
}

void 
LXM32::init() {
    if(m_available) {
        //get status
        m_socket.send(CANopen::SDOOutboundRead(m_node_id, REG__DCOMstatus));
        m_dcom_status = m_socket.receive()->payload().value<uint16_t>();
        if(m_verbose)
            print_status();

        //init R_PDO2 and T_PDO2

        //set speed and accel
        set(Register::PPv_target, (uint32_t)4000);
        set(Register::RAMP_v_acc, (uint32_t)2000);
        set(Register::RAMP_v_dec, (uint32_t)4000);
        usleep(10000);
    }
}

void
LXM32::start(int8_t mode, uint16_t control) {

    if(m_available) {
        //start network manager
        m_can.send_NMT(NMT_START);
        //set mode
        set_mode(mode);
        //switch off first
        m_can.send_PDO<2>(m_can_id, (uint16_t)OP_DISABLEVOL, (int32_t)0x00);

        //set and send control to put in operating mode
        m_dcom_control = OP_ENABLEOP | control;
        m_can.send_PDO<2>(m_can_id, (uint16_t)m_dcom_control,
                          (int32_t)m_PPp_target);

        if(m_verbose)
            print_control();
        usleep(100000);
    }
}

void
LXM32::stop() {

    if(m_available) {
        fast_set(PDO::Control, (uint16_t)OP_DISABLEVOL);
        m_socket.send(CANopen::NMTMessage(CANopen::NMTMessage::Stopped, m_node_id));
    }
}

void
LXM32::fast_set(PDO pdo, uint16_t ctrl, int32_t param) {

    if(m_available) {
        CANopen::Payload payload;
	m_dcom_control = ctrl;
	payload << m_dcom_control;
        switch(pdo) {
        case PDO::Control:
            break;
        case PDO::Position:
            payload << param;
            break;
        case PDO::Velocity:
            payload << param;
            break;
        }
        CANopen::PDOMessage::PDOFunctionCode fn = (CANopen::PDOMessage::PDOFunctionCode)pdo;
        m_socket.send(CANopen::PDOMessage(fn, m_node_id, payload));
    }
}

void
LXM32::set_mode(int8_t mode) {

    if(m_available) {
      m_dcom_mode = MODE_ProfilePosition;
      set(Register::DCOMopmode, mode);
    }
}

void
LXM32::print_status() {

    if(m_available) {
        printf("> LXM32A Status\n");
        printf("\t>> CANopen bus info:\n");
        printf("\t\t >>> Node_id: %03d\n", m_node_id);
        printf("\t\t >>> Baud: %d kps\n", m_can_baud);
        printf("\t>> Operating State info:\n");

        if(m_dcom_status & 0x0008) //FAULT
        {
            m_op_state = 6;
            if(m_dcom_status & 0x0007)
                m_op_state = 7;
        } else if(m_dcom_status & 0x0040) //DISABLE
            m_op_state = 0;
        else if((m_dcom_status & 0x0020) == 0)
            m_op_state = 5;
        else {
            m_op_state = (m_dcom_status & 0x0007);
            m_op_state =
                (m_op_state == 0x00)
                    ? 0
                    : (m_op_state == 0x01) ? 1 : (m_op_state == 0x03) ? 2 : 3;
            m_op_state += 1;
        }
        printf("\t\t>>> [ %s ]\n", m_op_state_str[m_op_state]);

        printf("\t\t>>> Voltage: %s\n", (m_dcom_status & 0x10) ? "ON" : "OFF");
        printf("\t\t>>> Error Msg: %s\n",
               (m_dcom_status & 0x80) ? "YES" : "NO");

        printf("\t\t>>> Halt: %s\n",
               (m_dcom_status & 0x0100) ? "ACTIVATED" : "DESACTIVATED");

        printf("\t\t>>> Field bus: %s\n",
               (m_dcom_status & 0x0200) ? "ACTIVATED" : "DESACTIVATED");
        printf("\t\t>>> Error : %s\n", (m_dcom_status & 0x2000) ? "YES" : "NO");

        printf("\t\t>>> Operating : %s\n",
               (m_dcom_status & 0x4000) ? "NO" : "YES");
        printf("\t\t>>> Valid Zero Point : %s\n",
               (m_dcom_status & 0x8000) ? "YES" : "NO");

        if(m_dcom_mode == 1) {
            printf("\t>> Position Profile:\n");
            printf("\t\t>>> Target : %s\n", ((m_dcom_status & 0x40) == 0x40)
                                                ? "NOT REACHED"
                                                : "REACHED");
            printf("\t\t>>> Position : %s\n", ((m_dcom_status & 0x40) == 0x40)
                                                  ? "POSSIBLE"
                                                  : "IMPOSSIBLE");
        }
    }
}

void
LXM32::print_control() {

    if(m_available) {
        printf("> LXM32A Control s\n");
        printf("\t>> Operating Control info:\n");

        if((m_dcom_control & 0x87) == OP_SHUTDOWN) //FAULT
            m_ctrl_state = 0;
        else if((m_dcom_control & 0x8F) == OP_SWITCHON)
            m_ctrl_state = 1;
        else if((m_dcom_control & 0x82) == OP_DISABLEVOL)
            m_ctrl_state = 2;
        else if((m_dcom_control & 0x86) == OP_QUICKSTOP)
            m_ctrl_state = 3;
        else if((m_dcom_control & 0x8F) == OP_DISABLEOP)
            m_ctrl_state = 4;
        else if((m_dcom_control & 0x8F) == OP_ENABLEOP)
            m_ctrl_state = 5;
        else if((m_dcom_control & 0x80) == OP_FAULTRESEST)
            m_ctrl_state = 6;
        printf("\t\t>>> [ %s ]\n", m_op_control_str[m_ctrl_state]);

        if(m_dcom_mode == 1) {
            printf("\t>> Position Profile:\n");
            printf("\t\t>>> Set point: %s\n",
                   ((m_dcom_control & 0x0010) == 0x0010) ? "SET" : "UNSET");

            if((m_dcom_control & 0x0030) == 0x0000)
                printf("\t\t>>> MODE: on set point \n");
            if((m_dcom_control & 0x0030) == 0x0020)
                printf("\t\t>>> MODE: on target reached \n");
            if((m_dcom_control & 0x0010) == 0x0010)
                printf("\t\t>>> MODE: immediately \n");
            printf("\t\t>>> Movement : %s\n",
                   ((m_dcom_status & 0x40) == 0x40) ? "ABSOLUTE" : "RELATIVE");
        }
    }
}

#include "CANopen_driver.h"

CANopen::Driver::Driver(const char *ifname, uint16_t can_id, bool verbose)
    : m_ifname(ifname), m_verbose(verbose), m_available(true), m_socket(ifname, verbose), m_node_id(can_id) {

    if(!m_socket.bind())
        m_available = false;

    m_parameters[_DCOMstatus] = new Parameter("_DCOMstatus", (s8_t)0, _DCOMstatus);
    m_parameters[DCOMcontrol] = new Parameter("DCOMcontrol", (s16_t)0, DCOMcontrol);
    m_parameters[DCOMopmode] = new Parameter("DCOMopmode", (s8_t)0, DCOMopmode);
    m_parameters[PPp_target] = new Parameter("PPp_target", (s32_t)0, PPp_target);
    m_parameters[PPv_target] = new Parameter("PPv_target", (s32_t)0, PPv_target);
    m_parameters[RAMP_v_acc] = new Parameter("RAMP_v_acc", (s32_t)0, RAMP_v_acc);
    m_parameters[RAMP_v_dec] = new Parameter("RAMP_v_dec", (s32_t)0, RAMP_v_dec);

    m_parameters[_p_act] = new Parameter("_p_act", (s32_t)0, _p_act);
    m_parameters[_v_act] = new Parameter("_v_act", (s32_t)0, _p_act);
    m_parameters[_tq_act] = new Parameter("_tq_act", (s32_t)0, _p_act);

    // Map the 4 different PDO with default mapping
    map_PDO(PDO1Transmit, m_parameters[_DCOMstatus], 0);

    map_PDO(PDO2Transmit, m_parameters[_DCOMstatus], 0);
    map_PDO(PDO2Transmit, m_parameters[_p_act], m_parameters[_DCOMstatus]->size);

    map_PDO(PDO3Transmit, m_parameters[_DCOMstatus], 0);
    map_PDO(PDO3Transmit, m_parameters[_v_act], m_parameters[_DCOMstatus]->size);

    map_PDO(PDO4Transmit, m_parameters[_DCOMstatus], 0);
    map_PDO(PDO4Transmit, m_parameters[_tq_act], m_parameters[_DCOMstatus]->size);


    map_PDO(PDO2Receive , m_parameters[DCOMcontrol], 0);
    map_PDO(PDO2Receive, m_parameters[PPp_target], m_parameters[DCOMcontrol]->size);

    m_t_socket_thread = new std::thread(&Driver::T_socket, this);
    m_rpdo_socket_thread = new std::thread(&Driver::RPDO_socket, this);
}

void
CANopen::Driver::map_PDO(PDOFunctionCode fn, Parameter *param, int slot) {
    if(std::find(m_PDO_map[fn].begin(), m_PDO_map[fn].end(), param) == m_PDO_map[fn].end()) {
        param->link_to_pdo((Parameter::PDOFunctionCode)fn, slot);
        m_PDO_map[fn].push_back(param);
        activate_PDO(fn, true);
    }
}

void
CANopen::Driver::activate_PDO(PDOFunctionCode fn, bool set) {
    uint16_t index = 0;
    if((fn & 0x80) == 0x00) //T_PDO
    {
        index = (0x1400 + (fn >> 8) - 2);
    } else {
        index = (0x1800 + (fn >> 8) - 1);
    }
    m_socket.send(CANopen::SDOOutboundWrite(m_node_id, index, 1, (set ? 0x04000000 : 0x80000000) + fn + m_node_id));
}

void
CANopen::Driver::send(Parameter *param) {
    if(m_available)
        m_socket.send(CANopen::SDOOutboundWrite(m_node_id, param->index, param->subindex, param->payload()));
}

void
CANopen::Driver::update(Parameter *param) {
    if(m_available)
        m_socket.send(CANopen::SDOOutboundRead(m_node_id, param->index, param->subindex));
}

void
CANopen::Driver::T_socket() {
    //create a socket dedicated to the incomming pdo (T_PDO)
    Socket socket(m_ifname, m_verbose);
    //set the filter to only receive T_PDO messages
    socket.add_filter(1, 0xF80 + m_node_id); //all message with the right node id
    socket.bind();

    std::shared_ptr<Message> msg;
    Payload p;
    Message::FunctionCode fn;
    while(1) {
        //try to receive pdo
        msg = socket.receive();
        if(msg.get() != nullptr) { //if pdo received
            fn = msg->function_code();
            p = msg->payload();
            switch(fn) {
            case CANopen::Message::TimeStamp:
                break;
            case CANopen::Message::PDO1Transmit:
            case CANopen::Message::PDO2Transmit:
            case CANopen::Message::PDO3Transmit:
            case CANopen::Message::PDO4Transmit:
                if(m_PDO_map.count((PDOFunctionCode)fn)) {
                    for(auto param : m_PDO_map[(PDOFunctionCode)fn])
                        param->from_payload(p, true);
                }
                break;
            case CANopen::Message::SDOReceive:
                m_parameters[(Register)msg->id()]->from_payload(p);
                break;
            default:
                break;
            }
        }
    }
}

void
CANopen::Driver::RPDO_socket() {
    //create a socket dedicated to the incomming pdo (T_PDO)
    Socket socket(m_ifname, m_verbose);
    socket.bind();
    PDOFunctionCode RPDO_fn_list[] = {PDO1Receive, PDO2Receive, PDO3Receive, PDO4Receive};

    for(auto fn : RPDO_fn_list) {
        if(m_PDO_map.count(fn)) {
            bool dont_update = true;
            Parameter *order[4] = {nullptr, nullptr, nullptr, nullptr};
            Payload payload;
            for(auto p : m_PDO_map[fn]) {
                dont_update *= p->is_updated.test_and_set();
                order[p->pdo_slot] = p;
            }
            if(!dont_update) {
                for(auto p : order) {
                    if(p != nullptr)
                        payload << p->payload();
                }
                socket.send(PDOMessage((PDOMessage::PDOFunctionCode) fn, m_node_id, payload));
            }
        }
    }
}

void
CANopen::Driver::set_mode(OperationMode mode) {
    if(m_available) {
        m_parameters[DCOMopmode]->set(mode);
        send(m_parameters[DCOMopmode]);
    }
}

void
CANopen::Driver::print_status() {

    if(m_available) {
        printf("> Driver Status\n");
        printf("\t>> Operating State info:\n");

        printf("\t\t>>> State     :");
        switch(get_state() & State::mask) {
        case NotReadyToSwitchtON:
            printf("Not Ready to Switch ON");
            break;
        case SwitchONDisabled:
            printf("Switch ON Disabled");
            break;
        case ReadyToSwitchON:
            printf("Ready To Switch ON");
            break;
        case SwitchedON:
            printf("Switch ON");
            break;
        case OperationEnabled:
            printf("Operation Enabled");
            break;
        case Fault:
            printf("Fault");
            break;
        case QuickStop:
            printf("Quick Stop");
            break;
        }
        printf("\n");
        s16_t s = get_state();
        printf("\t\t>>> Voltage   : %s\n", (s & StatusBits::Voltage_disable) ? "OFF" : "ON");
        printf("\t\t>>> Quick Stop: %s\n", (s & StatusBits::Quick_stop) ? "Inactive" : "Active");
        printf("\t\t>>> Warning   : %s\n", (s & StatusBits::Warning) ? "Present" : "None");
        printf("\t\t>>> Remote         : %s\n", (s & StatusBits::Quick_stop) ? "ON" : "OFF");
        printf("\t\t>>> Target    : %s\n", (s & StatusBits::Quick_stop) ? "Reached" : "Not reached");
        printf("\t\t>>> Int.Limit : %s\n", (s & StatusBits::Quick_stop) ? "Active" : "Inactive");

        switch(get_mode()) {
        case ProfilePosition:
            printf("\t\t>>> Setpoint   : %s\n", ((s & StatusBits::Operation_Mode) == 0x1000) ? "Aknowledged" : "Not Aknowledged");
            printf("\t\t>>> Folowing   : %s\n", ((s & StatusBits::Operation_Mode) == 0x2000) ? "Error" : "OK");
            break;
        case Velocity:
            break;
        case ProfileVelocity:

            printf("\t\t>>> Speed      : %s\n", ((s & StatusBits::Operation_Mode) == 0x1000) ? "Zero" : "Not Zero");
            printf("\t\t>>> Max slip.  : %s\n", ((s & StatusBits::Operation_Mode) == 0x2000) ? "Error" : "OK");
            break;
        case ProfileTorque:
            break;
        case Homing:

            printf("\t\t>>> Homing     : %s\n", ((s & StatusBits::Operation_Mode) == 0x1000) ? "Attained" : ((s & StatusBits::Operation_Mode) == 0x2000) ? "Error" : "Progress");
            break;
        case InterpolatedPosition:
            break;
        }

        print_manufacturer_status();
    }
}

void
CANopen::Driver::print_control(Control control) {

    if(m_available) {
        printf("> Driver Control s\n");
        printf("\t>> Operating Control info:\n");
    }
}

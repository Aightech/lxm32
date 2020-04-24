#include "CANopen_driver.h"

CANopen::Driver::Driver(const char *ifname, uint16_t can_id, bool verbose)
    : m_ifname(ifname), m_verbose(verbose), m_available(true), m_socket(ifname, verbose), m_node_id(can_id) {

    if(!m_socket.bind())
        m_available = false;

    for(int i = 0; i < NB_PDO; i++)
        for(int j = 0; j < MAX_PDO_SLOT; j++) {
            m_T_PDO_mapping[i][j] = nullptr;
            m_T_PDO_mapping_t[i][j] = 0;
        }

    // Map the 4 different PDO with default mapping
    map_PDO(0, 0, &m_statusWord);

    map_PDO(2, 0, &m_statusWord);
    map_PDO(2, 1, &m_current_position);

    map_PDO(3, 0, &m_statusWord);
    map_PDO(3, 1, &m_current_velocity);

    map_PDO(4, 0, &m_statusWord);
    map_PDO(4, 1, &m_current_torque);

    m_pdo_socket_thread = new std::thread(&Driver::T_PDO_socket, this);
}

bool
CANopen::Driver::set_state(Control control) {
    if(m_available)
        m_socket.send(CANopen::PDOMessage((CANopen::PDOMessage::PDOFunctionCode)PDO1Receive, m_node_id, control));
    //TODO: make some test to ensure the change of state
    return true;
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
    //TODO: if set a T_PDO, run a thread to read incoming PDO
}

void
CANopen::Driver::send_PDO(PDOFunctionCode pdo, Payload payload) {
    if(m_available)
        m_socket.send(CANopen::PDOMessage((PDOMessage::PDOFunctionCode)pdo, m_node_id, payload));
}

void
CANopen::Driver::set_mode(OperationMode mode) {

    if(m_available) {
        m_opMode = mode;
        set(Register::OpMode, mode);
    }
}

void
CANopen::Driver::set(Register reg, Payload param) {
    if(m_available)
        m_socket.send(CANopen::SDOOutboundWrite(m_node_id, reg, param));
}

void
CANopen::Driver::set_target(uint32_t target, bool byPDO, PDOFunctionCode pdo) {
  if(pdo)
    send_PDO(pdo, target);
  else
     

}

void
CANopen::Driver::T_PDO_socket() {
    //create a socket dedicated to the incomming pdo (T_PDO)
    Socket socket(m_ifname, m_verbose);
    //set the filter to only receive T_PDO messages
    socket.add_filter(4,
                      PDO1Transmit + m_node_id,
                      PDO2Transmit + m_node_id,
                      PDO3Transmit + m_node_id,
                      PDO4Transmit + m_node_id);
    socket.bind();

    std::shared_ptr<Message> msg;
    Payload p;
    int pdo_n, slot, index;
    size_t size;
    while(1) {
        //try to receive pdo
        msg = socket.receive();
        if(msg.get() != nullptr) { //if pdo received
            p = msg->payload();
            pdo_n = (msg->function_code() >> 8) - 1; //pdo number
            slot = 0;
            index = 0;
            size = m_T_PDO_mapping_t[pdo_n][0]; //get the size of the parameter mapped for this pdo
            while(size != 0) {                  //while a parameter is mapped in the following slot.
                if(index + size <= p.size()) {  //ensure the payload contain the data.
                    switch(size) {              //store the received value in the coresponding parameter (with the right size)
                    case 0:
                        break;
                    case 1:
                        m_ro_mutex.lock();
                        *((s8_t *)m_T_PDO_mapping[pdo_n][slot]) = p.value<s8_t>(index);
                        m_ro_mutex.unlock();
                        break;
                    case 2:
                        m_ro_mutex.lock();
                        *((s16_t *)m_T_PDO_mapping[pdo_n][slot]) = p.value<s16_t>(index);
                        m_ro_mutex.unlock();
                        break;
                    case 4:
                        m_ro_mutex.lock();
                        *((s32_t *)m_T_PDO_mapping[pdo_n][slot]) = p.value<s32_t>(index);
                        m_ro_mutex.unlock();
                        break;
                    }
                    index += size;
                    size = (slot < MAX_PDO_SLOT) ? m_T_PDO_mapping_t[pdo_n][++slot] : 0;

                } else
                    throw std::runtime_error(std::string("Error: Wrong PDO mapping"));
            }
        }
    }
}

void
CANopen::Driver::print_status() {

    if(m_available) {
        printf("> Driver Status\n");
        printf("\t>> Operating State info:\n");

        printf("\t\t>>> State     :");
        switch(m_state & State::mask) {
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

        printf("\t\t>>> Voltage   : %s\n", (m_statusWord & StatusBits::Voltage_disable) ? "OFF" : "ON");
        printf("\t\t>>> Quick Stop: %s\n", (m_statusWord & StatusBits::Quick_stop) ? "Inactive" : "Active");
        printf("\t\t>>> Warning   : %s\n", (m_statusWord & StatusBits::Warning) ? "Present" : "None");
        printf("\t\t>>> Remote         : %s\n", (m_statusWord & StatusBits::Quick_stop) ? "ON" : "OFF");
        printf("\t\t>>> Target    : %s\n", (m_statusWord & StatusBits::Quick_stop) ? "Reached" : "Not reached");
        printf("\t\t>>> Int.Limit : %s\n", (m_statusWord & StatusBits::Quick_stop) ? "Active" : "Inactive");

        switch(m_opMode) {
        case ProfilePosition:
            printf("\t\t>>> Setpoint   : %s\n", ((m_statusWord & StatusBits::Operation_Mode) == 0x1000) ? "Aknowledged" : "Not Aknowledged");
            printf("\t\t>>> Folowing   : %s\n", ((m_statusWord & StatusBits::Operation_Mode) == 0x2000) ? "Error" : "OK");
            break;
        case Velocity:
            break;
        case ProfileVelocity:

            printf("\t\t>>> Speed      : %s\n", ((m_statusWord & StatusBits::Operation_Mode) == 0x1000) ? "Zero" : "Not Zero");
            printf("\t\t>>> Max slip.  : %s\n", ((m_statusWord & StatusBits::Operation_Mode) == 0x2000) ? "Error" : "OK");
            break;
        case ProfileTorque:
            break;
        case Homing:

            printf("\t\t>>> Homing     : %s\n", ((m_statusWord & StatusBits::Operation_Mode) == 0x1000) ? "Attained" : ((m_statusWord & StatusBits::Operation_Mode) == 0x2000) ? "Error" : "Progress");
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

#include "CANopen_driver.h"

CANopen::Driver::Driver(const char *ifname, uint16_t can_id, bool verbose)
  : m_ifname(ifname), m_verbose(verbose), m_available(true), m_socket(ifname, verbose), m_node_id(can_id) {

    if(!m_socket.bind())
        m_available = false;

    activated_PDO[0] = true;
}

bool
CANopen::Driver::set_state(Control control) {
    if(m_available)
        m_socket.send(CANopen::PDOMessage((CANopen::PDOMessage::PDOFunctionCode)PDO1Receive, m_node_id, control));
    //TODO: make some test to ensure the change of state
    return true;
}

void
CANopen::Driver::activate_PDO(uint8_t node_id, PDOFunctionCode fn, bool set) {
    uint16_t index = 0;
    if((fn & 0x80) == 0x00) //T_PDO
    {
        index = (0x1400 + (fn >> 8) - 2);
    } else {
        index = (0x1800 + (fn >> 8) - 1);
    }
    m_socket.send(CANopen::SDOOutboundWrite(node_id, index, 1, (set ? 0x04000000 : 0x80000000) + fn + node_id));
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

        printf("\t\t>>> Voltage   : %s\n", (m_status & StatusBits::Voltage_disable) ? "OFF" : "ON");
        printf("\t\t>>> Quick Stop: %s\n", (m_status & StatusBits::Quick_stop) ? "Inactive" : "Active");
        printf("\t\t>>> Warning   : %s\n", (m_status & StatusBits::Warning) ? "Present" : "None");
        printf("\t\t>>> Remote         : %s\n", (m_status & StatusBits::Quick_stop) ? "ON" : "OFF");
        printf("\t\t>>> Target    : %s\n", (m_status & StatusBits::Quick_stop) ? "Reached" : "Not reached");
        printf("\t\t>>> Int.Limit : %s\n", (m_status & StatusBits::Quick_stop) ? "Active" : "Inactive");

        switch(m_opMode) {
        case ProfilePosition:
            printf("\t\t>>> Setpoint   : %s\n", ((m_status & StatusBits::Operation_Mode) == 0x1000) ? "Aknowledged" : "Not Aknowledged");
            printf("\t\t>>> Folowing   : %s\n", ((m_status & StatusBits::Operation_Mode) == 0x2000) ? "Error" : "OK");
            break;
        case Velocity:
            break;
        case ProfileVelocity:

            printf("\t\t>>> Speed      : %s\n", ((m_status & StatusBits::Operation_Mode) == 0x1000) ? "Zero" : "Not Zero");
            printf("\t\t>>> Max slip.  : %s\n", ((m_status & StatusBits::Operation_Mode) == 0x2000) ? "Error" : "OK");
            break;
        case ProfileTorque:
            break;
        case Homing:

            printf("\t\t>>> Homing     : %s\n", ((m_status & StatusBits::Operation_Mode) == 0x1000) ? "Attained" : ((m_status & StatusBits::Operation_Mode) == 0x2000) ? "Error" : "Progress");
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

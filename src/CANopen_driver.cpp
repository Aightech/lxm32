#include "CANopen_driver.h"

CANopen::Driver::Driver(const char *ifname, uint16_t can_id, bool verbose)
    : m_ifname(ifname), m_verbose(verbose), m_available(true), m_socket(ifname, verbose), m_node_id(can_id) {


    m_parameters[_DCOMstatus] = new Parameter("_DCOMstatus", (uint16_t)0, _DCOMstatus);
    m_parameters[DCOMcontrol] = new Parameter("DCOMcontrol", (uint16_t)0, DCOMcontrol);
    m_parameters[DCOMopmode] = new Parameter("DCOMopmode", (uint8_t)0, DCOMopmode);
    m_parameters[PPp_target] = new Parameter("PPp_target", (int32_t)0x365b539, PPp_target);
    m_parameters[PPv_target] = new Parameter("PPv_target", (int32_t)0, PPv_target);
    m_parameters[RAMP_v_acc] = new Parameter("RAMP_v_acc", (int32_t)0, RAMP_v_acc);
    m_parameters[RAMP_v_dec] = new Parameter("RAMP_v_dec", (int32_t)0, RAMP_v_dec);

    m_parameters[_p_act] = new Parameter("_p_act", (int32_t)0, _p_act);
    m_parameters[_v_act] = new Parameter("_v_act", (int32_t)0, _p_act);
    m_parameters[_tq_act] = new Parameter("_tq_act", (int32_t)0, _p_act);
    
    m_parameters[HMmethod] = new Parameter("HMmethod", (int8_t)0, HMmethod);
    
    
    
    t_socket_flag.test_and_set();
    m_t_socket_thread = new std::thread(&Driver::T_socket, this);
    while(t_socket_flag.test_and_set());
   
    

    // Map the 4 different PDO with default mapping
    map_PDO(PDO1Transmit, m_parameters[_DCOMstatus], 0);

    map_PDO(PDO2Transmit, m_parameters[_DCOMstatus], 0);
    map_PDO(PDO2Transmit, m_parameters[_p_act], m_parameters[_DCOMstatus]->size);

//    map_PDO(PDO3Transmit, m_parameters[_DCOMstatus], 0);
//    map_PDO(PDO3Transmit, m_parameters[_v_act], m_parameters[_DCOMstatus]->size);

//    map_PDO(PDO4Transmit, m_parameters[_DCOMstatus], 0);
//    map_PDO(PDO4Transmit, m_parameters[_tq_act], m_parameters[_DCOMstatus]->size);

	map_PDO(PDO1Receive, m_parameters[DCOMcontrol], 0);
    map_PDO(PDO2Receive , m_parameters[DCOMcontrol], 0);
    map_PDO(PDO2Receive, m_parameters[PPp_target], m_parameters[DCOMcontrol]->size);
    
   
   	
   	rpdo_socket_flag.test_and_set();
    m_rpdo_socket_thread = new std::thread(&Driver::RPDO_socket, this);
    while(rpdo_socket_flag.test_and_set());
    
    
    m_socket.send(CANopen::NMTMessage(CANopen::NMTMessage::GoToOperational,0));
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
    std::string reg_name;
    
    
    if((fn & 0x80) == 0x00) //R_PDO
    {
        index = (0x1400 + (fn >> 8) - 2);
        reg_name = "R_PDO" + std::to_string((fn>>8)-1);
    } else {
        index = (0x1800 + (fn >> 8) - 1);
        reg_name = "T_PDO" + std::to_string(fn>>8);
    }
    reg_name+="_setting";
    
    Register reg = (Register)( (index<<16) + 0x0001);
	int32_t val = (set ? 0x04000000 : 0x80000000) + fn + m_node_id;
    if(m_parameters.count(reg)==0)
    {
    	m_parameters[reg] = new Parameter(reg_name, val, index, 1);
    	this->send(m_parameters[reg]);
    	m_parameters[reg]->sdo_flag.test_and_set();
    	std::cout << "Setting " + reg_name;
    	while(m_parameters[reg]->sdo_flag.test_and_set());
    	std::cout << "\xd" +reg_name << " " << ((set)?"activated":"desactivated") <<  "\n";
    }
    else if((int32_t)*m_parameters[reg] != val)
    {
    	*m_parameters[reg] = val;
    	this->send(m_parameters[reg]);
    	m_parameters[reg]->sdo_flag.test_and_set();
    	std::cout << "Setting " + reg_name;
    	while(m_parameters[reg]->sdo_flag.test_and_set());
    	std::cout << "\xd" +reg_name << " " << ((set)?"activated":"desactivated") << "\n";
    }
    
    //m_socket.send(CANopen::SDOOutboundWrite(m_node_id, index, 1, (set ? 0x04000000 : 0x80000000) + fn + m_node_id));
}

void
CANopen::Driver::send(Parameter *param) {
    if(m_available)
    {
    	param->sdo_flag.test_and_set();
        m_socket.send(CANopen::SDOOutboundWrite(m_node_id, param->index, param->subindex, param->payload()));
    }
}

void
CANopen::Driver::update(Parameter *param) {
    if(m_available)
        m_socket.send(CANopen::SDOOutboundRead(m_node_id, param->index, param->subindex));
}

void
CANopen::Driver::T_socket() {
    //create a socket dedicated to the incomming pdo (T_PDO)
    Socket socket(m_ifname, false);
    //set the filter to only receive T_PDO messages
    socket.add_filter({{m_node_id,0x07f}}); //all message with the right node id

    std::shared_ptr<Message> msg;
    Payload p;
    Message::FunctionCode fn;
    if(m_verbose)
    	std::cout << "Receiving socket thread ready.\n";
    t_socket_flag.clear();
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
            case CANopen::Message::SDOTransmit:
            	if(m_parameters.count((Register)msg->id()))
            	{
            		std::shared_ptr<CANopen::SDOMessage> sdo_msg = std::dynamic_pointer_cast<CANopen::SDOMessage>(msg);
		        	if(!sdo_msg->is_confirmation() && !sdo_msg->is_error())
		            	m_parameters[(Register)msg->id()]->from_payload(p);
		            m_parameters[(Register)msg->id()]->sdo_flag.clear();
		        }
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
    Socket socket(m_ifname, false);
    PDOFunctionCode RPDO_fn_list[] = {PDO1Receive, PDO2Receive, PDO3Receive, PDO4Receive};

	if(m_verbose)
		std::cout << "Sending PDO socket thread ready.\n";
	rpdo_socket_flag.clear();
    while(1)
    {
		for(auto fn : RPDO_fn_list) {
		    if(m_PDO_map.count(fn)) {
		        bool dont_update = true;
		        Payload order[4];
		        Payload payload;
		        
		        
		        if(fn==PDO1Receive && m_PDO_map[fn].size()>0)
		        {
		        	payload << m_PDO_map[fn][0]->payload();//save the actual value at the right sending posit
		            if(!m_PDO_map[fn][0]->is_updated.test_and_set())
		            	socket.send(PDOMessage((PDOMessage::PDOFunctionCode) fn, m_node_id, payload));
		        }
		        else
		        {
		        	for(auto p : m_PDO_map[fn]) {
				    	order[p->pdo_slot] << p->payload();//save the actual value at the right sending position
				    	if(p != m_parameters[DCOMcontrol])//update only if the other parameter has to be update
				        	dont_update *= p->is_updated.test_and_set(); // test if update was requsted
				    }
				    if(!dont_update) {//if at least one of the parameter had to be update then send the PDO
				        for(auto pay : order)
				        	if(pay.size()>0)
				        		payload << (Payload &&)pay;
				    	socket.send(PDOMessage((PDOMessage::PDOFunctionCode) fn, m_node_id, payload));
				    }
		        }
		        
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

bool
CANopen::Driver::set_position(int32_t target)
{
	switch(get_mode())
	{
		case ProfilePosition:
			if((*m_parameters[PPp_target] = target))
			{
				set_state((Control)(EnableOperation|0x0050));
				set_state((Control)(EnableOperation));
				return true;
			}
			return false;
		default:
			return false;
	}
};

bool
CANopen::Driver::set_velocity(int32_t target)
{
	switch(get_mode())
	{
		case ProfilePosition:
			return *m_parameters[PPv_target] = target;
		case ProfileVelocity:
			return *m_parameters[PVv_target] = target;
		default:
			return false;
	}
};

bool
CANopen::Driver::set_torque(int32_t target)
{
	switch(get_mode())
	{
		case ProfileTorque:
			return *m_parameters[PTtq_target] = target;
		default:
			return false;
	}
};

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
        int16_t s = get_state();
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

std::string
CANopen::Driver::ctrl_to_str(Control control) {
	switch(control)
	{
		case Shutdown: return "Shutdown";         /*!< goto ReadySwitchON */
        case SwitchON: return "SwitchON";        /*!< goto SwitchedON */
        case DisableVoltage: return "DisableVoltage";  /*!< goto SwitchONDisabled */
        case QuickStop: return "QuickStop";      /*!< goto QuickStopActiv */
        case EnableOperation: return "EnableOperation";  /*!< goto OperationEnabled */
        case FaultResest: return "FaultResest";
	}
	return "unknown control";
}

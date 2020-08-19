#include "CANopen_driver.h"

 
CANopen::Driver::Driver(const char *ifname, uint16_t can_id, int verbose_lvl)
    : m_ifname(ifname), m_verbose_level(verbose_lvl), m_available(true), m_socket(ifname, verbose_lvl), m_node_id(can_id) {


    m_parameters[_DCOMstatus] = new Parameter("_DCOMstatus", (uint16_t)0, _DCOMstatus, (verbose_lvl>0)?&print_status:nullptr);
    m_parameters[DCOMcontrol] = new Parameter("DCOMcontrol", (uint16_t)0, DCOMcontrol);
    m_parameters[DCOMopmode] = new Parameter("DCOMopmode", (uint8_t)0, DCOMopmode);
    m_parameters[_DCOMopmd_act] = new Parameter("_DCOMopmd_act", (uint8_t)0, _DCOMopmd_act);
    
    m_parameters[PPp_target] = new Parameter("PPp_target", (int32_t)0x365b539, PPp_target);
    m_parameters[PPv_target] = new Parameter("PPv_target", (int32_t)0, PPv_target);
    m_parameters[RAMP_v_acc] = new Parameter("RAMP_v_acc", (int32_t)0, RAMP_v_acc);
    m_parameters[RAMP_v_dec] = new Parameter("RAMP_v_dec", (int32_t)0, RAMP_v_dec);
    
    m_parameters[PVv_target] = new Parameter("PVv_target", (int32_t)0, PVv_target);
    m_parameters[PTtq_target] = new Parameter("PTtq_target", (int16_t)0, PTtq_target);

    m_parameters[_p_act] = new Parameter("_p_act", (int32_t)0, _p_act);
    m_parameters[_v_act] = new Parameter("_v_act", (int32_t)0, _p_act);
    m_parameters[_tq_act] = new Parameter("_tq_act", (int32_t)0, _p_act);
    
    m_parameters[HMmethod] = new Parameter("HMmethod", (int8_t)0, HMmethod);
    m_parameters[HMv] = new Parameter("HMv", (int32_t)100, HMv);
    m_parameters[HMv_out] = new Parameter("HMv_out", (uint32_t)10, HMv_out);
    
    
    
    t_socket_flag.test_and_set();
    m_t_socket_thread = new std::thread(&Driver::T_socket, this);
    while(t_socket_flag.test_and_set());
    

    // Map the different PDO with default mapping
    map_PDO(PDO1Transmit, m_parameters[_DCOMstatus], 0);
	map_PDO(PDO1Receive, m_parameters[DCOMcontrol], 0);
	
	

    

   	
   	rpdo_socket_flag.test_and_set();
    m_rpdo_socket_thread = new std::thread(&Driver::RPDO_socket, this);
    while(rpdo_socket_flag.test_and_set());
    
    
    m_socket.send(CANopen::NMTMessage(CANopen::NMTMessage::GoToOperational,0));
}

void
CANopen::Driver::map_PDO(PDOFunctionCode fn, Parameter *param, int slot) {
    if(std::find(m_PDO_map[fn].begin(), m_PDO_map[fn].end(), param) == m_PDO_map[fn].end()) {
        param->link_to_pdo((Parameter::PDOFunctionCode)fn, slot);
        activate_PDO(fn, true);
        m_PDO_map[fn].push_back(param);
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
    	while(m_parameters[reg]->sdo_flag.test_and_set());
    	
    	IF_VERBOSE(1, std::cout <<  reg_name << " " << ((set)?"activated":"desactivated") <<  "\n", m_verbose_level)
    	
    }
    else if((int32_t)*m_parameters[reg] != val)
    {
    	*m_parameters[reg] = val;
    	this->send(m_parameters[reg]);
    	while(m_parameters[reg]->sdo_flag.test_and_set());
    	
    	IF_VERBOSE(1, std::cout <<  reg_name << " " << ((set)?"activated":"desactivated") <<  "\n", m_verbose_level)
    }
    
    //m_socket.send(CANopen::SDOOutboundWrite(m_node_id, index, 1, (set ? 0x04000000 : 0x80000000) + fn + m_node_id));
}

void
CANopen::Driver::send(Parameter *param) {
    if(m_available)
    {
    	param->sdo_flag.test_and_set();
    	try {
		m_socket.send(CANopen::SDOOutboundWrite(m_node_id, param->index, param->subindex, param->payload()));
	}
	catch (const std::runtime_error& e) {
		std::cerr << e.what();
	}
        
    }
}

void
CANopen::Driver::update(Parameter *param) {
    if(m_available)
    {
    	param->sdo_flag.test_and_set();
        try {
		m_socket.send(CANopen::SDOOutboundRead(m_node_id, param->index, param->subindex));
	}
	catch (const std::runtime_error& e) {
		std::cerr << e.what();
	}
    }
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
    
    IF_VERBOSE(1, std::cout << "Receiving socket thread ready.\n";, m_verbose_level)
    
    
    t_socket_flag.clear();
    while(1) {
        //try to receive pdo
        msg = socket.receive();
        if(msg.get() != nullptr) { //if pdo received
            fn = msg->function_code();
            p = msg->payload();
            switch(fn) {
            case CANopen::Message::Emergency:
            {
            	std::shared_ptr<CANopen::EMCYMessage> emcy_msg = std::dynamic_pointer_cast<CANopen::EMCYMessage>(msg);
            	std::cout <<  "Error {" << (int)emcy_msg->node_id() << "} code: "<< std::hex << (int)emcy_msg->code() << "  reg:" << (int)emcy_msg->reg() << "\n";
            }
            	break;
            case CANopen::Message::TimeStamp:
                break;
            case CANopen::Message::PDO1Transmit:
            case CANopen::Message::PDO2Transmit:
            case CANopen::Message::PDO3Transmit:
            case CANopen::Message::PDO4Transmit:
                if(m_PDO_map.count((PDOFunctionCode)fn)) {
                	
                    for(auto param : m_PDO_map[(PDOFunctionCode)fn])
                    {
                        if(param->from_payload(p, param->pdo_slot))
                        	param->callback();
                        //if(param == m_parameters[DCOMcontrol]) 
                        	//std::cout << param->name << ": "<< std::hex << (uint16_t)*param << " " << p << "\n";
                    }
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

	
	IF_VERBOSE(1, std::cout << "Sending PDO socket thread ready.\n";, m_verbose_level)
	
	rpdo_socket_flag.clear();
    while(1)
    {
		for(auto fn : RPDO_fn_list) {
		    if(m_PDO_map.count(fn)) {
				bool should_update=false, should_update_PDO1=false;
		        Payload payload;

	        	for(auto p : m_PDO_map[fn])// for each parameter mapped on the pdo, get the payload and store it at the right positon + get if the data has to be updated.
	        	{
	        		//std::cout << p->name << "\n";
			    	payload.store_at(p->payload(((p != m_parameters[DCOMcontrol])?&should_update:&should_update_PDO1)), p->pdo_slot);
			    }
			    	
			    if(should_update) 
			    {
			    	try {
			    		socket.send(PDOMessage((PDOMessage::PDOFunctionCode) fn, m_node_id, payload));
					}
					catch (const std::runtime_error& e) {
						IF_VERBOSE(1, std::cerr << e.what();, m_verbose_level)
					}
			    	
			    }
			    else if(should_update_PDO1) 
			    {
			    	payload.resize(m_parameters[DCOMcontrol]->size);
			    	try {
			    		socket.send(PDOMessage((PDOMessage::PDOFunctionCode) PDO1Receive, m_node_id, payload));
					}
					catch (const std::runtime_error& e) {
						IF_VERBOSE(1, std::cerr << e.what();, m_verbose_level)
					}
			    	
			    }
		        
		    }
		}
	}
}



bool
CANopen::Driver::set_position(int32_t target, bool absolute)
{
	switch(get_mode(false))
	{
		case ProfilePosition: 	
			
			    	//std::cout << std::dec << target << " dd\n";
			if(m_parameters[PPp_target]->set(target+(absolute?m_offset_pos:0),true))
			{
				set_control((Control)(EnableOperation));
				if(absolute)
					set_control((Control)(EnableOperation|0x0030));
				else
					set_control((Control)(EnableOperation|0x0070));
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
			//this->set(PPv_target,target,true,true);
			return true;
		case ProfileVelocity:
			return m_parameters[PVv_target]->set(target,true);
		default:
			return false;
	}
};

bool
CANopen::Driver::set_torque(int16_t target)
{
	switch(get_mode())
	{
		case ProfileTorque:
			this->set(PTtq_target,target,true,true);
			return true;
		default:
			return false;
	}
};

void
CANopen::print_status(CANopen::Parameter* param) {
    
    g_verbose_mutex.lock();
	int16_t s = param->get<int16_t>();
        std::cout << "> Driver Status [0x" << std::hex << s << "] State: [ ";
        switch(s & Driver::State::mask) {
        case Driver::NotReadyToSwitchtON& Driver::State::mask:
            std::cout << "Not Ready to Switch ON ]\n";
            break;
        case Driver::SwitchONDisabled& Driver::State::mask:
            std::cout << "Switch ON Disabled ]\n";
            break;
        case Driver::ReadyToSwitchON& Driver::State::mask:
            std::cout << "Ready To Switch ON ]\n";
            break;
        case Driver::SwitchedON& Driver::State::mask:
            std::cout << "Switch ON ]\n";
            break;
        case Driver::OperationEnabled& Driver::State::mask:
            std::cout << "Operation Enabled ]\n";
            break;
        case Driver::Fault& Driver::State::mask:
            std::cout << "Fault ]\n";
            break;
        case Driver::QuickStop& Driver::State::mask:
            std::cout << "Quick Stop ]\n";
            break;
        default:
            std::cout << "Unknown ]\n";
            break;
        }
        std::string bits_names[16] = {"Ready To SwitchOn",
									"Switched On",
									"Operation Enabled",
									"Fault",
									"Voltage Enabled",
									"QuickStop Disabled",
									"Switch ON Disabled",
									"Error0",
									"Halt Request",
									"Remote",
									"Target Reached",
									"Internal Limit Reached",
									"Operation Mode",
									"Blocking Error",
									"Operation Mode Not Started",
									"Valid Zero Point"};
        
        for(int i =0; i<16; i++)
        {
        	std::cout << (((s&(0x0001<<i))==0)?"0":"1") << " [" << i << "] " << bits_names[i] << "\n";
        }
        std::cout << "\n\n";
	g_verbose_mutex.unlock();

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

void 
CANopen::Driver::start()
{
    this->set_control(DisableVoltage);
    this->set_control(Shutdown);
    this->set_control(EnableOperation);
    this->wait_state(OperationEnabled);
}

void 
CANopen::Driver::pause()
{
	this->set_control(DisableOperation);
	this->wait_state(SwitchedON);
}

void 
CANopen::Driver::stop()
{
	this->set_control(Shutdown);
	this->wait_state(ReadyToSwitchON);
}
    
void 
CANopen::Driver::profilePosition_mode()
{
    map_PDO(PDO2Transmit, m_parameters[_DCOMstatus], 0);
    map_PDO(PDO2Transmit, m_parameters[_p_act], m_parameters[_DCOMstatus]->size);
    map_PDO(PDO2Receive , m_parameters[DCOMcontrol], 0);
    map_PDO(PDO2Receive, m_parameters[PPp_target], m_parameters[DCOMcontrol]->size);
    this->set(RAMP_v_acc,1000,true,true);
	this->set(RAMP_v_dec,2000,true,true);
	this->set(PPv_target,2000,true,true);
    this->set_mode(ProfilePosition, true);
}

void 
CANopen::Driver::profileVelocity_mode()
{
    map_PDO(PDO3Transmit, m_parameters[_DCOMstatus], 0);
    map_PDO(PDO3Transmit, m_parameters[_v_act], m_parameters[_DCOMstatus]->size);
    map_PDO(PDO3Receive , m_parameters[DCOMcontrol], 0);
    map_PDO(PDO3Receive, m_parameters[PVv_target], m_parameters[DCOMcontrol]->size);
    
    this->set(RAMP_v_acc,1000,true,true);
	this->set(RAMP_v_dec,1000,true,true);
	this->set(PPv_target,1000,true,true);
	this->set_mode(ProfilePosition, true);
}

void 
CANopen::Driver::profileTorque_mode()
{
	this->set_mode(ProfileTorque, true);
}
 
void
CANopen::Driver::homing()
{
	this->set(HMv,(uint8_t)100,true,true);
	this->set(HMv_out,(uint8_t)10,true,true);
	this->set_mode(Driver::Homing);
	this->set(HMmethod,(uint8_t)1,true,true);
	this->set_control((Control)(EnableOperation|0x0010));

}

void
CANopen::Driver::set_mode(OperationMode mode, bool wait) {
    if(m_available) {
        m_parameters[DCOMopmode]->set(mode,true);
        send(m_parameters[DCOMopmode]);
        if(wait)
        	while(get_mode()!=mode);
    }
}

void
CANopen::Driver::set_control(Control ctrl) { 
    	m_parameters[DCOMcontrol]->set(ctrl, true);
    	while(!m_parameters[DCOMcontrol]->has_been_sent()); 
    	
    	IF_VERBOSE(1, std::cout << "Mode: " << ctrl_to_str(ctrl)  << "[" << std::hex << ctrl << "] Activated.\n";, m_verbose_level)
                    	}


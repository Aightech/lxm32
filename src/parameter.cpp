#include "parameter.h"

void
CANopen::Parameter::link_to_pdo(PDOFunctionCode fn, int8_t slot) {

    if(pdo_slot == -1) {
        pdo_fn = fn;
        pdo_slot = slot;
    }
    
};

bool
CANopen::Parameter::from_payload(Payload &p, int slot, bool received_data) {
    if(slot + size <= p.size()) { //ensure the payload contain the data.
        switch(size) {         //store the received value in the coresponding parameter (with the right size)
        case 1:
            return set(p.value<uint8_t>(slot),true,received_data);
        case 2:
            return set(p.value<uint16_t>(slot),true,received_data);
        case 4:
            return set(p.value<uint32_t>(slot),true,received_data);
        }
    }
    return false;
}

CANopen::Payload
CANopen::Parameter::payload(bool* should_be_sent) {
    const std::lock_guard<std::mutex> lock(mutex);
    if(should_be_sent!=nullptr)
    {
   	
    	*should_be_sent=m_should_be_sent;
//    	if(m_should_be_sent)
//    		std::cout << name  << "\n";
    	m_should_be_sent = false;
    }
    return std::vector<uint8_t>((uint8_t *)var, (uint8_t *)var + size);
};

void
CANopen::Parameter::callback()
{
    if(_cb!=nullptr)
    {
        cb_mutex.lock();
	_cb(this);
	cb_mutex.unlock();
    }
}

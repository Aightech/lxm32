#include "parameter.h"

void
CANopen::Parameter::link_to_pdo(PDOFunctionCode fn, int8_t slot) {
    if(pdo_slot == -1) {
        pdo_fn = fn;
        pdo_slot = slot;
    }
};

void
CANopen::Parameter::from_payload(Payload &p, bool from_pdo) {
    int i = (from_pdo) ? pdo_slot : 0;
    if(i + size <= p.size()) { //ensure the payload contain the data.
        switch(size) {         //store the received value in the coresponding parameter (with the right size)
        case 0:
            break;
        case 1:
            set(p.value<s8_t>(i));
            break;
        case 2:
            set(p.value<s16_t>(i));
            break;
        case 4:
            set(p.value<s32_t>(i));
            break;
        }
    }
}

CANopen::Payload
CANopen::Parameter::payload() {
    return std::vector<uint8_t>((uint8_t *)var, (uint8_t *)var + size);
};

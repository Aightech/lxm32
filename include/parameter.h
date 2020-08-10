#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#include "CANopen_socket.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace CANopen {
struct Parameter {

    enum PDOFunctionCode : uint32_t {
        PDO1Transmit = Message::PDO1Transmit,
        PDO1Receive = Message::PDO1Receive,
        PDO2Transmit = Message::PDO2Transmit,
        PDO2Receive = Message::PDO2Receive,
        PDO3Transmit = Message::PDO3Transmit,
        PDO3Receive = Message::PDO3Receive,
        PDO4Transmit = Message::PDO4Transmit,
        PDO4Receive = Message::PDO4Receive,
    };

    Parameter() { var = new int32_t; };

    template <typename T>
    Parameter(std::string name_, T val, uint16_t index_, uint8_t subindex_) : name(name_), index(index_), subindex(subindex_) {
        var = new T;
        *(T *)var = (T)val;
        is_updated.test_and_set();
        size = sizeof(T);
    };

    template <typename T>
    Parameter(std::string name_, T val, uint32_t index__sub) : Parameter(name_, val, (uint16_t)(index__sub >> 16), (uint8_t)index__sub){};

    ~Parameter() {
        delete(int32_t *)var;
    }

    void
    link_to_pdo(PDOFunctionCode fn, int8_t slot);

    template <typename T>
    bool
    set(T val, bool force_update=true) {
    	//std::cout << name << " " << std::hex << val << "\n";
	bool was_updated;
        if(sizeof(T) == size) {
            mutex.lock();
            was_updated=is_updated.test_and_set(); // get if the parameter is waiting to been sent (true:sent / false:not yet)
            if(force_update || was_updated)//if force update (even if the previous value was not sent yet) or value already update
            	*(T *)var = (T)val;
            is_updated.clear();//in any case set the flag to ensure the parameter will be sent.
            mutex.unlock();
            return (force_update || was_updated);
        }
        return false;
    }

    bool operator=(int8_t val){return this->set<int8_t>(val, false);}//return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    bool operator=(int16_t val){return this->set<int16_t>(val, false);}//return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    bool operator=(int32_t val){return this->set<int32_t>(val, false);}//return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    bool operator=(uint8_t val){return this->set<uint8_t>(val, false);}//return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    bool operator=(uint16_t val){return this->set<uint16_t>(val, false);}//return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    bool operator=(uint32_t val){return this->set<uint32_t>(val, false);}//return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    


    template <typename T>
    T
    get() {
        T val;
        if(sizeof(T) == size) {
            mutex.lock();
            val = *(T *)var;
            mutex.unlock();
        }
        return val;
    }
    
    operator int8_t(){ return this->get<int8_t>();}; 
    operator int16_t(){ return this->get<int16_t>();}; 
    operator int32_t(){ return this->get<int32_t>();}; 
    operator uint8_t(){ return this->get<uint8_t>();}; 
    operator uint16_t(){ return this->get<uint16_t>();}; 
    operator uint32_t(){ return this->get<uint32_t>();}; 

    void
    from_payload(Payload &p, bool from_pdo = false);

    Payload
    payload();

    void *var = nullptr;
    size_t size = 0;
    std::string name;
    uint16_t index = 0;
    uint8_t subindex = 0;

    PDOFunctionCode pdo_fn;
    int8_t pdo_slot = -1;

    std::atomic_flag is_updated;
    std::atomic_flag sdo_flag;

    private:
    std::mutex mutex;
};
} // namespace CANopen
#endif

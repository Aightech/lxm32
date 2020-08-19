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

    typedef void (*param_cb_t)(Parameter *); //type of the parameter callback function
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
    Parameter(std::string name_, T val, uint16_t index_, uint8_t subindex_, param_cb_t cb = nullptr) : name(name_), index(index_), subindex(subindex_), _cb(cb) {
        mutex.lock();
        var = new T;
        *(T *)var = (T)val;
        m_should_be_sent = false;
        mutex.unlock();
        size = sizeof(T);
    };

    template <typename T>
    Parameter(std::string name_, T val, uint32_t index__sub, param_cb_t cb = nullptr) : Parameter(name_, val, (uint16_t)(index__sub >> 16), (uint8_t)index__sub, cb){};

    ~Parameter() {
        delete(int32_t *)var;
    }

    void
    link_to_pdo(PDOFunctionCode fn, int8_t slot);

    template <typename T>
    bool
    set(T val, bool force_update = false, bool received_data = false) {
        bool was_updated = false;
        if(sizeof(T) == size) {
            mutex.lock();
            if(!m_should_be_sent || force_update) //if force update (even if the previous value was not sent yet) or value already update
            {
                if(*(T *)var != (T)val) {
                    was_updated = true;
                    *(T *)var = (T)val;
                }
                if(!received_data)
                    m_should_be_sent = true;
            }
            mutex.unlock();
            return was_updated;
        }
        return false;
    }

    bool
    operator=(int8_t val) { return this->set<int8_t>(val, false); } //return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    bool
    operator=(int16_t val) { return this->set<int16_t>(val, false); } //return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    bool
    operator=(int32_t val) { return this->set<int32_t>(val, false); } //return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    bool
    operator=(uint8_t val) { return this->set<uint8_t>(val, false); } //return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    bool
    operator=(uint16_t val) { return this->set<uint16_t>(val, false); } //return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    bool
    operator=(uint32_t val) { return this->set<uint32_t>(val, false); } //return fakse if the assignement didn't succeed (wrong type size, not updated yet)

    template <typename T>
    T
    get() {
      T val={};
        if(sizeof(T) == size) {
            mutex.lock();
            val = *(T *)var;
            mutex.unlock();
        }
        return val;
    }

    operator int8_t() { return this->get<int8_t>(); };
    operator int16_t() { return this->get<int16_t>(); };
    operator int32_t() { return this->get<int32_t>(); };
    operator uint8_t() { return this->get<uint8_t>(); };
    operator uint16_t() { return this->get<uint16_t>(); };
    operator uint32_t() { return this->get<uint32_t>(); };

    bool
    from_payload(Payload &p, int slot = 0, bool received_data = true);

    bool
    has_been_sent() {
        const std::lock_guard<std::mutex> lock(mutex);
        return !m_should_be_sent;
    }

    void
    callback();

    Payload
    payload(bool *should_be_sent = nullptr);

    size_t size = 0;
    std::string name;
    uint16_t index = 0;
    uint8_t subindex = 0;

    PDOFunctionCode pdo_fn;
    int8_t pdo_slot = -1;

    std::atomic_flag sdo_flag;

    private:
    void *var = nullptr;
    param_cb_t _cb = nullptr;
    bool m_should_be_sent;
    std::mutex mutex;
};
} // namespace CANopen
#endif

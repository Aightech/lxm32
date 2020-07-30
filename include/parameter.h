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
    using s8_t = uint8_t;
    using s16_t = uint16_t;
    using s32_t = uint32_t;

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

    Parameter() { var = new s32_t; };

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
        delete(s32_t *)var;
    }

    void
    link_to_pdo(PDOFunctionCode fn, int8_t slot);

    template <typename T>
    void
    set(T val) {

        if(sizeof(T) == size) {
            mutex.lock();
            *(T *)var = (T)val;
            mutex.unlock();
            is_updated.clear();
            std::cout << name << " hoy\n";
        }
    }

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

    private:
    std::mutex mutex;
};
} // namespace CANopen
#endif

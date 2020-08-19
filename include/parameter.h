#ifndef _PARAMETER_H_
#define _PARAMETER_H_

/*!
 * \file Parameter.h
 * \brief  Device Object from the object dictionary of a remote CANopen device.
 * \author Alexis Devillard
 * \version 1.0
 */

#include "CANopen_socket.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace CANopen {
/*!
 * \brief Object from the object dictionary of a remote CANopen device.
 */
struct Parameter {

    /*!
     * Parameter Callback function type
     */
    typedef void (*param_cb_t)(Parameter *); //type of the parameter callback function

    /*!
     * \brief PDO Function code
     */
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
    /*!
     * \brief Parameter Constructor
     * \param name_ Name of the parameter.
     * \param val Value to store in the parameter (the type will be used to fixed the data size)
     * \param index_ Index of the object in the object dictionary of the device.
     * \param subindex_ Subindex of the object in the object dictionary of the device.
     * \param cb [facultative] The address of a callback function that will be called each time the parameter is updated by the device.
     */
    Parameter(std::string name_, T val, uint16_t index_, uint8_t subindex_, param_cb_t cb = nullptr) : name(name_), index(index_), subindex(subindex_), _cb(cb) {
        mutex.lock();
        var = new T;
        *(T *)var = (T)val;
        m_should_be_sent = false;
        mutex.unlock();
        size = sizeof(T);
    };

    template <typename T>
    /*!
     * \brief Parameter Constructor
     * \param name_ Name of the parameter.
     * \param val Value to store in the parameter (the type will be used to fixed the data size)
     * \param index__sub Index and subindex of the object in the object dictionary of the device. (in the format index__sub)
     * \param cb [facultative] The address of a callback function that will be called each time the parameter is updated by the device.
     */
    Parameter(std::string name_, T val, uint32_t index__sub, param_cb_t cb = nullptr) : Parameter(name_, val, (uint16_t)(index__sub >> 16), (uint8_t)index__sub, cb){};

    ~Parameter() {
        delete(int32_t *)var;
    }

    /*!
     * \brief link_to_pdo Links a paramet to a PDO message
     * \param fn The function code of the PDO
     * \param slot The index of the parameter data inside the PDO Message Payload
     */
    void
    link_to_pdo(PDOFunctionCode fn, int8_t slot);

    template <typename T>
    /*!
     * \brief sets Set the value of a parameter.
     * \param val The value to set. (the type has to be coherent with the parameter data size)
     * \param force_update If set the value of the Parameter is set even idf the previous value has not been sent yet to the remote device.
     * \param received_data Has to be set to True if the data being set come from the remote device (so the sending flag will not be raised)
     * \return  True if the parameter value has been changed.
     */
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

    /*!
     * \brief Sets the value of the parameter.
     * \param val New int8_t value to set
     * \return  True if the parameter value has been changed.
     */
    bool
    operator=(int8_t val) { return this->set<int8_t>(val, false); } //return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    /*!
     * \brief Sets the value of the parameter.
     * \param val New int16_t value to set
     * \return  True if the parameter value has been changed.
     */
    bool
    operator=(int16_t val) { return this->set<int16_t>(val, false); } //return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    /*!
     * \brief Sets the value of the parameter.
     * \param val New int32_t value to set
     * \return  True if the parameter value has been changed.
     */
    bool
    operator=(int32_t val) { return this->set<int32_t>(val, false); } //return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    /*!
     * \brief Sets the value of the parameter.
     * \param val New uint8_t value to set
     * \return  True if the parameter value has been changed.
     */
    bool
    operator=(uint8_t val) { return this->set<uint8_t>(val, false); } //return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    /*!
     * \brief Sets the value of the parameter.
     * \param val New uint16_t value to set
     * \return  True if the parameter value has been changed.
     */
    bool
    operator=(uint16_t val) { return this->set<uint16_t>(val, false); } //return fakse if the assignement didn't succeed (wrong type size, not updated yet)
    /*!
     * \brief Sets the value of the parameter.
     * \param val New uint32_t value to set
     * \return True if the parameter value has been changed.
     */
    bool
    operator=(uint32_t val) { return this->set<uint32_t>(val, false); } //return fakse if the assignement didn't succeed (wrong type size, not updated yet)

    template <typename T>
    /*!
     * \brief gets Returns the value (with a type T coherent with the data size)
     * \return The value of the parameter.
     */
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

    /*!
     * \brief operator int8_t Returns the value as an int8_t
     */
    operator int8_t() { return this->get<int8_t>(); };
    /*!
     * \brief operator int16_t Returns the value as an int16_t
     */
    operator int16_t() { return this->get<int16_t>(); };
    /*!
     * \brief operator int32_t Returns the value as an int32_t
     */
    operator int32_t() { return this->get<int32_t>(); };
    /*!
     * \brief operator uint8_t Returns the value as an uint8_t
     */
    operator uint8_t() { return this->get<uint8_t>(); };
    /*!
     * \brief operator uint16_t Returns the value as an uint16_t
     */
    operator uint16_t() { return this->get<uint16_t>(); };
    /*!
     * \brief operator uint32_t Returns the value as an uint32_t
     */
    operator uint32_t() { return this->get<uint32_t>(); };

    /*!
     * \brief from_payload Sets the value of the parameters with the data of a payload.
     * \param p Payload to store in the parameter
     * \param slot Index in the payload array where the parameter data is
     * \param received_data If set the data is processed as a receied data (no sending flag raised)
     * \return True if the parameter value has been changed.
     */
    bool
    from_payload(Payload &p, int slot = 0, bool received_data = true);

    /*!
     * \brief has_been_sent Returns the sending flag.
     * \return The sending flag.
     */
    bool
    has_been_sent() {
        const std::lock_guard<std::mutex> lock(mutex);
        return !m_should_be_sent;
    }

    /*!
     * \brief callback Execute the parameter callback (If not null)
     */
    void
    callback();

    /*!
     * \brief payload Returns a payload filled with the parameter data.
     * \param should_be_sent If a boolean pointer is passed the sending flag is stored in the booled and then cleared.
     * \return A payload filled with the parameter data.
     */
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

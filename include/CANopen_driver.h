#ifndef _CANOPEN_DRIVER_H_
#define _CANOPEN_DRIVER_H_

#include "CANopen_socket.h"
//#include "LXM32A_CANopen_register.h"
#include <cstdarg>
#include <iostream>
#include <string>
#include <thread>

namespace CANopen {
class Driver {
    public:
    enum Register : uint32_t {
        StatusWord = 0x60410000,
        ControlWord = 0x60400000,
        OpMode = 0x60600000,
        PPp_target = 0x607A0000,
        PPv_target = 0x60810000,
        RAMP_v_acc = 0x60830000,
        RAMP_v_dec = 0x60840000
    };

    enum OperationMode : int8_t {
        ProfilePosition = 1,
        Velocity = 2,
        ProfileVelocity = 3,
        ProfileTorque = 4,
        Homing = 6,
        InterpolatedPosition = 7,
    };

    /*! Possible States */
    enum State : uint16_t {
        mask = 0x006f,                /*!< Keeping only main state bytes. */
        NotReadyToSwitchtON = 0x0000, /*!< Not Ready to Switch ON:
				       * - Low level Power (e.g. 15V, 5V) has been applied to the drive.
				       * - The drive is being initialized or is running self test.
				       * - A brake, if present, has to be applied in this state.
				       * -  The drive function is disabled. */

        SwitchONDisabled = 0x0040, /*!< Switch ON Disabled:
				      * - Drive Initialisation is complete.
				      * - The drive parameters have been set up.
				      * - Drive parameters may be changed.
				      * - High Voltage may not be applied to the drive, (e.g. for safety reasons).
				      * - The drive function is disabled. */

        ReadyToSwitchON = 0x0021, /*!< Ready to Switch ON:
				       * -  High Voltage may be applied to the drive.
				       * - The drive parameters may be changed.
				       * - The drive function is disabled */

        SwitchedON = 0x0023, /*!< Switched ON:
				      * - High Voltage has been applied to the drive. 
				      * - The Power Amplifier is ready.
				      * - The drive parameters may be changed.
				      * - The drive function is disabled. */

        OperationEnabled = 0x0027, /*!< Operation Enabled:
				      * - No faults have been detected.
				      * - The drive function is enabled and power is applied to the motor.
				      * - The drive parameters may be changed.(This corresponds to normal operation of the drive.) */

        Fault = 0x000f, /*!< Fault:
				      * - The drive parameters may be changed.
				      * - A fault has occured in the drive.
				      * - The drive function is disabled. */

        FaultReactionActive = 0x000f, /*!< Fault Reaction Active:
				      * - The drive parameters may be changed.
				      * - A non-fatal fault has occured in the drive.
				      * - The Quick Stop function is being executed.
				      * - The drive function is enabled and power is applied to the motor.*/

        QuickStopActive = 0x0007 /*!< Quick Stop Active:
				      * - The drive parameters may be changed.The Quick Stop function is being executed.
				      * - The drive function is enabled and power is applied to the motor.
				      * If the ‘Quick-Stop-Option-Code’ is switched to 5 (Stay in Quick-Stop), 
				      * you can’t leave the  Quick-Stop-State,  but  you  can  transmit  to  ‘Operation  Enable’
				      * with  the  command‘Enable Operation’*/
    };
    enum StatusBits : uint16_t {
        Voltage_disable = 0x0010,
        Quick_stop = 0x0020,
        Warning = 0x0080,
        Manufacterer = 0xC100,
        Remote = 0x0200,
        Target_reached = 0x0400,
        Internal_limit_reached = 0x0800,
        Operation_Mode = 0x3000
    };

    /*! Possible Control commands */
    enum Control : uint16_t {
        Shutdown = 0x0006,         /*!< goto ReadySwitchON */
        SwitchON = 0x0007,         /*!< goto SwitchedON */
        DisableVoltage = 0x0000,   /*!< goto SwitchONDisabled */
        QuickStop = 0x0002,        /*!< goto QuickStopActiv */
        DisableOperation = 0x0007, /*!< goto SwitchedON */
        EnableOperation = 0x000f,  /*!< goto OperationEnabled */
        FaultResest = 0x0080       /*!< goto SwitchONDisabled */
    };

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

    /*!
     *  \brief Constructor
     *  \param ifname : Name of the CAN interface.
     *  \param can_id : Node CAN ID of the driver.
     */
    Driver(const char *ifname, uint16_t can_id, bool verbose = false);

    void
    test() {
        std::cout << "hey base" << std::endl;
    };

    bool
    set_state(Control ctrl);

    State
    get_state();

    void
    activate_PDO(uint8_t node_id, PDOFunctionCode fn, bool set = true);

    void
    set_mode(OperationMode mode);

    template <typename T>
    void
    set(Register reg, T param) {
        if(m_available)
            m_socket.send(CANopen::SDOOutboundWrite(m_node_id, reg, param));
    }

    template <typename T, typename S = uint8_t>
    void
    pdo_watchdog(PDOFunctionCode pdo, T *p1, S *p2 = nullptr) {
        std::cout << m_ifname << std::endl;
        Socket socket(m_ifname, m_verbose);//TODO: add mask
        socket.bind();
	std::shared_ptr<Message> msg;
	Payload p;
        while(1) {
	  msg = socket.receive();
	  p = msg->payload();
	  *p1 = p.value<T>();
            if(p2 != nullptr)
                *p2 = p.value<T>(sizeof(T));
        }
    }

    void
    send_PDO(PDOFunctionCode pdo, Payload payload);

    void
    print_status();

    virtual void
    print_manufacturer_status() = 0;

    void
    print_control(Control control);

    /*!
     *  \brief return true if the can interface is available
     */
    bool
    is_available() {
        return m_available;
    };

    protected:
    const char *m_ifname;
    bool m_verbose;
    bool m_available;

    CANopen::Socket m_socket;

    bool activated_PDO[8] = {0};

    uint8_t m_node_id;
    uint16_t m_can_baud;
    uint16_t m_status;
    State m_state;
    OperationMode m_opMode;
};

} // namespace CANopen
#endif

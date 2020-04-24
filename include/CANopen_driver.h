#ifndef _CANOPEN_DRIVER_H_
#define _CANOPEN_DRIVER_H_

#include "CANopen_socket.h"

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

namespace CANopen {
class Driver {

    static constexpr int NB_PDO = 4;
    static constexpr int MAX_PDO_SLOT = 2;
    using s8_t = uint8_t;
    using s16_t = uint16_t;
    using s32_t = uint32_t;

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

    /*!
     *  \brief Enables to map the different parameters of the driver to the Transmit PDO. When a PDO is received in the T_PDO_socket() thread, the value of the pdo will be stored in the mapped parameter.
     *  \param pdo_n : Numero of the PDO.
     *  \param can_id : Position in the PDO payload.(eg.: pdo2: |status:slot0|current_pos:slot1|)
     *  \param param : address of the parameter to map
     */
    template <typename T>
    void
    map_PDO(int pdo_n, int slot, T *param) {
        m_T_PDO_mapping[pdo_n][slot] = param;
        m_T_PDO_mapping_t[pdo_n][slot] = sizeof(T);
    };

    /*!
     *  \brief Sends a SDO message to activate the specified PDO.
     *  \param ifname : N
     *  \param can_id : Node CAN ID of the driver.
     */
    void
    activate_PDO(PDOFunctionCode fn, bool set = true);

    /*!
     *  \brief Enables to store value in specified registers
     *  \param reg : The register to set in the format 0x|REGISTER:2bytes|00:1byte|SUB:1byte|.
     *  \param can_id : The value to store in the register.
     */
    void
    set(Register reg, Payload param);

    /*!
     *  \brief Sends a PDO message with a specified payload.
     *  \param pdo : The COB-ID of the PDO.
     *  \param payload : The payload to send.
     */
    void
    send_PDO(PDOFunctionCode pdo, Payload payload);

    /*!
     *  \brief Sends transition states order. 
     *  \param ctrl : Control to send.
     */
    bool
    set_state(Control ctrl);

    /*!
     *  \brief Returns the current state of the driver by reading the status word.
     */
    State
    get_state();

    /*!
     *  \brief Set the operationanl mode of the driver.
     *  \param mode : Mode to set.
     */
    void
    set_mode(OperationMode mode);

    void
    set_target(uint32_t target, bool byPDO = false, PDOFunctionCode pdo = PDO1Receive);

    virtual void
    set_position(int32_t target);
    virtual void
    set_velocity(int32_t target);
    virtual void
    set_torque(int32_t target);

    void
    homing();

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
    void
    T_PDO_socket();
    std::thread *m_pdo_socket_thread;
    std::mutex m_ro_mutex;

    const char *m_ifname;
    bool m_verbose;
    bool m_available;

    CANopen::Socket m_socket;

    void *m_T_PDO_mapping[NB_PDO][MAX_PDO_SLOT];
    size_t m_T_PDO_mapping_t[NB_PDO][MAX_PDO_SLOT];
    PDOFunctionCode m_T_PDO_mapping_rev[NB_PDO][MAX_PDO_SLOT];

    uint8_t m_node_id;
    uint16_t m_can_baud;

    //read only
    uint16_t m_statusWord;
    int32_t m_current_position;
    int32_t m_current_velocity;
    int32_t m_current_torque;

    State m_state;
    OperationMode m_opMode;
    Control m_controlWord;
};

} // namespace CANopen
#endif

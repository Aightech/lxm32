#ifndef _CANOPEN_DRIVER_H_
#define _CANOPEN_DRIVER_H_

/*!
 * \file CANopen_driver.h
 * \brief  Device Profile Drives and Motion Control.
 * \author Alexis Devillard
 * \version 1.0
 */

#include "CANopen_socket.h"
#include "parameter.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace CANopen {
void
print_status(Parameter *);
/*!
 * \brief Device Profile Drives and Motion Control.
 */
class Driver {

    static constexpr int NB_PDO = 4;
    static constexpr int MAX_PDO_SLOT = 2;

    public:
    /*! CIA 402 CANopen Driver */
    enum Register : uint32_t {
        _DCOMstatus = 0x60410000,   /*!< **6041<sub>h</sub>** - The statusword indicates the current state of the drive. No bits are latched. The statusword consist of bits for:
				     * - the current state of the drive,
				     * - the operating state of the mode and
				     * - manufacturer specific options. */
        DCOMcontrol = 0x60400000,   /*!< **6040<sub>h</sub>** - The controlword consist of bits for:
				     * - the controlling of the state,
				     * - the controlling of operating modes and
				     * - manufacturer specific options. */
        DCOMopmode = 0x60600000,    /*!< **6060<sub>h</sub>** - The parameter modes of operation switches the actually choosen operation mode.*/
        _DCOMopmd_act = 0x60610000, /*!< **6061<sub>h</sub>** - The modes of operation display shows the current mode of operation. The meaning of the returned
				      value corresponds to that of the modes of operation option code*/
        PPp_target = 0x607A0000,    /*!< **607A<sub>h</sub>** - The target position is the position that the drive should move to in position profile mode using the
				      current settings of motion control parameters such as velocity, acceleration, deceleration, motion
				      profile type etc. The target position is given in user defined position units. It is converted to position
				      increments using the position factor. The target position will be interpreted as
				      absolute or relative depending on the ‘abs / rel' flag in the controlword.*/
        PPv_target = 0x60810000,    /*!< **6081<sub>h</sub>** - The profile velocity is the velocity normally attained at the end of the acceleration ramp during a
				      profiled move and is valid for both directions of motion. The profile velocity is given in user defined
				      speed units. It is converted to position increments per second using the velocity encoder factor.*/
        PVv_target = 0x60FF0000,    /*!< **60FF<sub>h</sub>** - The target velocity is the input for the trajectory generator and the value is given in velocity units.*/
        PTtq_target = 0x60710000,   /*!< **6071<sub>h</sub>** - This parameter is the input value for the torque controller in profile torque mode and the value is given
				      per thousand of rated torque.*/
        RAMP_v_acc = 0x60830000,    /*!< **6083<sub>h</sub>** - The profile acceleration is given in user defined acceleration units. It is converted to position
				      increments per second 2 using the normalizing factors.*/
        RAMP_v_dec = 0x60840000,    /*!< **6084<sub>h</sub>** - The profile deceleration is given in the same units as profile acceleration. If this parameter is not
				      supported, then the profile acceleration value is also used for deceleration.*/
        _p_act = 0x60640000,        /*!< **6064<sub>h</sub>** - This object represents the actual value of the position measurement device in user defined units.*/
        _v_act = 0x606C0000,        /*!< **606C<sub>h</sub>** - The velocity actual value is also represented in velocity units and is coupled to the velocity used as
				      input to the velocity controller.*/
        _tq_act = 0x60770000,       /*!< **6077<sub>h</sub>** - The torque actual value corresponds to the instantaneous torque in the drive motor. The value is given
				      per thousand of rated torque.*/
        HMmethod = 0x60980000,      /*!< **6098<sub>h</sub>** - The homing method object determines the method that will be used during homing.*/
        HMv = 0x60990001,           /*!< **6099<sub>h</sub>01** - Speed during search for switch.*/
        HMv_out = 0x60990002        /*!< **6099<sub>h</sub>02** - Speed during search for zero*/
    };

    /*! Operational modes */
    enum OperationMode : int8_t {
        ProfilePosition = 1,      /*!<  The positioning of the drive is defined in this mode. Speed, position and acceleration can be
				  limited and profiled moves using a Trajectory Generator are possible as well.*/
        Velocity = 2,             /*!< Many frequency inverters use this simple mode to control the velocity of the drive with limits
				  and ramp functions. */
        ProfileVelocity = 3,      /*!<  The Profile Velocity Mode is used to control the velocity of the drive with no special regard of
				  the position. It supplies limit functions and Trajectory Generation.*/
        ProfileTorque = 4,        /*!<  The profile torque mode allows a host (external)
				  control system (i.e. closed-loop speed controller, open-loop transmission force controller) to transmit
				  the target torque value, which is processed via the trajectory generator. The torque slope and torque
				  profile type parameters are required.*/
        Homing = 6,               /*!<  Homming mode.*/
        InterpolatedPosition = 7, /*!<  The interpolated position mode is used to control multiple coordinated axles or a single axle with the
				  need for time-interpolation of set-point data. The interpolated position mode normally uses time
				  synchronization mechanisms like the sync object defined in /3/ for a time coordination of the related
				  drive units.*/
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

        OperationEnabled = 0x0037, /*!< Operation Enabled:
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
        ReadyToSwitchOn_bit = 0x0001,
        SwitchedOn_bit = 0x0002,
        OperationEnabled_bit = 0x0004,
        Fault_bit = 0x0008,
        VoltageEnabled_bit = 0x0010,
        QuickStop_bit = 0x0020,
        SwitchONDisabled_bit = 0x0040,
        Error0_bit = 0x0080,
        HaltRequest_bit = 0x0100,
        Remote_bit = 0x0200,
        TargetReached_bit = 0x0400,
        InternalLimitReached_bit = 0x0800,
        OperationMode_bit = 0x1000,
        BlockingError_bit = 0x2000,
        OperationModeStart_bit = 0x4000,
        ValidRef_bit = 0x8000
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
     *  \param verbose_lvl : Level of verbosity.
     */
    Driver(const char *ifname, uint16_t can_id, int verbose_lvl = 0);

    /*!
     *  \brief Enables to store a value in a specified registers.
     *  \param reg : The register to set. (In the format ind__sub)
     *  \param val : The value to store in the register.
     *  \param force_sdo : If true, the parameter will  be send to the driver via a SDO. Else the parametr will be sent via PDO if it was mapped to an activated RPDO.
     *  \param wait : If set, the function is blocking and wait for the parameter to be sent via SDO.
     */
    template <typename T>
    void
    set(Register reg, T val, bool force_sdo = false, bool wait = false) {

        if(m_available) {
            m_parameters[reg]->set(val);
            if(m_parameters[reg]->pdo_slot == -1 || force_sdo || wait)
                send(m_parameters[reg]);
            if(wait)
                while(m_parameters[reg]->sdo_flag.test_and_set())
                    ;
        }
    }

    /*!
     *  \brief get Gets the value of a specified registers.
     *  \param reg : The register to get. (In the format ind__sub)
     *  \param force_sdo : If true, the parameter will  be updated via a  reading SDO.
     *  \return The value of the register in the templated format T selected.
     */
    template <typename T>
    T
    get(Register reg, bool force_sdo = false) {

        if(force_sdo && m_available) {
            update(m_parameters[reg]);
            while(m_parameters[reg]->sdo_flag.test_and_set())
                ;
        }
        return m_parameters[reg]->get<T>();
    };

    /*!
     *  \brief set_control : Send transition states order.
     *  \param ctrl : Control to send.
     */
    void
    set_control(Control ctrl);

    /*!
     *  \brief Returns the current state of the driver by reading the status word.
     *  \return The current state.
     */
    State
    get_state() { return m_parameters[_DCOMstatus]->get<State>(); };

    /*!
     * \brief wait_state loop until the driver state is different from the one passed while(actual_state()&mask) != (state&mask))
     * \param state State to wait for.
     * \param _mask Mask to selected specific bits of the state.
     */
    void
    wait_state(State state, uint16_t _mask = mask) {
        this->sync();
        while((get_state() & mask) != (state & mask))
            ;
    }; //std::cout << (get_state()&mask) << " " << (state&mask)<< "\n";}

    /*!
     * \brief set_mode Set the desired opereration mode.
     * \param mode Mode to set.
     * \param wait Repeatidly test the actual operation mode register until it is eqaual to the selected mode.
     */
    void
    set_mode(OperationMode mode, bool wait = false);

    /*!
     * \brief get_mode Returns the actual operation mode of the driver.
     * \param force_sdo If set a sdo read message will be send to get the mode. Set to false to save some communication time.
     * \return The actual operational mode
     */
    OperationMode
    get_mode(bool force_sdo = true) { return this->get<OperationMode>(_DCOMopmd_act, force_sdo); };

    /*!
     * \brief set_position Send the new position to reach. Has to be in ProfilPositon mode to have some effect.
     * \param target Position to reach (in internal unit)
     * \param absolute If set the target will be process as an absolute value. Else it will be procecced as a relative (to the current position) value.
     * \return True if successfully sent.
     */
    bool
    set_position(int32_t target, bool absolute = true);
    /*!
     * \brief set_velocity Send the new velocity to reach. Has to be in ProfilPositon or ProfilVelocity mode to have some effect.
     * \param target Velocity to reach (in internal unit)
     * \return True if successfully sent.
     */
    bool
    set_velocity(int32_t target);
    /*!
     * \brief set_torque Send the new torque to reach. Has to be in ProfilTorque mode to have some effect.
     * \param target Torque to reach (in internal unit)
     * \return True if successfully sent.
     */
    bool
    set_torque(int16_t target);

    /*!
     * \brief get_position Returns the actual postion of the motor.
     * \return The actual postion of the motor.
     */
    int32_t
    get_position() { return m_parameters[_p_act]->get<int32_t>() - m_offset_pos; };
    /*!
     * \brief get_velocity Returns the actual velocity of the motor.
     * \return The actual velocity of the motor.
     */
    int32_t
    get_velocity() { return m_parameters[_v_act]->get<int32_t>(); };
    /*!
     * \brief get_torque Returns the actual torque of the motor.
     * \return The actual torque of the motor.
     */
    int32_t
    get_torque() { return m_parameters[_tq_act]->get<int32_t>(); };

    /*!
     * \brief set_position_offset Set the postion offset of the motor. Rq better to have a large offset to avoid letting the motor switch off in negativ position: it would result in a wrong position when restarting
     * \param offset_pos Offest of the motor in internal Unit
     */
    void
    set_position_offset(int32_t offset_pos) {
        m_offset_pos = offset_pos;
    };

    /*!
     * \brief start
     */
    void
    start();
    /*!
     * \brief pause
     */
    void
    pause();
    /*!
     * \brief stop
     */
    void
    stop();

    /*!
     * \brief profilePosition_mode
     */
    void
    profilePosition_mode(bool sync=false);
    /*!
     * \brief profileVelocity_mode
     */
    void
    profileVelocity_mode();
    /*!
     * \brief profileTorque_mode
     */
    void
    profileTorque_mode();
    /*!
     * \brief homing
     */
    void
    homing();

    Parameter *
    get_param(Register reg) { return m_parameters[reg]; };

    virtual void
    print_manufacturer_status() = 0;

    std::string
    ctrl_to_str(Control control);

    /*!
     *  \brief return true if the can interface is available
     */
    bool
    is_available() { return m_available; };

    /*!
     * \brief sync Send sync message.
     */
    void sync();
    void sync_PDO(PDOFunctionCode fn, bool sync);
protected:
    /*!
     *  \brief send the parameter via a Writting SDO message to the driver
     *  \param param Parameter to send.
     */
    void
    send(Parameter *param);

    /*!
     *  \brief Request an update of the parameter via a Reading SDO message. (the parameter has been updated when param->sdo_flag is down.
     *  \param param Parameter to update.
     */
    void
    update(Parameter *param);

    /*!
     *  \brief Enables to map the different parameters of the driver to the Transmit PDO. When a PDO is received in the T_PDO_socket() thread, the value of the pdo will be stored in the mapped parameter.
     *  \param fn : Function code of the PDO.
     *  \param param : Parameter to map.
     *  \param slot : Slot of the parameter in the PDO message.
     */
    void
    map_PDO(PDOFunctionCode fn, Parameter *param, int slot, bool sync=true);

    /*!
     *  \brief Sends a SDO message to activate the specified PDO.
     *  \param ifname : N
     *  \param can_id : Node CAN ID of the driver.
     */
    void
    activate_PDO(PDOFunctionCode fn, bool sync=true, bool set = true);

    void
    T_socket();

    void
    RPDO_socket();

    std::thread *m_rpdo_socket_thread;
    std::atomic_flag rpdo_socket_flag;
    std::mutex rpdo_mutex;
    std::thread *m_t_socket_thread;
    std::atomic_flag t_socket_flag;

    const char *m_ifname;
    int m_verbose_level;
    bool m_available;

    CANopen::Socket m_socket;

    std::map<PDOFunctionCode, std::vector<Parameter *>> m_PDO_map;
    std::map<Register, Parameter *> m_parameters;

    uint8_t m_node_id;
    uint16_t m_can_baud;
    int32_t m_offset_pos = 0;

    bool m_sync;
};

} // namespace CANopen
#endif

#ifndef _CANOPEN_DRIVER_H_
#define _CANOPEN_DRIVER_H_

#include "CANopen_socket.h"
#include "parameter.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace CANopen {
class Driver {

    static constexpr int NB_PDO = 4;
    static constexpr int MAX_PDO_SLOT = 2;

    public:
    enum Register : uint32_t {
        _DCOMstatus = 0x60410000,
        DCOMcontrol = 0x60400000,
        DCOMopmode = 0x60600000,
        PPp_target = 0x607A0000,
        PPv_target = 0x60810000,
        PVv_target = 0x60FF0000,
        PTtq_target = 0x60710000,
        RAMP_v_acc = 0x60830000,
        RAMP_v_dec = 0x60840000,
        _p_act = 0x60640000,
        _v_act = 0x606C0000,
        _tq_act = 0x60770000,
        HMmethod = 0x60980000
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
        Voltage_disable = 0x0010,
        Quick_stop = 0x0020,
        Warning = 0x0080,
        Manufacterer = 0xC100,
        Remote = 0x0200,
        Target_reached = 0x0400,
        Internal_limit_reached = 0x0800,
        Operation_Mode = 0x3000,
        Operation_mode_start = 0x4000
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
    send(Parameter *param);

    void
    update(Parameter *param);

    /*!
     *  \brief Enables to store value in specified registers
     *  \param reg : The register to set in the format 0x|REGISTER:2bytes|00:1byte|SUB:1byte|.
     *  \param can_id : The value to store in the register.
     */
    template <typename T>
    void
    set(Register reg, T val, bool force_sdo=false, bool wait=false) {

        if(m_available) {
            m_parameters[reg]->set(val);
	    if(m_parameters[reg]->pdo_slot==-1 || force_sdo)
	    {
	    	m_parameters[reg]->sdo_flag.test_and_set();
	      	send(m_parameters[reg]);
	    }
	    if(wait)
	    {
    		while(m_parameters[reg]->sdo_flag.test_and_set());
	    }
        }
    }
    
     /*!
     *  \brief Enables to store value in specified registers
     *  \param reg : The register to set in the format 0x|REGISTER:2bytes|00:1byte|SUB:1byte|.
     *  \param can_id : The value to store in the register.
     */
    template <typename T>
    T
    get(Register reg, bool force_sdo=false) {

	    if(force_sdo && m_available)
	    {
	    	m_parameters[reg]->sdo_flag.test_and_set();
	      	update(m_parameters[reg]);
	      	while(m_parameters[reg]->sdo_flag.test_and_set());
	    }
	    return m_parameters[reg]->get<T>();
    };

    /*!
     *  \brief Enables to map the different parameters of the driver to the Transmit PDO. When a PDO is received in the T_PDO_socket() thread, the value of the pdo will be stored in the mapped parameter.
     *  \param pdo_n : Numero of the PDO.
     *  \param can_id : Position in the PDO payload.(eg.: pdo2: |status:slot0|current_pos:slot1|)
     *  \param param : address of the parameter to map
     */
    void
    map_PDO(PDOFunctionCode fn, Parameter *param, int slot);

    /*!
     *  \brief Sends a SDO message to activate the specified PDO.
     *  \param ifname : N
     *  \param can_id : Node CAN ID of the driver.
     */
    void
    activate_PDO(PDOFunctionCode fn, bool set = true);

    /*!
     *  \brief Sends transition states order. 
     *  \param ctrl : Control to send.
     */
    void
    set_state(Control ctrl) { 
    //while(!m_parameters[DCOMcontrol]->is_updated.test());
    	std::cout << "Setting mode: " << ctrl_to_str(ctrl);
    	while(!m_parameters[DCOMcontrol]->set(ctrl, false)); 
    	std::cout << "\xdMode: " << ctrl_to_str(ctrl) << " Activated.\n";
                    	};

    /*!
     *  \brief Returns the current state of the driver by reading the status word.
     */
    State
    get_state() { return m_parameters[_DCOMstatus]->get<State>(); };

    /*!
     *  \brief Set the operationanl mode of the driver.
     *  \param mode : Mode to set.
     */
    void
    set_mode(OperationMode mode);

    OperationMode
    get_mode() { 
    return this->get<OperationMode>(DCOMopmode,true); 
    };

    bool
    set_position(int32_t target);
    bool
    set_velocity(int32_t target);
    bool
    set_torque(int32_t target);
    
    int32_t
    get_position(){return m_parameters[_p_act]->get<int32_t>();};
    int32_t
    get_velocity(){return m_parameters[_v_act]->get<int32_t>();};
    int32_t
    get_torque(){return m_parameters[_tq_act]->get<int32_t>();};
    
    void 
    wait_state(State state, uint16_t mask)
    {
    	while((get_state()&mask) != (state&mask))
    		 std::cout << std::hex << (get_state()&mask) << " " << (state&mask) << "\n";
    	std::cout << std::hex << (get_state()&mask) << " " << (state&mask) << "\n";
    }

    void
    homing();

    void
    print_status();
    
    Parameter* get_param(Register reg){return m_parameters[reg];};

    virtual void
    print_manufacturer_status() = 0;

    std::string
    ctrl_to_str(Control control);

    /*!
     *  \brief return true if the can interface is available
     */
    bool
    is_available() {
        return m_available;
    };

    protected:
    void
    T_socket();

  void
    RPDO_socket();
    std::thread *m_rpdo_socket_thread;
    std::atomic_flag rpdo_socket_flag;
    std::thread *m_t_socket_thread;
    std::atomic_flag t_socket_flag;

    const char *m_ifname;
    bool m_verbose;
    bool m_available;

    CANopen::Socket m_socket;

    std::map<PDOFunctionCode, std::vector<Parameter *>> m_PDO_map;
    std::map<Register, Parameter *> m_parameters;

    uint8_t m_node_id;
    uint16_t m_can_baud;
};

} // namespace CANopen
#endif

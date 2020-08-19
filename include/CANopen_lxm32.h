#ifndef _LEXIUM32A_CANOPEN_H_
#define _LEXIUM32A_CANOPEN_H_

/*!
 * \file CANopen_lxm32.h
 * \brief  Implementation of the Driver Class for a LXM32 driver.
 * \author Alexis Devillard
 * \version 1.0
 */

#include "CANopen_driver.h"

#include <cmath>
#include <iostream>
#include <string>
#include <unistd.h>

namespace CANopen {
/*!
 * \brief Implementation of the Driver Class for a LXM32 driver.
 */
class LXM32 : public Driver {
    public:
    /*!
     * \brief Constructor
     * \param ifname : Name of the CAN interface.
     * \param can_id : Node CAN ID of the driver.
     * \param verbose
     */
    LXM32(const char *ifname, uint16_t can_id, bool verbose = false);

    /*!
     * \brief set_angle Sets the motor at a desired angle.
     * \param ang Angle to reached.
     * \param absolute If set the angle will be interpreted as a absolute angle. Else it will be interpreted as a relative angle.
     * \param radian If set the angle will be interpreted in radian else degree.
     * \return True if the position was sent.
     */
    bool
    set_angle(double ang, bool absolute = true, bool radian = true);

    /*!
     * \brief get_angle Gets the motor at a desired angle.
     * \param radian If set the angle return in radian else degree.
     * \return The actual motor angle.
     */
    double
    get_angle(bool radian = true);

    void
    print_manufacturer_status(){};


    int nb_index_per_turn = 737280;
};
} // namespace CANopen

#endif

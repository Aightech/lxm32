#ifndef _LEXIUM32A_CANOPEN_H_
#define _LEXIUM32A_CANOPEN_H_

#include "CANopen_driver.h"

#include <cmath>
#include <iostream>
#include <string>
#include <unistd.h>

namespace CANopen {
class LXM32 : public Driver {
    public:
    /*!
     *  \brief Constructor
     *  \param ifname : Name of the CAN interface.
     *  \param can_id : Node CAN ID of the driver.
     */
    LXM32(const char *ifname, uint16_t can_id, bool verbose = false);

    bool
    set_angle(double ang, bool absolute = true, bool radian = true);

    double
    get_angle(bool radian = true);

    void
    print_manufacturer_status(){};


    int nb_index_per_turn = 737280;
};
} // namespace CANopen

#endif

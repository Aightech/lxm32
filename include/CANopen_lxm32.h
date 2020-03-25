#ifndef _LEXIUM32A_CANOPEN_H_
#define _LEXIUM32A_CANOPEN_H_

#include "CANopen_driver.h"
//#include "LXM32A_CANopen_register.h"
#include <string>
#include <unistd.h>

namespace CANopen {
class LXM32 : Driver {
    public:
    enum OPmode : uint8_t {
        ProfilePosition = 1,
        ProfileVelocity = 2

    };
    /*!
     *  \brief Constructor
     *  \param ifname : Name of the CAN interface.
     *  \param can_id : Node CAN ID of the driver.
     */
    LXM32(const char *ifname, uint16_t can_id, bool verbose = false);

    int32_t
    init();

    void
    get_param();

    void
    print_status();

    void
    print_control();
};
} // namespace CANopen

#endif

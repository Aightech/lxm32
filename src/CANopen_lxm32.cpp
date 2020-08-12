#include "CANopen_lxm32.h"

CANopen::LXM32::LXM32(const char *ifname, uint16_t can_id, bool verbose)
  : Driver(ifname,can_id,verbose) {

}

bool
CANopen::LXM32::set_angle(double ang,bool absolute, bool radian)
{
	uint32_t target = ang*nb_index_per_turn/((radian)?2*M_PI:360.0f);
	return this->set_position(target, absolute);
}

double
CANopen::LXM32::get_angle(bool radian)
{
	return get_position()*((radian)?2*M_PI:360.0f)/nb_index_per_turn;	
}

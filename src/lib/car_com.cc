#include "conductrix/car_com.h"

CarCom::CarCom(const std::string &port, int baud_rate)
: SerialBase(port, baud_rate)
{
    
}

CarCom::~CarCom()
{
}

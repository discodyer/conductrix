#ifndef CAR_H
#define CAR_H

#include "conductrix/foc_wheel.h"

class Car
{
private:
    FocControl wheel_l_;
    FocControl wheel_r_;


public:
    Car(const std::string &port, int baud_rate, int address_l, int address_r, size_t timeout_ms);
    ~Car();

    bool open();

    bool setVelocity(double velocity, uint8_t acceleration = 64);
    bool stop();

    bool setPosition(double distance, uint16_t velocity = 50, uint8_t acceleration = 64);

    bool setYawSpeed(double yaw_speed, uint8_t acceleration = 64);
    bool setYawPosition(double yaw, uint16_t velocity = 50, uint8_t acceleration = 64);
};

#endif // CAR_H

#include "conductrix/car.h"

Car::Car(const std::string &port, int baud_rate, int address_l, int address_r, size_t timeout_ms)
    : wheel_l_(port, baud_rate, address_l, timeout_ms),
      wheel_r_(port, baud_rate, address_r, timeout_ms)
{
}

Car::~Car()
{
    wheel_l_.close();
    // wheel_r_.close();
}

bool Car::open()
{
    return wheel_l_.open();
}

bool Car::setVelocity(double velocity, uint8_t acceleration)
{
    return (wheel_l_.setSpeed(velocity > 0 ? CCW : CW,
                              (uint16_t) static_cast<uint16_t>(std::abs(velocity * 100)),
                              static_cast<uint8_t>(acceleration))) &&
           (wheel_r_.setSpeed(velocity > 0 ? CW : CCW,
                              (uint16_t) static_cast<uint16_t>(std::abs(velocity * 100)),
                              static_cast<uint8_t>(acceleration)));
}
bool Car::stop()
{
    return wheel_l_.stop() &&
           wheel_r_.stop();
}

bool Car::setPosition(double distance, uint16_t velocity, uint8_t acceleration)
{
    uint32_t pulses = (uint32_t)(static_cast<uint16_t>(std::abs(distance)) * 15236);
    return (wheel_l_.setPosition(distance > 0 ? CCW : CW,
                                 velocity,
                                 acceleration,
                                 pulses)) &&
           (wheel_r_.setPosition(distance > 0 ? CW : CCW,
                                 velocity,
                                 acceleration,
                                 pulses));
}

bool Car::setYawSpeed(double yaw_speed, uint8_t acceleration)
{
    return (wheel_l_.setSpeed(yaw_speed > 0 ? CW : CCW,
                              (uint16_t) static_cast<uint16_t>(std::abs(yaw_speed * 100)),
                              acceleration)) &&
           (wheel_r_.setSpeed(yaw_speed > 0 ? CW : CCW,
                              (uint16_t) static_cast<uint16_t>(std::abs(yaw_speed * 100)),
                              acceleration));
}
bool Car::setYawPosition(double yaw, uint16_t velocity, uint8_t acceleration)
{
    uint32_t pulses = (uint32_t)(static_cast<uint16_t>(std::abs(yaw)) * 2583);
    return (wheel_l_.setPosition(yaw > 0 ? CW : CCW,
                                 velocity,
                                 acceleration,
                                 pulses)) &&
           (wheel_r_.setPosition(yaw > 0 ? CW : CCW,
                                 velocity,
                                 acceleration,
                                 pulses));
}
#include "conductrix/foc_wheel.h"
#include <ros/console.h>

FocControl::FocControl(const std::string &port, int baud_rate, int address, size_t timeout_ms_)
    : SerialBase(port, baud_rate), address_(address), parity_byte_(0x6b), timeout_ms_(200)
{
}

FocControl::~FocControl() {}

void FocControl::setCommandAddress(int address)
{
    address_ = address;
}
void FocControl::setTimeout(size_t timeout_ms)
{
    timeout_ms_ = timeout_ms;
}

void FocControl::sendData(const std::vector<uint8_t> &data)
{
    // 将 address、data 和校验和 (parity_byte_) 组合到一个新的 vector 中
    std::vector<uint8_t> frame;
    frame.push_back(address_);
    frame.insert(frame.end(), data.begin(), data.end());
    frame.push_back(parity_byte_);

    // 现在 frame 包含了 address、data 和校验和 parity_byte_，可以发送该数据了
    // 可以使用 SerialBase::write(complete_data) 来发送数据到串口设备
    write(frame);
}

bool FocControl::checkResponse(size_t timeout_ms)
{
    std::vector<uint8_t> buffer;
    if (!read(buffer, 3, timeout_ms))
    {
        ROS_ERROR("Serial read timeout or read error.");
        return false;
    }
    // 检查返回值是否符合预期
    if (buffer.size() != 3 || buffer[0] != address_ || buffer[2] != parity_byte_)
    {
        ROS_ERROR("Received unexpected response");
        return false;
    }
    // 判断是否为命令错误
    if (buffer[1] == 0xEE)
    {
        ROS_ERROR("Command error");
        return false;
    }
    // 检查是否为地址 + 02 + 校验字节
    if (buffer[1] = 0x02)
    {
        return true;
    }

    // 其他未知情况
    ROS_ERROR("Received unknown response");
    return false;
}

bool FocControl::encoderCalibration()
{
    std::vector<uint8_t> buffer;
    buffer.push_back(0x06);
    buffer.push_back(0x45);
    sendData(buffer);

    return checkResponse(timeout_ms_);
}

bool FocControl::setZero()
{
    std::vector<uint8_t> buffer;
    buffer.push_back(0x0a);
    buffer.push_back(0x6d);
    sendData(buffer);

    return checkResponse(timeout_ms_);
}

bool FocControl::unlockProtection()
{
    std::vector<uint8_t> buffer;
    buffer.push_back(0x0e);
    buffer.push_back(0x52);
    sendData(buffer);

    return checkResponse(timeout_ms_);
}

uint16_t FocControl::readEncoderValue()
{
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0x30);
    sendData(tx_buffer);

    std::vector<uint8_t> rx_buffer;
    if (!read(rx_buffer, 4, timeout_ms_))
    {
        ROS_ERROR("Serial read timeout or read error.");
        return 0;
    }
    // 检查返回值是否符合预期
    if (rx_buffer.size() != 4 || rx_buffer[0] != address_ || rx_buffer[3] != parity_byte_)
    {
        ROS_ERROR("Received unexpected response");
        return 0;
    }

    uint16_t value = (uint16_t)(((uint16_t)rx_buffer[1] << 8) | ((uint16_t)rx_buffer[2] << 0));
    return value;
}

int32_t FocControl::readPluses()
{
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0x33);
    sendData(tx_buffer);

    std::vector<uint8_t> rx_buffer;
    if (!read(rx_buffer, 6, timeout_ms_))
    {
        ROS_ERROR("Serial read timeout or read error.");
        return 0;
    }
    // 检查返回值是否符合预期
    if (rx_buffer.size() != 6 || rx_buffer[0] != address_ || rx_buffer[5] != parity_byte_)
    {
        ROS_ERROR("Received unexpected response");
        return 0;
    }

    int32_t value = (int32_t)(((int32_t)rx_buffer[1] << 24) |
                              ((int32_t)rx_buffer[2] << 16) |
                              ((int32_t)rx_buffer[3] << 8) |
                              ((int32_t)rx_buffer[4] << 0));
    return value;
}

int32_t FocControl::readPosition()
{
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0x36);
    sendData(tx_buffer);

    std::vector<uint8_t> rx_buffer;
    if (!read(rx_buffer, 6, timeout_ms_))
    {
        ROS_ERROR("Serial read timeout or read error.");
        return 0;
    }
    // 检查返回值是否符合预期
    if (rx_buffer.size() != 6 || rx_buffer[0] != address_ || rx_buffer[5] != parity_byte_)
    {
        ROS_ERROR("Received unexpected response");
        return 0;
    }

    int32_t value = (int32_t)(((int32_t)rx_buffer[1] << 24) |
                              ((int32_t)rx_buffer[2] << 16) |
                              ((int32_t)rx_buffer[3] << 8) |
                              ((int32_t)rx_buffer[4] << 0));
    return value;
}

double FocControl::readAngle()
{
    int32_t pos = readPosition();
    return (pos * 360) / 65536;
}

int16_t FocControl::readPositionError()
{
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0x39);
    sendData(tx_buffer);

    std::vector<uint8_t> rx_buffer;
    if (!read(rx_buffer, 4, timeout_ms_))
    {
        ROS_ERROR("Serial read timeout or read error.");
        return 0;
    }
    // 检查返回值是否符合预期
    if (rx_buffer.size() != 4 || rx_buffer[0] != address_ || rx_buffer[3] != parity_byte_)
    {
        ROS_ERROR("Received unexpected response");
        return 0;
    }

    uint16_t value = (uint16_t)(((uint16_t)rx_buffer[1] << 8) | ((uint16_t)rx_buffer[2] << 0));
    return value;
}

double FocControl::readAngleError()
{
    int32_t pos = readPosition();
    return (pos * 360) / 65536;
}

bool FocControl::readEnableStatus()
{
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0x3a);
    sendData(tx_buffer);

    std::vector<uint8_t> rx_buffer;
    if (!read(rx_buffer, 1, timeout_ms_))
    {
        ROS_ERROR("Serial read timeout or read error.");
        return 0;
    }

    if (rx_buffer.size() != 3 || rx_buffer[0] != address_ || rx_buffer[2] != parity_byte_)
    {
        ROS_ERROR("Received unexpected response");
        return 0;
    }

    if (rx_buffer[1] == 0x00)
    {
        return false;
    }
    else if (rx_buffer[1] == 0x01)
    {
        return true;
    }

    ROS_ERROR("Received unknown response");
    return false;
}

bool FocControl::readStallFlag()
{
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0x3e);
    sendData(tx_buffer);

    std::vector<uint8_t> rx_buffer;
    if (!read(rx_buffer, 1, timeout_ms_))
    {
        ROS_ERROR("Serial read timeout or read error.");
        return 0;
    }

    if (rx_buffer.size() != 3 || rx_buffer[0] != address_ || rx_buffer[2] != parity_byte_)
    {
        ROS_ERROR("Received unexpected response");
        return 0;
    }

    if (rx_buffer[1] == 0x00)
    {
        return false;
    }
    else if (rx_buffer[1] == 0x01)
    {
        return true;
    }

    ROS_ERROR("Received unknown response");
    return false;
}

bool FocControl::readAutoZeroFlag()
{
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0x3f);
    sendData(tx_buffer);

    std::vector<uint8_t> rx_buffer;
    if (!read(rx_buffer, 1, timeout_ms_))
    {
        ROS_ERROR("Serial read timeout or read error.");
        return 0;
    }

    if (rx_buffer.size() != 3 || rx_buffer[0] != address_ || rx_buffer[2] != parity_byte_)
    {
        ROS_ERROR("Received unexpected response");
        return 0;
    }

    if (rx_buffer[1] == 0x00)
    {
        return true;
    }
    else if (rx_buffer[1] == 0x01)
    {
        return false;
    }

    ROS_ERROR("Received unknown response");
    return false;
}

bool FocControl::setSubStep(uint8_t step)
{
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0x84);
    tx_buffer.push_back(step);
    sendData(tx_buffer);

    return checkResponse(timeout_ms_);
}

bool FocControl::setDeviceAddress(uint8_t address)
{
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0xae);
    tx_buffer.push_back(address);
    sendData(tx_buffer);

    return checkResponse(timeout_ms_);
}

bool FocControl::setEnableStatus(bool is_enable)
{
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0xf3);
    if (is_enable)
    {
        tx_buffer.push_back(0x01);
    }
    else
    {
        tx_buffer.push_back(0x00);
    }
    sendData(tx_buffer);

    return checkResponse(timeout_ms_);
}

bool FocControl::setSpeed(FocDirection dir, uint16_t velocity, uint8_t acceleration)
{
    uint16_t direction_speed = (static_cast<uint16_t>(dir) << 12) | (velocity & 0x0FFF);
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0xf6);
    tx_buffer.push_back(direction_speed >> 8);
    tx_buffer.push_back(direction_speed & 0xFF);
    tx_buffer.push_back(acceleration);
    sendData(tx_buffer);

    return checkResponse(timeout_ms_);
}

bool FocControl::stop()
{
    return setSpeed(CW, 0, 0xff);
}

bool FocControl::recordSpeed()
{
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0xff);
    tx_buffer.push_back(0xc8);
    sendData(tx_buffer);

    return checkResponse(timeout_ms_);
}
bool FocControl::clearSpeed()
{
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0xff);
    tx_buffer.push_back(0xca);
    sendData(tx_buffer);

    return checkResponse(timeout_ms_);
}

bool FocControl::setPosition(FocDirection dir, uint16_t velocity, uint8_t acceleration, uint32_t pulses)
{
    // 方向和速度（共用 2 个字节）
    uint16_t direction_speed = (static_cast<uint16_t>(dir) << 12) | (velocity & 0x0FFF);
    std::vector<uint8_t> tx_buffer;
    tx_buffer.push_back(0xfd);
    tx_buffer.push_back(direction_speed >> 8);   // 方向和速度高位
    tx_buffer.push_back(direction_speed & 0xFF); // 方向和速度低位
    tx_buffer.push_back(acceleration);           // 加速度档位
    tx_buffer.push_back((pulses >> 16) & 0xFF);  // 脉冲数高位
    tx_buffer.push_back((pulses >> 8) & 0xFF);   // 脉冲数中位
    tx_buffer.push_back(pulses & 0xFF);          // 脉冲数低位
    sendData(tx_buffer);

    return checkResponse(timeout_ms_);
}

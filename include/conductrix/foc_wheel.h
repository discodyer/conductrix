#ifndef FOC_WHEEL_H
#define FOC_WHEEL_H
#include "conductrix/serial_base.h"

enum FocDirection
{
    CW = 0x00,
    CCW = 0x01,
};
 
class FocControl : public SerialBase
{
public:
    FocControl(const std::string &port, int baud_rate, int address, size_t timeout_ms = 200);
    ~FocControl();

    // 设置要控制的电机地址
    void setCommandAddress(int address);
    void setTimeout(size_t timeout_ms);

    // ---- 触发动作命令 ----

    // 触发编码器校准
    bool encoderCalibration();
    // 设置当前位置为零点
    bool setZero();
    // 解除堵转保护
    bool unlockProtection();

    // ---- 读取参数命令 ----

    // 读取编码器值
    uint16_t readEncoderValue();
    // 读取输入脉冲数
    int32_t readPluses();
    // 读取电机实时位置
    int32_t readPosition();
    // 读取角度
    double readAngle();
    // 读取位置误差
    int16_t readPositionError();
    // 读取角度误差
    double readAngleError();
    // 读取使能状态
    bool readEnableStatus();
    // 读取堵转标志
    bool readStallFlag();
    // 读取单圈上电自动回零状态标志
    bool readAutoZeroFlag();

    // ---- 修改参数命令 ----

    // 修改当前细分步数
    bool setSubStep(uint8_t step);
    // 修改当前串口通讯地址
    bool setDeviceAddress(uint8_t address);

    // ---- 运动控制命令 ----

    // 控制闭环电机的使能状态
    bool setEnableStatus(bool is_enable);
    // 控制闭环电机的正反转，即速度模式控制
    bool setSpeed(FocDirection dir, uint16_t velocity, uint8_t acceleration);
    bool stop();
    // 存储/清除闭环电机正反转，即速度模式当前的参数，上电会自动运行
    bool recordSpeed();
    bool clearSpeed();
    // 位置模式控制
    bool setPosition(FocDirection dir, uint16_t velocity, uint8_t acceleration, uint32_t pulses);
    
private:
    uint8_t address_;
    uint8_t parity_byte_;
    size_t timeout_ms_;

    void sendData(const std::vector<uint8_t>& data);

    bool checkResponse(size_t timeout_ms);
};

#endif // FOC_WHEEL_H
#ifndef FOC_WHEEL_H
#define FOC_WHEEL_H
#include "conductrix/serial_base.h"

enum FocDirection
{
    CW,
    CCW,
};
 
class FocControl : public SerialBase
{
public:
    FocControl(const std::string &port, int baud_rate);
    ~FocControl();

    // ---- 触发动作命令 ----

    // 触发编码器校准
    void encoderCalibration();
    // 设置当前位置为零点
    void setZero();
    // 解除堵转保护
    void unlockProtection();

    // ---- 读取参数命令 ----

    // 读取编码器值
    uint16_t readEncoderValue();
    // 读取输入脉冲数
    int32_t readPluses();
    // 读取电机实时位置
    int32_t readPosition();
    // 读取位置误差
    int16_t readPositionError();
    // 读取使能状态
    uint8_t readEnableStatus();
    // 读取堵转标志
    uint8_t readStallFlag();
    // 读取单圈上电自动回零状态标志
    uint8_t readAutoZeroFlag();

    // ---- 修改参数命令 ----

    // 修改当前细分步数
    bool setSubStep(uint8_t step);
    // 修改当前串口通讯地址
    bool setSerialAddress(uint8_t address);

    // ---- 运动控制命令 ----

    // 控制闭环电机的使能状态
    bool setEnableStatus(bool is_enable);
    // 控制闭环电机的正反转，即速度模式控制
    bool setSpeed(FocDirection dir, uint16_t velocity, uint8_t acceleration);
    

private:
};

#endif // FOC_WHEEL_H
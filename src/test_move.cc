#include <ros/ros.h>
#include "conductrix/foc_wheel.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "foc_wheel_node");
    ros::NodeHandle nh;
    // 串口设备信息
    std::string serial_port_name = "/dev/ttyS3";
    int baud_rate = 115200;

    // 创建FocControl对象并打开串口
    // FocControl foc_control_l(serial_port_name, baud_rate, 1, 200);
    FocControl foc_control_r(serial_port_name, baud_rate, 2, 200);

    if (!foc_control_r.open())
    {
        ROS_ERROR("Failed to open serial port!");
        return 1;
    }

    // 设置顺时针方向，速度档位为500，加速度档位为255，运动3200个脉冲（即一圈）
    // foc_control_l.setPosition(CW, 500, 0xFF, 0x000C80);
    foc_control_r.setPosition(CW, 500, 0xFF, 0x000C80);
    // foc_control_l.encoderCalibration();
    // foc_control_r.encoderCalibration();

    // 等待一段时间，让轮子转动一圈
    ros::Duration(5.0).sleep();

    // 停止闭环电机旋转
    foc_control_r.stop();

    // 关闭串口
    foc_control_r.close();

    return 0;
}
#include <ros/ros.h>
#include "conductrix/foc_wheel.h"
#include "conductrix/car.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "foc_wheel_node");
    ros::NodeHandle nh;
    // 串口设备信息
    std::string serial_port_name = "/dev/ttyS3";
    int baud_rate = 115200;

    Car car(serial_port_name, baud_rate, 1, 2, 200);

    if (!car.open())
    {
        ROS_ERROR("Failed to open serial port!");
        return 1;
    }

    // foc_control_l.setSpeed(CW, 50, 0x88);
    // foc_control_r.setSpeed(CCW, 50, 0x88);
    // foc_control_l.setPosition(CW, 50, 64, 8115);
    // foc_control_r.setPosition(CW, 50, 64, 8115);

    // foc_control_l.setPosition(CCW, 50, 64, 12189);
    // foc_control_r.setPosition(CW, 50, 64, 12189);

    // car.setPosition(0.5);
    // ros::Duration(5.0).sleep();

    car.setYawPosition(3.1415 * 0.5);
    // ros::Duration(5.0).sleep();

    // car.setPosition(0.5);
    // ros::Duration(5.0).sleep();

    // car.setYawPosition(3.1415 * 0.5 * -1.0);
    // ros::Duration(5.0).sleep();

    // car.setPosition(1.0);
    // ros::Duration(5.0).sleep();

    // car.setYawPosition(3.1415 * 0.5 * -1.0);
    // ros::Duration(5.0).sleep();

    // car.setPosition(1.0);
    // ros::Duration(5.0).sleep();

    // car.setYawPosition(3.1415 * 0.5 * -1.0);
    // ros::Duration(5.0).sleep();

    // car.setPosition(1.0);
    // ros::Duration(5.0).sleep();

    // car.setYawPosition(3.1415 * 0.5 * -1.0);
    // ros::Duration(5.0).sleep();

    // car.setPosition(0.5);
    // ros::Duration(5.0).sleep();

    // car.setYawPosition(3.1415 * 0.5 * -1.0);
    // ros::Duration(5.0).sleep();

    // car.setPosition(-0.5);
    // ros::Duration(5.0).sleep();



    // car.setVelocity(-1.0);
    // car.setYawPosition(3.1415);
    // car.setYawSpeed(-1.0);

    // 10000 65.5cm
    // 10000 65.5cm
    // 30000 197cm
    // 50000 320cm
    // 50000 319.4cm


    // 50000 325cm
    // 50000 325cm
    // 50000 326cm
    // 50000 324.5cm

    // 20000 131
    // 20000 131.4
    // 20000 131.5
    // 20000 131.2
    // 20000 131.2

    // 0.006563

    // 155

    // 8115 = 360d
    // 12189 = 80cm


    // 等待一段时间，让轮子转动一圈
    ros::Duration(5.0).sleep();

    // 停止闭环电机旋转
    car.stop();

    return 0;
}
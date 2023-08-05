#include "conductrix/serial_base.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include "tf2_ros/transform_broadcaster.h"
#include <std_msgs/String.h>
#include <libserial/SerialStream.h>
#include "conductrix/ansi_color.h"

bool takeoff_flag = false;

void takeoffCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("takeoff !");
    if (msg->data.c_str() == "takeoff")
    {
        takeoff_flag = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_com");
    ros::NodeHandle nh;

    ros::Publisher drone_pose_pub_ = nh.advertise<geometry_msgs::Point>("drone/pose", 100);
    ros::Publisher drone_fire_pub_ = nh.advertise<geometry_msgs::Point>("drone/fire", 100);
    ros::Publisher drone_ready_pub_ = nh.advertise<std_msgs::String>("drone/ready", 100);
    ros::Subscriber drone_takeoff_sub_ = nh.subscribe("drone/takeoff", 100, takeoffCallback);

    std::string serial_port_name_ = "/dev/ttyS0";
    LibSerial::SerialStream serial_stream_;

    try
    {
        serial_stream_.Open(serial_port_name_);
        serial_stream_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        serial_stream_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        serial_stream_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_stream_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_stream_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        ROS_INFO(SUCCESS("Serial port %s opened!"), serial_port_name_.c_str());
    }
    catch (const LibSerial::OpenFailed &e)
    {
        ROS_ERROR("Serial port open failed : %s", e.what());
        return 0;
    }

    ros::Rate rate(20);
    while (ros::ok())
    {
        const int BUFFER_SIZE = 128;
        char input_buffer[BUFFER_SIZE];
        int input_len = 0;
        while (serial_stream_.IsDataAvailable())
        {
            serial_stream_ >> input_buffer[input_len++];
        }

        switch (input_buffer[1])
        {
        case 0x01: // 待机信号
            if (input_buffer[2] == 0x6b)
            {
                std_msgs::String msg_;
                msg_.data = std::string("ok");
                ROS_INFO("ok");
                drone_ready_pub_.publish(msg_);
            }
            break;
        case 0x02: // 点发布频道
            if (input_buffer[8] == 0x6b)
            {
                double pose_x = ((double)((input_buffer[3] << 8) | input_buffer[4]) / 1000) * (input_buffer[2] > 0 ? -1 : 1);
                double pose_y = ((double)((input_buffer[6] << 8) | input_buffer[7]) / 1000) * (input_buffer[5] > 0 ? -1 : 1);
                geometry_msgs::Point pose;
                pose.x = pose_x;
                pose.y = pose_y;
                pose.z = 0.0;
                drone_pose_pub_.publish(pose);
                ROS_INFO("Pose drone: x: %f, y: %f", pose_x, pose_y);
            }
            break;
        case 0x03: // 火源发布频道
            if (input_buffer[8] == 0x6b)
            {   
                double fire_x = ((double)((input_buffer[3] << 8) | input_buffer[4]) / 1000) * (input_buffer[2] > 0 ? -1 : 1);
                double fire_y = ((double)((input_buffer[6] << 8) | input_buffer[7]) / 1000) * (input_buffer[5] > 0 ? -1 : 1);
                geometry_msgs::Point fire;
                fire.x = fire_x;
                fire.y = fire_y;
                fire.z = 0.0;
                drone_fire_pub_.publish(fire);
                ROS_INFO("fire location: x: %f, y: %f", fire_x, fire_y);
            }
            break;

        default:
            break;
        }

        if(takeoff_flag)
        {
            serial_stream_ << 0xac << 0xab << 0x6b; // 起飞
            takeoff_flag = false;
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
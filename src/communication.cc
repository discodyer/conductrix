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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_com");
    ros::NodeHandle nh;

    ros::Publisher drone_pose_pub_ = nh.advertise<geometry_msgs::Point>("drone/pose", 10);
    ros::Publisher drone_ready_pub_ = nh.advertise<std_msgs::String>("drone/ready", 10);

    SerialBase drone_com("/dev/ttyS0", 115200);

    if (!drone_com.open())
    {
        ROS_ERROR("Failed to open serial port!");
        return 1;
    }

    ros::Rate rate(10);
    while (ros::ok())
    {
        std::vector<uint8_t> buffer;

        drone_com.read(buffer, 8, 200);
        if (buffer[0] != 0xac)
        {
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        switch (buffer[1])
        {
        case 0x01: // 待机信号
            if (buffer[2] == 0x6b)
            {
                std_msgs::String msg_;
                msg_.data = std::string("ok");
                drone_ready_pub_.publish(msg_);
            }
            break;
        case 0x02: // 点发布频道
            if (buffer[6] == 0x6b)
            {
                double pose_x = ((buffer[2] << 8) | buffer[3]) / 1000;
                double pose_y = ((buffer[4] << 8) | buffer[5]) / 1000;
                geometry_msgs::Point pose;
                pose.x = pose_x;
                pose.y = pose_y;
                pose.z = 0.0;
                drone_pose_pub_.publish(pose);
            }

            break;

        default:
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
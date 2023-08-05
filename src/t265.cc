#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "tf2_ros/transform_broadcaster.h"

#define TF2_EULER_DEFAULT_ZYX

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_node");
    ros::NodeHandle nh;

    // 创建静态坐标转换广播器
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
    static tf2_ros::TransformBroadcaster tf_broadcaster;

    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("car_pose", 10);
    ros::Publisher body_path_pubisher = nh.advertise<nav_msgs::Path>("body_frame/path", 1);


    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    geometry_msgs::TransformStamped transform_stamped;
    geometry_msgs::PoseStamped msg_body_pose;
    nav_msgs::Path body_path;
    std::string target_frame_id = "camera_link";              // t265 坐标系
    std::string source_frame_id = "camera_odom_frame";        // 里程计坐标系
    std::string offset_frame_id = "camera_odom_offset_frame"; // 修正后的里程计坐标系
    std::string world_frame_id = "world"; // 世界坐标系
    std::string body_frame_id = "base_link"; // 机体坐标系

    double output_rate = 10, roll_cam = 0.0, pitch_cam = 0.0, yaw_cam = 0.0;

    transform_stamped = tf_buffer.lookupTransform(target_frame_id, source_frame_id, ros::Time(0), ros::Duration(3.0));
    ros::Time last_tf_time = ros::Time::now();

    ros::Rate rate(output_rate);

    tf2::Vector3 position_offset;
    tf2::Quaternion quat_offset;

    // 获取一次偏移量作为全局偏移
    ROS_INFO("Wait %0.1f s to get first offset...", 2.0);
    ros::Duration(2.0).sleep();
    transform_stamped = tf_buffer.lookupTransform(target_frame_id, source_frame_id, ros::Time(0), ros::Duration(1.0));

    if (last_tf_time < transform_stamped.header.stamp)
    {
        last_tf_time = transform_stamped.header.stamp;

        tf2::Transform transform;
        tf2::fromMsg(transform_stamped.transform, transform);
        position_offset -= transform.getOrigin();
        tf2::Quaternion rotation_offset = transform.getRotation();
        tf2::Matrix3x3 mat(rotation_offset);
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);
        quat_offset.setEulerZYX(-yaw, -pitch, -roll);
        ROS_INFO("Got first offset:");
        ROS_INFO("position_offset_x: %f", position_offset.getX());
        ROS_INFO("position_offset_y: %f", position_offset.getY());
        ROS_INFO("position_offset_z: %f", position_offset.getZ());
        ROS_INFO("rotation_offset_yaw: %f", -yaw);
        ROS_INFO("rotation_offset_pitch: %f", -pitch);
        ROS_INFO("rotation_offset_roll: %f", -roll);
        transform_stamped.header.frame_id = source_frame_id;
        transform_stamped.child_frame_id = offset_frame_id;
        transform_stamped.transform.translation.x = position_offset.getX();
        transform_stamped.transform.translation.y = position_offset.getY();
        transform_stamped.transform.translation.z = position_offset.getZ();
        transform_stamped.transform.rotation.x = quat_offset.getX();
        transform_stamped.transform.rotation.y = quat_offset.getY();
        transform_stamped.transform.rotation.z = quat_offset.getZ();
        transform_stamped.transform.rotation.w = quat_offset.getW();
        static_tf_broadcaster.sendTransform(transform_stamped);
    }

    while (ros::ok())
    {
        ros::Time now = ros::Time(0);
        ros::Duration timeout(1.0);

        try
        {
            // 获取变换信息
            transform_stamped = tf_buffer.lookupTransform(target_frame_id, offset_frame_id, now, timeout);

            // Only publish pose messages when we have new transform data.
            if (last_tf_time < transform_stamped.header.stamp)
            {
                last_tf_time = transform_stamped.header.stamp;

                // 将ROS的geometry_msgs转换为tf2的数据类型
                tf2::Transform transform;
                tf2::fromMsg(transform_stamped.transform, transform);

                // 进行坐标系变换
                tf2::Vector3 position_body;
                tf2::Vector3 position_orig = transform.getOrigin();
                position_body.setX(cos(yaw_cam) * position_orig.getX() + sin(yaw_cam) * position_orig.getY());
                position_body.setY(-sin(yaw_cam) * position_orig.getX() + cos(yaw_cam) * position_orig.getY());
    
                // 读取旋转偏移并进行旋转
                tf2::Quaternion quat_cam = transform.getRotation();
                tf2::Quaternion quat_body;
                quat_body.setRPY(roll_cam, pitch_cam, yaw_cam);

                quat_body = quat_cam * quat_body;

                // 归一化
                quat_body.normalize();

                // Create PoseStamped message to be sent
                msg_body_pose.header.stamp = transform_stamped.header.stamp;
                msg_body_pose.header.frame_id = transform_stamped.header.frame_id;
                msg_body_pose.pose.position.x = position_body.getX();
                msg_body_pose.pose.position.y = position_body.getY();
                msg_body_pose.pose.position.z = position_body.getZ();
                msg_body_pose.pose.orientation.x = quat_body.getX();
                msg_body_pose.pose.orientation.y = quat_body.getY();
                msg_body_pose.pose.orientation.z = quat_body.getZ();
                msg_body_pose.pose.orientation.w = quat_body.getW();

                // Publish pose of body frame in world frame
                pose_publisher.publish(msg_body_pose);

                // Publish trajectory path for visualization
                body_path.header.stamp = msg_body_pose.header.stamp;
                body_path.header.frame_id = msg_body_pose.header.frame_id;
                body_path.poses.push_back(msg_body_pose);
                body_path_pubisher.publish(body_path);
                
                geometry_msgs::TransformStamped tfs;
                tfs.header.frame_id = world_frame_id;
                tfs.header.stamp = ros::Time::now();
                tfs.child_frame_id = body_frame_id;
                tfs.transform.translation.x = position_body.getX();
                tfs.transform.translation.y = position_body.getY();
                tfs.transform.translation.z = position_body.getZ();
                tfs.transform.rotation.x = quat_body.getX();
                tfs.transform.rotation.y = quat_body.getY();
                tfs.transform.rotation.z = quat_body.getZ();
                tfs.transform.rotation.w = quat_body.getW();
                tf_broadcaster.sendTransform(tfs);
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failed to lookup transform: %s", ex.what());
            ros::Duration(1.0).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
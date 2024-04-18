#pragma once

#include <map>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <rerun.hpp>

void log_imu(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg
);

void log_image(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::msg::Image::ConstSharedPtr& msg
);

void log_pose_stamped(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg
);

void log_odometry(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg
);

void log_camera_info(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg
);

void log_tf_message(
    const rerun::RecordingStream& rec,
    const std::map<std::string, std::string>& tf_frame_to_entity_path,
    const tf2_msgs::msg::TFMessage::ConstSharedPtr& msg
);

void log_transform(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const geometry_msgs::msg::TransformStamped& msg
);

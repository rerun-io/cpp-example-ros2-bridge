#pragma once

#include <map>
#include <memory>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <rerun.hpp>

using TopicOptions = std::map<std::string, YAML::Node>;

class RerunLoggerNode : public rclcpp::Node {
  public:
    RerunLoggerNode();
    void spin();

  private:
    rclcpp::CallbackGroup::SharedPtr _parallel_callback_group;
    std::map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> _topic_to_subscription;
    std::map<std::string, std::string> _tf_frame_to_entity_path;
    std::map<std::string, std::string> _tf_frame_to_parent;
    std::map<std::string, TopicOptions> _raw_topic_options;
    rclcpp::Time _last_tf_update_time;

    void _read_yaml_config(std::string yaml_path);

    TopicOptions _resolve_topic_options(const std::string& topic, const std::string& message_type)
        const;
    std::string _resolve_entity_path(const std::string& topic, const TopicOptions& topic_options)
        const;

    void _add_tf_tree(
        const YAML::Node& node, const std::string& parent_entity_path,
        const std::string& parent_frame
    );

    const rerun::RecordingStream _rec{"rerun_logger_node"};
    std::string _root_frame;
    float _tf_fixed_rate;
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener;

    rclcpp::TimerBase::SharedPtr _create_subscriptions_timer;
    rclcpp::TimerBase::SharedPtr _update_tf_timer;

    void _create_subscriptions();
    void _update_tf();

    /* Message specific subscriber factory functions */
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> _create_image_subscription(
        const std::string& topic, const TopicOptions& topic_options
    );
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> _create_imu_subscription(
        const std::string& topic, const TopicOptions& topic_options
    );
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>>
        _create_pose_stamped_subscription(
            const std::string& topic, const TopicOptions& topic_options
        );
    std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>> _create_tf_message_subscription(
        const std::string& topic, const TopicOptions& topic_options
    );
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> _create_odometry_subscription(
        const std::string& topic, const TopicOptions& topic_options
    );
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>>
        _create_camera_info_subscription(
            const std::string& topic, const TopicOptions& topic_options
        );
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>>
        _create_point_cloud2_subscription(
            const std::string& topic, const TopicOptions& topic_options
        );
};

#include "visualizer_node.hpp"
#include "rerun_bridge/rerun_ros_interface.hpp"

#include <algorithm>
#include <chrono>
#include <memory>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

ImageOptions image_options_from_topic_options(const TopicOptions& topic_options) {
    ImageOptions options;
    if (topic_options.find("min_depth") != topic_options.end()) {
        options.min_depth = topic_options.at("min_depth").as<float>();
    }
    if (topic_options.find("max_depth") != topic_options.end()) {
        options.max_depth = topic_options.at("max_depth").as<float>();
    }
    return options;
}

PointCloud2Options point_cloud2_options_from_topic_options(const TopicOptions& topic_options) {
    PointCloud2Options options;
    if (topic_options.find("colormap") != topic_options.end()) {
        options.colormap = topic_options.at("colormap").as<std::string>();
    }
    if (topic_options.find("colormap_field") != topic_options.end()) {
        options.colormap_field = topic_options.at("colormap_field").as<std::string>();
    }
    if (topic_options.find("colormap_min") != topic_options.end()) {
        options.colormap_min = topic_options.at("colormap_min").as<float>();
    }
    if (topic_options.find("colormap_max") != topic_options.end()) {
        options.colormap_max = topic_options.at("colormap_max").as<float>();
    }
    return options;
}

std::string parent_entity_path(const std::string& entity_path) {
    auto last_slash = entity_path.rfind('/');
    if (last_slash == std::string::npos) {
        return "";
    }
    return entity_path.substr(0, last_slash);
}

std::string resolve_ros_path(const std::string& path) {
    if (path.find("package://") == 0) {
        std::string package_name = path.substr(10, path.find('/', 10) - 10);
        std::string relative_path = path.substr(10 + package_name.size());
        try {
            std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
            return package_path + relative_path;
        } catch (ament_index_cpp::PackageNotFoundError& e) {
            throw std::runtime_error(
                "Could not resolve " + path +
                ". Replace with relative / absolute path, source the correct ROS environment, or install " +
                package_name + "."
            );
        }
    } else if (path.find("file://") == 0) {
        return path.substr(7);
    } else {
        return path;
    }
}

RerunLoggerNode::RerunLoggerNode() : Node("rerun_logger_node") {
    _rec.spawn().exit_on_failure();

    _parallel_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer);

    _last_tf_update_time = this->get_clock()->now();

    // Read additional config from yaml file
    // NOTE We're not using the ROS parameter server for this, because roscpp doesn't support
    //   reading nested data structures.
    std::string yaml_path;
    this->declare_parameter("yaml_path", "");
    if (this->get_parameter("yaml_path", yaml_path)) {
        RCLCPP_INFO(this->get_logger(), "Read yaml config at %s", yaml_path.c_str());
    }
    _read_yaml_config(yaml_path);

    // check for new topics every 0.1 seconds
    _create_subscriptions_timer = this->create_wall_timer(
        100ms,
        [&]() -> void { _create_subscriptions(); },
        _parallel_callback_group
    );

    if (_tf_fixed_rate != 0.0) {
        _update_tf_timer = this->create_wall_timer(
            std::chrono::duration<float>(1. / _tf_fixed_rate),
            [&]() { _update_tf(); },
            _parallel_callback_group
        );
    }
}

/// Convert a topic name to its entity path.
/// If the topic is explicitly mapped to an entity path, use that.
/// Otherwise, the topic name will be automatically converted to a flattened entity path like this:
///   "/one/two/three/four" -> "/topics/one-two-three/four"
std::string RerunLoggerNode::_resolve_entity_path(
    const std::string& topic, const TopicOptions& topic_options
) const {
    if (topic_options.find("entity_path") != topic_options.end()) {
        return topic_options.at("entity_path").as<std::string>();
    } else {
        std::string flattened_topic = topic;
        auto last_slash =
            (std::find(flattened_topic.rbegin(), flattened_topic.rend(), '/') + 1).base();

        if (last_slash != flattened_topic.begin()) {
            // keep leading slash and last slash
            std::replace(flattened_topic.begin() + 1, last_slash, '/', '-');
        }

        return "/topics" + flattened_topic;
    }
}

// Resolve topic options for a given topic.
// The options are merged from all matching keys overwriting previous values in the following order:
//  1. Options for partial topic names (such as /, /ns, etc.)
//  2. Options for message types (such as sensor_msgs::msg::Image)
//  3. Options for the exact topic name (such as /ns/topic)
// Aside from these rules, the options are merged in the order they are defined in the yaml file.
TopicOptions RerunLoggerNode::_resolve_topic_options(
    const std::string& topic, const std::string& message_type
) const {
    TopicOptions merged_options;
    // 1. partial topic names
    for (const auto& [prefix, options] : _raw_topic_options) {
        if (topic.find(prefix) == 0) {
            merged_options.insert(options.begin(), options.end());
        }
    }

    // 2. message types
    if (_raw_topic_options.find(message_type) != _raw_topic_options.end()) {
        auto options = _raw_topic_options.at(message_type);
        merged_options.insert(options.begin(), options.end());
    }

    // 3. exact topic name
    if (_raw_topic_options.find(topic) != _raw_topic_options.end()) {
        auto options = _raw_topic_options.at(topic);
        merged_options.insert(options.begin(), options.end());
    }

    return merged_options;
}

void RerunLoggerNode::_read_yaml_config(std::string yaml_path) {
    const YAML::Node config = YAML::LoadFile(yaml_path);

    // see https://www.rerun.io/docs/howto/ros2-nav-turtlebot#tf-to-rrtransform3d
    if (config["extra_transform3ds"]) {
        for (const auto& extra_transform3d : config["extra_transform3ds"]) {
            const std::array<float, 3> translation = {
                extra_transform3d["transform"][3].as<float>(),
                extra_transform3d["transform"][7].as<float>(),
                extra_transform3d["transform"][11].as<float>()};
            // Rerun uses column-major order for Mat3x3
            const std::array<float, 9> mat3x3 = {
                extra_transform3d["transform"][0].as<float>(),
                extra_transform3d["transform"][4].as<float>(),
                extra_transform3d["transform"][8].as<float>(),
                extra_transform3d["transform"][1].as<float>(),
                extra_transform3d["transform"][5].as<float>(),
                extra_transform3d["transform"][9].as<float>(),
                extra_transform3d["transform"][2].as<float>(),
                extra_transform3d["transform"][6].as<float>(),
                extra_transform3d["transform"][10].as<float>()};
            _rec.log_static(
                extra_transform3d["entity_path"].as<std::string>(),
                rerun::Transform3D(
                    rerun::Vec3D(translation),
                    rerun::Mat3x3(mat3x3),
                    extra_transform3d["from_parent"].as<bool>()
                )
            );
        }
    }
    if (config["extra_pinholes"]) {
        for (const auto& extra_pinhole : config["extra_pinholes"]) {
            // Rerun uses column-major order for Mat3x3
            const std::array<float, 9> image_from_camera = {
                extra_pinhole["image_from_camera"][0].as<float>(),
                extra_pinhole["image_from_camera"][3].as<float>(),
                extra_pinhole["image_from_camera"][6].as<float>(),
                extra_pinhole["image_from_camera"][1].as<float>(),
                extra_pinhole["image_from_camera"][4].as<float>(),
                extra_pinhole["image_from_camera"][7].as<float>(),
                extra_pinhole["image_from_camera"][2].as<float>(),
                extra_pinhole["image_from_camera"][5].as<float>(),
                extra_pinhole["image_from_camera"][8].as<float>(),
            };
            _rec.log_static(
                extra_pinhole["entity_path"].as<std::string>(),
                rerun::Pinhole(image_from_camera)
                    .with_resolution(
                        extra_pinhole["width"].as<int>(),
                        extra_pinhole["height"].as<int>()
                    )
            );
        }
    }
    if (config["tf"]) {
        if (config["tf"]["update_rate"]) {
            _tf_fixed_rate = config["tf"]["update_rate"].as<float>();
        }

        if (config["tf"]["tree"] && config["tf"]["tree"].size()) {
            // set root frame, all messages with frame_id will be logged relative to this frame
            _root_frame = config["tf"]["tree"].begin()->first.as<std::string>();

            // recurse through the tree and add all transforms
            _add_tf_tree(config["tf"]["tree"], "", "");
        }
    }

    if (config["topic_options"]) {
        _raw_topic_options = config["topic_options"].as<std::map<std::string, TopicOptions>>();
    }

    if (config["urdf"]) {
        std::string urdf_entity_path;
        if (config["urdf"]["entity_path"]) {
            urdf_entity_path = config["urdf"]["entity_path"].as<std::string>();
        }
        if (config["urdf"]["file_path"]) {
            std::string urdf_file_path =
                resolve_ros_path(config["urdf"]["file_path"].as<std::string>());
            if (urdf_file_path.size()) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Logging URDF from file path %s",
                    urdf_file_path.c_str()
                );
                _rec.log_file_from_path(urdf_file_path, urdf_entity_path, true);
            }
        }
    }
}

void RerunLoggerNode::_add_tf_tree(
    const YAML::Node& topic_options, const std::string& parent_entity_path,
    const ::std::string& parent_frame
) {
    for (const auto& child : topic_options) {
        auto frame = child.first.as<std::string>();
        auto value = child.second;
        const std::string entity_path = parent_entity_path + "/" + frame;
        _tf_frame_to_entity_path[frame] = entity_path;
        _tf_frame_to_parent[frame] = parent_frame;
        RCLCPP_INFO(
            this->get_logger(),
            "Mapping tf frame %s to entity path %s",
            frame.c_str(),
            entity_path.c_str()
        );
        if (value.size() >= 1) {
            _add_tf_tree(value, entity_path, frame);
        }
    }
}

void RerunLoggerNode::_create_subscriptions() {
    for (const auto& [topic_name, topic_types] : this->get_topic_names_and_types()) {
        // already subscribed to this topic?
        if (_topic_to_subscription.find(topic_name) != _topic_to_subscription.end()) {
            continue;
        }

        if (topic_types.size() != 1) {
            RCLCPP_WARN(
                this->get_logger(),
                "Skipping topic %s with multiple types",
                topic_name.c_str()
            );
            continue;
        }

        const auto topic_type = topic_types[0];
        const auto message_type = topic_types[0];
        const auto topic_options = _resolve_topic_options(topic_name, message_type);

        if (topic_type == "sensor_msgs/msg/Image") {
            _topic_to_subscription[topic_name] =
                _create_image_subscription(topic_name, topic_options);
        } else if (topic_type == "sensor_msgs/msg/Imu") {
            _topic_to_subscription[topic_name] =
                _create_imu_subscription(topic_name, topic_options);
        } else if (topic_type == "geometry_msgs/msg/PoseStamped") {
            _topic_to_subscription[topic_name] =
                _create_pose_stamped_subscription(topic_name, topic_options);
        } else if (topic_type == "tf2_msgs/msg/TFMessage") {
            _topic_to_subscription[topic_name] =
                _create_tf_message_subscription(topic_name, topic_options);
        } else if (topic_type == "nav_msgs/msg/Odometry") {
            _topic_to_subscription[topic_name] =
                _create_odometry_subscription(topic_name, topic_options);
        } else if (topic_type == "sensor_msgs/msg/CameraInfo") {
            _topic_to_subscription[topic_name] =
                _create_camera_info_subscription(topic_name, topic_options);
        } else if (topic_type == "sensor_msgs/msg/PointCloud2") {
            _topic_to_subscription[topic_name] =
                _create_point_cloud2_subscription(topic_name, topic_options);
        } else if (topic_type == "sensor_msgs/msg/JointState") {
            _topic_to_subscription[topic_name] =
                _create_joint_state_subscription(topic_name, topic_options);
        }
    }
}

void RerunLoggerNode::_update_tf() {
    // NOTE We log the interpolated transforms with an offset assuming the whole tree has
    //  been updated after this offset. This is not an ideal solution. If a frame is updated
    //  with a delay longer than this offset we will never log interpolated transforms for it.
    //  It might be possible to always log the interpolated transforms on a per frame basis whenever
    //  a new message is received for that frame. This would require maintaining the latest
    //  transform for each frame. However, this would not work if transforms for a frame arrive
    //  out of order (maybe this is not a problem in practice?).

    auto now = this->get_clock()->now();

    // If no time has passed since the last update, don't do anything
    // This can happen when using simulation time that does not keep going at the end
    if (now - _last_tf_update_time == 0s) {
        return;
    }

    for (const auto& [frame, entity_path] : _tf_frame_to_entity_path) {
        auto parent = _tf_frame_to_parent.find(frame);
        if (parent == _tf_frame_to_parent.end() or parent->second.empty()) {
            continue;
        }
        try {
            auto transform = std::make_shared<geometry_msgs::msg::TransformStamped>(
                _tf_buffer->lookupTransform(parent->second, frame, now - rclcpp::Duration(1, 0))
            );
            log_transform(_rec, entity_path, transform);
            _last_tf_update_time = now;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Could not lookup transform: %s",
                ex.what()
            );
        }
    }
}

std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>>
    RerunLoggerNode::_create_image_subscription(
        const std::string& topic, const TopicOptions& topic_options
    ) {
    std::string entity_path = _resolve_entity_path(topic, topic_options);
    bool lookup_transform = (topic_options.find("entity_path") == topic_options.end());
    bool restamp = false;
    if (topic_options.find("restamp") != topic_options.end()) {
        restamp = topic_options.at("restamp").as<bool>();
    }

    auto image_options = image_options_from_topic_options(topic_options);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    RCLCPP_INFO(
        this->get_logger(),
        "Subscribing to Image topic %s, logging to entity path %s",
        topic.c_str(),
        entity_path.c_str()
    );

    return this->create_subscription<sensor_msgs::msg::Image>(
        topic,
        1000,
        [&, entity_path, lookup_transform, restamp, image_options](
            const sensor_msgs::msg::Image::SharedPtr msg
        ) {
            _handle_msg_header(
                restamp,
                lookup_transform,
                parent_entity_path(entity_path),
                msg->header
            );
            log_image(_rec, entity_path, msg, image_options);
        },
        subscription_options
    );
}

std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>>
    RerunLoggerNode::_create_imu_subscription(
        const std::string& topic, const TopicOptions& topic_options
    ) {
    std::string entity_path = _resolve_entity_path(topic, topic_options);
    bool restamp = false;
    if (topic_options.find("restamp") != topic_options.end()) {
        restamp = topic_options.at("restamp").as<bool>();
    }

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    return this->create_subscription<sensor_msgs::msg::Imu>(
        topic,
        1000,
        [&, entity_path, restamp](const sensor_msgs::msg::Imu::SharedPtr msg) {
            _handle_msg_header(restamp, false, entity_path, msg->header);
            log_imu(_rec, entity_path, msg);
        },
        subscription_options
    );
}

std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>>
    RerunLoggerNode::_create_pose_stamped_subscription(
        const std::string& topic, const TopicOptions& topic_options
    ) {
    std::string entity_path = _resolve_entity_path(topic, topic_options);
    bool lookup_transform = (topic_options.find("entity_path") == topic_options.end());
    bool restamp = false;
    if (topic_options.find("restamp") != topic_options.end()) {
        restamp = topic_options.at("restamp").as<bool>();
    }

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    return this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic,
        1000,
        [&, entity_path, lookup_transform, restamp](
            const geometry_msgs::msg::PoseStamped::SharedPtr msg
        ) {
            _handle_msg_header(restamp, lookup_transform, entity_path, msg->header);
            log_pose_stamped(_rec, entity_path, msg);
        },
        subscription_options
    );
}

std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>>
    RerunLoggerNode::_create_tf_message_subscription(
        const std::string& topic, const TopicOptions& topic_options
    ) {
    std::string entity_path = _resolve_entity_path(topic, topic_options);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    return this->create_subscription<tf2_msgs::msg::TFMessage>(
        topic,
        1000,
        [&, entity_path](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
            log_tf_message(_rec, _tf_frame_to_entity_path, msg);
        },
        subscription_options
    );
}

std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>>
    RerunLoggerNode::_create_odometry_subscription(
        const std::string& topic, const TopicOptions& topic_options
    ) {
    std::string entity_path = _resolve_entity_path(topic, topic_options);
    bool lookup_transform = (topic_options.find("entity_path") == topic_options.end());
    bool restamp = false;
    if (topic_options.find("restamp") != topic_options.end()) {
        restamp = topic_options.at("restamp").as<bool>();
    }

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    return this->create_subscription<nav_msgs::msg::Odometry>(
        topic,
        1000,
        [&, entity_path, lookup_transform, restamp](const nav_msgs::msg::Odometry::SharedPtr msg) {
            _handle_msg_header(restamp, lookup_transform, entity_path, msg->header);
            log_odometry(_rec, entity_path, msg);
        },
        subscription_options
    );
}

std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>>
    RerunLoggerNode::_create_camera_info_subscription(
        const std::string& topic, const TopicOptions& topic_options
    ) {
    std::string entity_path = _resolve_entity_path(topic, topic_options);
    bool restamp = false;
    if (topic_options.find("restamp") != topic_options.end()) {
        restamp = topic_options.at("restamp").as<bool>();
    }

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    // If the camera_info topic has not been explicility mapped to an entity path,
    // we assume that the camera_info topic is a sibling of the image topic, and
    // hence use the parent as the entity path for the pinhole model.
    if (topic_options.find("entity_path") == topic_options.end()) {
        entity_path = parent_entity_path(entity_path);
    }

    return this->create_subscription<sensor_msgs::msg::CameraInfo>(
        topic,
        1,
        [&, entity_path, restamp](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            _handle_msg_header(restamp, false, entity_path, msg->header);
            log_camera_info(_rec, entity_path, msg);
        },
        subscription_options
    );
}

std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>>
    RerunLoggerNode::_create_point_cloud2_subscription(
        const std::string& topic, const TopicOptions& topic_options
    ) {
    std::string entity_path = _resolve_entity_path(topic, topic_options);
    bool lookup_transform = (topic_options.find("entity_path") == topic_options.end());
    bool restamp = false;
    if (topic_options.find("restamp") != topic_options.end()) {
        restamp = topic_options.at("restamp").as<bool>();
    }
    auto point_cloud2_options = point_cloud2_options_from_topic_options(topic_options);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    RCLCPP_INFO(
        this->get_logger(),
        "Subscribing to PointCloud2 topic %s, logging to entity path %s",
        topic.c_str(),
        entity_path.c_str()
    );

    return this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic,
        1000,
        [&, entity_path, lookup_transform, restamp, point_cloud2_options](
            const sensor_msgs::msg::PointCloud2::SharedPtr msg
        ) {
            _handle_msg_header(restamp, lookup_transform, entity_path, msg->header);
            log_point_cloud2(_rec, entity_path, msg, point_cloud2_options);
        },
        subscription_options
    );
}

std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>>
    RerunLoggerNode::_create_joint_state_subscription(
        const std::string& topic, const TopicOptions& topic_options
    ) {
    std::string entity_path = _resolve_entity_path(topic, topic_options);
    bool lookup_transform = false; // Joint states don't need coordinate transformations
    bool restamp = false;
    if (topic_options.find("restamp") != topic_options.end()) {
        restamp = topic_options.at("restamp").as<bool>();
    }

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    RCLCPP_INFO(
        this->get_logger(),
        "Subscribing to JointState topic %s, logging to entity path %s",
        topic.c_str(),
        entity_path.c_str()
    );

    return this->create_subscription<sensor_msgs::msg::JointState>(
        topic,
        1000,
        [&, entity_path, lookup_transform, restamp](
            const sensor_msgs::msg::JointState::SharedPtr msg
        ) {
            _handle_msg_header(restamp, lookup_transform, entity_path, msg->header);
            log_joint_state(_rec, entity_path, msg);
        },
        subscription_options
    );
}

void RerunLoggerNode::_handle_msg_header(
    bool restamp, bool lookup_transform, const std::string& entity_path,
    std_msgs::msg::Header& header
) {
    if (restamp) {
        auto now = this->get_clock()->now();
        header.stamp = now;
    }
    if (!_root_frame.empty() && lookup_transform) {
        try {
            auto transform = std::make_shared<geometry_msgs::msg::TransformStamped>(
                _tf_buffer->lookupTransform(_root_frame, header.frame_id, header.stamp, 100ms)
            );
            log_transform(_rec, entity_path, transform);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    // Executor does not take ownership of the topic_options, so we have to maintain a shared_ptr
    auto rerun_logger_node = std::make_shared<RerunLoggerNode>();
    executor.add_node(rerun_logger_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}

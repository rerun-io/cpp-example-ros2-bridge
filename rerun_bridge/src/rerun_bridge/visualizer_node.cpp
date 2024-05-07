#include "visualizer_node.hpp"
#include "rerun_bridge/rerun_ros_interface.hpp"

#include <algorithm>
#include <chrono>
#include <memory>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

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

    _parallel_callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer);

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
std::string RerunLoggerNode::_resolve_entity_path(const std::string& topic) const {
    if (_topic_to_entity_path.find(topic) != _topic_to_entity_path.end()) {
        return _topic_to_entity_path.at(topic);
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

void RerunLoggerNode::_read_yaml_config(std::string yaml_path) {
    const YAML::Node config = YAML::LoadFile(yaml_path);

    // see https://www.rerun.io/docs/howto/ros2-nav-turtlebot#tf-to-rrtransform3d
    if (config["topic_to_entity_path"]) {
        _topic_to_entity_path =
            config["topic_to_entity_path"].as<std::map<std::string, std::string>>();

        for (auto const& [key, val] : _topic_to_entity_path) {
            RCLCPP_INFO(
                this->get_logger(),
                "Mapping topic %s to entity path %s",
                key.c_str(),
                val.c_str()
            );
        }
    }
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
            _rec.log_timeless(
                extra_transform3d["entity_path"].as<std::string>(),
                rerun::Transform3D(
                    rerun::Vec3D(translation),
                    rerun::Mat3x3(mat3x3),
                    extra_transform3d["from_parent"].as<bool>()
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
        _topic_options = config["topic_options"].as<std::map<std::string, YAML::Node>>();
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
    const YAML::Node& node, const std::string& parent_entity_path, const ::std::string& parent_frame
) {
    for (const auto& child : node) {
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

        if (topic_type == "sensor_msgs/msg/Image") {
            _topic_to_subscription[topic_name] = _create_image_subscription(topic_name);
        } else if (topic_type == "sensor_msgs/msg/Imu") {
            _topic_to_subscription[topic_name] = _create_imu_subscription(topic_name);
        } else if (topic_type == "geometry_msgs/msg/PoseStamped") {
            _topic_to_subscription[topic_name] = _create_pose_stamped_subscription(topic_name);
        } else if (topic_type == "tf2_msgs/msg/TFMessage") {
            _topic_to_subscription[topic_name] = _create_tf_message_subscription(topic_name);
        } else if (topic_type == "nav_msgs/msg/Odometry") {
            _topic_to_subscription[topic_name] = _create_odometry_subscription(topic_name);
        } else if (topic_type == "sensor_msgs/msg/CameraInfo") {
            _topic_to_subscription[topic_name] = _create_camera_info_subscription(topic_name);
        }
    }
}

void RerunLoggerNode::_update_tf() {
    RCLCPP_INFO(this->get_logger(), "Update TF transforms");

    // NOTE We log the interpolated transforms with an offset assuming the whole tree has
    //  been updated after this offset. This is not an ideal solution. If a frame is updated
    //  with a delay longer than this offset we will never log interpolated transforms for it.
    //  It might be possible to always log the interpolated transforms on a per frame basis whenever
    //  a new message is received for that frame. This would require maintaining the latest
    //  transform for each frame. However, this would not work if transforms for a frame arrive
    //  out of order (maybe this is not a problem in practice?).

    auto now = this->get_clock()->now();
    for (const auto& [frame, entity_path] : _tf_frame_to_entity_path) {
        auto parent = _tf_frame_to_parent.find(frame);
        if (parent == _tf_frame_to_parent.end() or parent->second.empty()) {
            continue;
        }
        try {
            auto transform =
                _tf_buffer->lookupTransform(parent->second, frame, now - rclcpp::Duration(1, 0));
            log_transform(_rec, entity_path, transform);
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
    RerunLoggerNode::_create_image_subscription(const std::string& topic) {
    std::string entity_path = _resolve_entity_path(topic);
    bool lookup_transform = (_topic_to_entity_path.find(topic) == _topic_to_entity_path.end());
    ImageOptions image_options;

    if (_topic_options.find(topic) != _topic_options.end()) {
        image_options = _topic_options.at(topic).as<ImageOptions>();
    }

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    RCLCPP_INFO(
        this->get_logger(),
        "Subscribing to image topic %s, logging to entity path %s",
        topic.c_str(),
        entity_path.c_str()
    );

    return this->create_subscription<sensor_msgs::msg::Image>(
        topic,
        1000,
        [&, entity_path, lookup_transform, image_options](
            const sensor_msgs::msg::Image::SharedPtr msg
        ) {
            if (!_root_frame.empty() && lookup_transform) {
                try {
                    auto transform = _tf_buffer->lookupTransform(
                        _root_frame,
                        msg->header.frame_id,
                        msg->header.stamp,
                        100ms
                    );
                    log_transform(_rec, parent_entity_path(entity_path), transform);
                } catch (tf2::TransformException& ex) {
                    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
                }
            }
            log_image(_rec, entity_path, msg, image_options);
        },
        subscription_options
    );
}

std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>>
    RerunLoggerNode::_create_imu_subscription(const std::string& topic) {
    std::string entity_path = _resolve_entity_path(topic);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    return this->create_subscription<sensor_msgs::msg::Imu>(
        topic,
        1000,
        [&, entity_path](const sensor_msgs::msg::Imu::SharedPtr msg) {
            log_imu(_rec, entity_path, msg);
        },
        subscription_options
    );
}

std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>>
    RerunLoggerNode::_create_pose_stamped_subscription(const std::string& topic) {
    std::string entity_path = _resolve_entity_path(topic);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    return this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic,
        1000,
        [&, entity_path](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            log_pose_stamped(_rec, entity_path, msg);
        },
        subscription_options
    );
}

std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>>
    RerunLoggerNode::_create_tf_message_subscription(const std::string& topic) {
    std::string entity_path = _resolve_entity_path(topic);
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
    RerunLoggerNode::_create_odometry_subscription(const std::string& topic) {
    std::string entity_path = _resolve_entity_path(topic);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    return this->create_subscription<nav_msgs::msg::Odometry>(
        topic,
        1000,
        [&, entity_path](const nav_msgs::msg::Odometry::SharedPtr msg) {
            log_odometry(_rec, entity_path, msg);
        },
        subscription_options
    );
}

std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>>
    RerunLoggerNode::_create_camera_info_subscription(const std::string& topic) {
    std::string entity_path = _resolve_entity_path(topic);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = _parallel_callback_group;

    // If the camera_info topic has not been explicility mapped to an entity path,
    // we assume that the camera_info topic is a sibling of the image topic, and
    // hence use the parent as the entity path for the pinhole model.
    if (_topic_to_entity_path.find(topic) == _topic_to_entity_path.end()) {
        entity_path = parent_entity_path(entity_path);
    }

    return this->create_subscription<sensor_msgs::msg::CameraInfo>(
        topic,
        1,
        [&, entity_path](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            log_camera_info(_rec, entity_path, msg);
        },
        subscription_options
    );
}

namespace YAML {
    template <>
    struct convert<ImageOptions> {
        static bool decode(const Node& node, ImageOptions& rhs) {
            int total = 0;

            if (!node.IsMap()) {
                return false;
            }

            if (node["min_depth"]) {
                rhs.min_depth = node["max_depth"].as<float>();
                ++total;
            }
            if (node["max_depth"]) {
                rhs.max_depth = node["max_depth"].as<float>();
                ++total;
            }

            if (total != node.size()) {
                return false;
            }

            return true;
        }
    };
} // namespace YAML

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    // Executor does not take ownership of the node, so we have to maintain a shared_ptr
    auto rerun_logger_node = std::make_shared<RerunLoggerNode>();
    executor.add_node(rerun_logger_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}

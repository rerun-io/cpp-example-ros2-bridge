#include "rerun_bridge/rerun_ros_interface.hpp"
#include "collection_adapters.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

// Turbo colormap lookup table
// from: https://gist.github.com/mikhailov-work/6a308c20e494d9e0ccc29036b28faa7a
// Copyright 2019 Google LLC.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Anton Mikhailov
constexpr float TurboBytes[256][3] = {
    {48, 18, 59},    {50, 21, 67},    {51, 24, 74},   {52, 27, 81},   {53, 30, 88},
    {54, 33, 95},    {55, 36, 102},   {56, 39, 109},  {57, 42, 115},  {58, 45, 121},
    {59, 47, 128},   {60, 50, 134},   {61, 53, 139},  {62, 56, 145},  {63, 59, 151},
    {63, 62, 156},   {64, 64, 162},   {65, 67, 167},  {65, 70, 172},  {66, 73, 177},
    {66, 75, 181},   {67, 78, 186},   {68, 81, 191},  {68, 84, 195},  {68, 86, 199},
    {69, 89, 203},   {69, 92, 207},   {69, 94, 211},  {70, 97, 214},  {70, 100, 218},
    {70, 102, 221},  {70, 105, 224},  {70, 107, 227}, {71, 110, 230}, {71, 113, 233},
    {71, 115, 235},  {71, 118, 238},  {71, 120, 240}, {71, 123, 242}, {70, 125, 244},
    {70, 128, 246},  {70, 130, 248},  {70, 133, 250}, {70, 135, 251}, {69, 138, 252},
    {69, 140, 253},  {68, 143, 254},  {67, 145, 254}, {66, 148, 255}, {65, 150, 255},
    {64, 153, 255},  {62, 155, 254},  {61, 158, 254}, {59, 160, 253}, {58, 163, 252},
    {56, 165, 251},  {55, 168, 250},  {53, 171, 248}, {51, 173, 247}, {49, 175, 245},
    {47, 178, 244},  {46, 180, 242},  {44, 183, 240}, {42, 185, 238}, {40, 188, 235},
    {39, 190, 233},  {37, 192, 231},  {35, 195, 228}, {34, 197, 226}, {32, 199, 223},
    {31, 201, 221},  {30, 203, 218},  {28, 205, 216}, {27, 208, 213}, {26, 210, 210},
    {26, 212, 208},  {25, 213, 205},  {24, 215, 202}, {24, 217, 200}, {24, 219, 197},
    {24, 221, 194},  {24, 222, 192},  {24, 224, 189}, {25, 226, 187}, {25, 227, 185},
    {26, 228, 182},  {28, 230, 180},  {29, 231, 178}, {31, 233, 175}, {32, 234, 172},
    {34, 235, 170},  {37, 236, 167},  {39, 238, 164}, {42, 239, 161}, {44, 240, 158},
    {47, 241, 155},  {50, 242, 152},  {53, 243, 148}, {56, 244, 145}, {60, 245, 142},
    {63, 246, 138},  {67, 247, 135},  {70, 248, 132}, {74, 248, 128}, {78, 249, 125},
    {82, 250, 122},  {85, 250, 118},  {89, 251, 115}, {93, 252, 111}, {97, 252, 108},
    {101, 253, 105}, {105, 253, 102}, {109, 254, 98}, {113, 254, 95}, {117, 254, 92},
    {121, 254, 89},  {125, 255, 86},  {128, 255, 83}, {132, 255, 81}, {136, 255, 78},
    {139, 255, 75},  {143, 255, 73},  {146, 255, 71}, {150, 254, 68}, {153, 254, 66},
    {156, 254, 64},  {159, 253, 63},  {161, 253, 61}, {164, 252, 60}, {167, 252, 58},
    {169, 251, 57},  {172, 251, 56},  {175, 250, 55}, {177, 249, 54}, {180, 248, 54},
    {183, 247, 53},  {185, 246, 53},  {188, 245, 52}, {190, 244, 52}, {193, 243, 52},
    {195, 241, 52},  {198, 240, 52},  {200, 239, 52}, {203, 237, 52}, {205, 236, 52},
    {208, 234, 52},  {210, 233, 53},  {212, 231, 53}, {215, 229, 53}, {217, 228, 54},
    {219, 226, 54},  {221, 224, 55},  {223, 223, 55}, {225, 221, 55}, {227, 219, 56},
    {229, 217, 56},  {231, 215, 57},  {233, 213, 57}, {235, 211, 57}, {236, 209, 58},
    {238, 207, 58},  {239, 205, 58},  {241, 203, 58}, {242, 201, 58}, {244, 199, 58},
    {245, 197, 58},  {246, 195, 58},  {247, 193, 58}, {248, 190, 57}, {249, 188, 57},
    {250, 186, 57},  {251, 184, 56},  {251, 182, 55}, {252, 179, 54}, {252, 177, 54},
    {253, 174, 53},  {253, 172, 52},  {254, 169, 51}, {254, 167, 50}, {254, 164, 49},
    {254, 161, 48},  {254, 158, 47},  {254, 155, 45}, {254, 153, 44}, {254, 150, 43},
    {254, 147, 42},  {254, 144, 41},  {253, 141, 39}, {253, 138, 38}, {252, 135, 37},
    {252, 132, 35},  {251, 129, 34},  {251, 126, 33}, {250, 123, 31}, {249, 120, 30},
    {249, 117, 29},  {248, 114, 28},  {247, 111, 26}, {246, 108, 25}, {245, 105, 24},
    {244, 102, 23},  {243, 99, 21},   {242, 96, 20},  {241, 93, 19},  {240, 91, 18},
    {239, 88, 17},   {237, 85, 16},   {236, 83, 15},  {235, 80, 14},  {234, 78, 13},
    {232, 75, 12},   {231, 73, 12},   {229, 71, 11},  {228, 69, 10},  {226, 67, 10},
    {225, 65, 9},    {223, 63, 8},    {221, 61, 8},   {220, 59, 7},   {218, 57, 7},
    {216, 55, 6},    {214, 53, 6},    {212, 51, 5},   {210, 49, 5},   {208, 47, 5},
    {206, 45, 4},    {204, 43, 4},    {202, 42, 4},   {200, 40, 3},   {197, 38, 3},
    {195, 37, 3},    {193, 35, 2},    {190, 33, 2},   {188, 32, 2},   {185, 30, 2},
    {183, 29, 2},    {180, 27, 1},    {178, 26, 1},   {175, 24, 1},   {172, 23, 1},
    {169, 22, 1},    {167, 20, 1},    {164, 19, 1},   {161, 18, 1},   {158, 16, 1},
    {155, 15, 1},    {152, 14, 1},    {149, 13, 1},   {146, 11, 1},   {142, 10, 1},
    {139, 9, 2},     {136, 8, 2},     {133, 7, 2},    {129, 6, 2},    {126, 5, 2},
    {122, 4, 3}};

std::vector<rerun::Color> colormap(
    const std::vector<float>& values, std::optional<float> min_value, std::optional<float> max_value
) {
    if (!min_value) {
        min_value = *std::min_element(values.begin(), values.end());
    }
    if (!max_value) {
        max_value = *std::max_element(values.begin(), values.end());
    }

    std::vector<rerun::Color> colors;

    for (const auto& value : values) {
        auto idx = static_cast<size_t>(
            255 * (value - min_value.value()) / (max_value.value() - min_value.value())
        );
        colors.emplace_back(rerun::Color(TurboBytes[idx][0], TurboBytes[idx][1], TurboBytes[idx][2])
        );
    }
    return colors;
}

void log_imu(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg
) {
    rec.set_time_seconds(
        "timestamp",
        rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds()
    );

    rec.log(entity_path + "/x", rerun::Scalar(msg->linear_acceleration.x));
    rec.log(entity_path + "/y", rerun::Scalar(msg->linear_acceleration.y));
    rec.log(entity_path + "/z", rerun::Scalar(msg->linear_acceleration.z));
}

void log_image(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::msg::Image::ConstSharedPtr& msg, const ImageOptions& options
) {
    rec.set_time_seconds(
        "timestamp",
        rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds()
    );

    // Depth images are 32-bit float (in meters) or 16-bit uint (in millimeters)
    // See: https://ros.org/reps/rep-0118.html
    if (msg->encoding == "16UC1") {
        cv::Mat img = cv_bridge::toCvCopy(msg)->image;
        rec.log(
            entity_path,
            rerun::DepthImage(
                {static_cast<size_t>(img.rows), static_cast<size_t>(img.cols)},
                rerun::TensorBuffer::u16(img)
            )
                .with_meter(1000)
        );
    } else if (msg->encoding == "32FC1") {
        cv::Mat img = cv_bridge::toCvCopy(msg)->image;
        if (options.min_depth) {
            cv::threshold(img, img, options.min_depth.value(), 0, cv::THRESH_TOZERO);
        }
        if (options.max_depth) {
            cv::threshold(img, img, options.max_depth.value(), 0, cv::THRESH_TOZERO_INV);
        }
        rec.log(
            entity_path,
            rerun::DepthImage(
                {static_cast<size_t>(img.rows), static_cast<size_t>(img.cols)},
                rerun::TensorBuffer::f32(img)
            )
                .with_meter(1.0)
        );
    } else {
        cv::Mat img = cv_bridge::toCvCopy(msg, "rgb8")->image;
        rec.log(entity_path, rerun::Image(tensor_shape(img), rerun::TensorBuffer::u8(img)));
    }
}

void log_pose_stamped(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg
) {
    rec.set_time_seconds(
        "timestamp",
        rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds()
    );

    rec.log(
        entity_path,
        rerun::Transform3D(
            rerun::Vector3D(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
            rerun::Quaternion::from_wxyz(
                msg->pose.orientation.w,
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z
            )
        )
    );

    // this is a somewhat hacky way to get a trajectory visualization in Rerun
    // this should be be easier in the future, see https://github.com/rerun-io/rerun/issues/723
    std::string trajectory_entity_path = "/trajectories/" + entity_path;
    rec.log(
        trajectory_entity_path,
        rerun::Points3D(
            {{static_cast<float>(msg->pose.position.x),
              static_cast<float>(msg->pose.position.y),
              static_cast<float>(msg->pose.position.z)}}
        )
    );
}

void log_tf_message(
    const rerun::RecordingStream& rec,
    const std::map<std::string, std::string>& tf_frame_to_entity_path,
    const tf2_msgs::msg::TFMessage::ConstSharedPtr& msg
) {
    for (const auto& transform : msg->transforms) {
        if (tf_frame_to_entity_path.find(transform.child_frame_id) ==
            tf_frame_to_entity_path.end()) {
            rec.disable_timeline("timestamp");
            rec.log("/", rerun::TextLog("No entity path for frame_id " + transform.child_frame_id));
            continue;
        }

        rec.set_time_seconds(
            "timestamp",
            rclcpp::Time(transform.header.stamp.sec, transform.header.stamp.nanosec).seconds()
        );

        rec.log(
            tf_frame_to_entity_path.at(transform.child_frame_id),
            rerun::Transform3D(
                rerun::Vector3D(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ),
                rerun::Quaternion::from_wxyz(
                    transform.transform.rotation.w,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z
                )
            )
        );
    }
}

void log_odometry(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg
) {
    rec.set_time_seconds(
        "timestamp",
        rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds()
    );

    rec.log(
        entity_path,
        rerun::Transform3D(
            rerun::Vector3D(
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z
            ),
            rerun::Quaternion::from_wxyz(
                msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z
            )
        )
    );
}

void log_camera_info(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg
) {
    // Rerun uses column-major order for Mat3x3
    const std::array<float, 9> image_from_camera = {
        static_cast<float>(msg->k[0]),
        static_cast<float>(msg->k[3]),
        static_cast<float>(msg->k[6]),
        static_cast<float>(msg->k[1]),
        static_cast<float>(msg->k[4]),
        static_cast<float>(msg->k[7]),
        static_cast<float>(msg->k[2]),
        static_cast<float>(msg->k[5]),
        static_cast<float>(msg->k[8]),
    };
    rec.log(
        entity_path,
        rerun::Pinhole(image_from_camera)
            .with_resolution(static_cast<int>(msg->width), static_cast<int>(msg->height))
    );
}

void log_transform(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const geometry_msgs::msg::TransformStamped::ConstSharedPtr& msg
) {
    rec.set_time_seconds(
        "timestamp",
        rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds()
    );

    rec.log(
        entity_path,
        rerun::Transform3D(
            rerun::Vector3D(
                msg->transform.translation.x,
                msg->transform.translation.y,
                msg->transform.translation.z
            ),
            rerun::Quaternion::from_wxyz(
                msg->transform.rotation.w,
                msg->transform.rotation.x,
                msg->transform.rotation.y,
                msg->transform.rotation.z
            )
        )
    );
}

void log_point_cloud2(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, const PointCloud2Options& options
) {
    rec.set_time_seconds(
        "timestamp",
        rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds()
    );

    // TODO(leo) should match the behavior described here
    // https://wiki.ros.org/rviz/DisplayTypes/PointCloud
    // TODO(leo) if not specified, check if 2D points or 3D points
    // TODO(leo) allow arbitrary color mapping

    size_t x_offset, y_offset, z_offset;
    bool has_x{false}, has_y{false}, has_z{false};

    for (const auto& field : msg->fields) {
        if (field.name == "x") {
            x_offset = field.offset;
            if(field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
                rec.log(entity_path, rerun::TextLog("Only FLOAT32 x field supported"));
                return;
            }
            has_x = true;
        } else if (field.name == "y") {
            y_offset = field.offset;
            if(field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
                rec.log(entity_path, rerun::TextLog("Only FLOAT32 y field supported"));
                return;
            }
        } else if (field.name == "z") {
            z_offset = field.offset;
            if(field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
                rec.log(entity_path, rerun::TextLog("Only FLOAT32 z field supported"));
                return;
            }
        }
    }

    if (!has_x || !has_y || !has_z) {
        rec.log(entity_path, rerun::TextLog("Currently only PointCloud2 messages with x, y, z fields are supported"));
        return;
    }

    std::vector<rerun::Position3D> points(msg->width * msg->height);
    std::vector<rerun::Color> colors;

    for (size_t i = 0; i < msg->height; ++i) {
        for (size_t j = 0; j < msg->width; ++j) {
            auto point_offset = i * msg->row_step + j * msg->point_step;
            rerun::Position3D position;
            // TODO(leo) if xyz are consecutive fields we can do this in a single memcpy
            std::memcpy(&position.xyz.xyz[0], &msg->data[point_offset + x_offset], sizeof(float));
            std::memcpy(&position.xyz.xyz[1], &msg->data[point_offset + y_offset], sizeof(float));
            std::memcpy(&position.xyz.xyz[2], &msg->data[point_offset + z_offset], sizeof(float));
            points.emplace_back(std::move(position));
        }
    }

    if (options.colormap == "turbo") {
        std::vector<float> values(msg->width * msg->height);
        auto& colormap_field =
            *std::find_if(msg->fields.begin(), msg->fields.end(), [&](const auto& field) {
                return field.name == options.colormap_field;
            });
        for (size_t i = 0; i < msg->height; ++i) {
            for (size_t j = 0; j < msg->width; ++j) {
                float value;
                std::memcpy(
                    &value,
                    &msg->data[i * msg->row_step + j * msg->point_step + colormap_field.offset],
                    sizeof(float)
                );
                values.emplace_back(value);
            }
        }
        colors = colormap(values, options.colormap_min, options.colormap_max);
    } else if (options.colormap) {
        rec.log("/", rerun::TextLog("Unsupported colormap specified: " + options.colormap.value()));
    }

    rec.log(entity_path, rerun::Points3D(points).with_colors(colors));
}

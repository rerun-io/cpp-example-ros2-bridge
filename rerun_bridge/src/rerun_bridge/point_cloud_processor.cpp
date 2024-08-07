#include "rerun_bridge/point_cloud_processor.hpp"
#include "rerun_bridge/color_mapper.hpp"

#include "rclcpp/rclcpp.hpp"

PointCloudProcessor::PointCloudProcessor(const PointCloud2Options& options)
    : colorMapper_(options) {}

bool PointCloudProcessor::isMsgLonger(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg
) const {
    return (msg->width * msg->height) > maxNumPerMsg_;
}

void PointCloudProcessor::reserve(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    if (isMsgLonger(msg)) {
        const auto size = msg->width * msg->height;
        points_.reserve(size);
        colorMapper_.reserve(size);
        maxNumPerMsg_ = size;
    }
    points_.clear();
    colorMapper_.clear();
}

const std::vector<rerun::components::Position3D>& PointCloudProcessor::convertPclMsgToPosition3DVec(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg
) {
    // TODO(leo) if xyz are consecutive fields we can do this in a single memcpy
    auto iterX = pclConstIter(*msg, "x");
    auto iterY = pclConstIter(*msg, "y");
    auto iterZ = pclConstIter(*msg, "z");
    for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ) {
        points_.emplace_back(*iterX, *iterY, *iterZ);
    }
    return points_;
}

bool PointCloudProcessor::isFieldTypeFloat(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::msg::PointField& field
) const {
    auto result = (field.datatype == sensor_msgs::msg::PointField::FLOAT32);
    if (!result) {
        rec.log(entity_path, rerun::TextLog("Only FLOAT32 " + field.name + " field supported"));
    }
    return result;
}

bool PointCloudProcessor::areFieldNamesSupported(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg
) const {
    // TODO(leo) should match the behavior described here
    // https://wiki.ros.org/rviz/DisplayTypes/PointCloud
    // TODO(leo) if not specified, check if 2D points or 3D points

    bool hasXfield{false}, hasYfield{false}, hasZfield{false};

    for (const auto& field : msg->fields) {
        if (field.name == "x") {
            hasXfield = true;
            if (!isFieldTypeFloat(rec, entity_path, field)) {
                return false;
            }
        } else if (field.name == "y") {
            hasYfield = true;
            if (!isFieldTypeFloat(rec, entity_path, field)) {
                return false;
            }
        } else if (field.name == "z") {
            hasZfield = true;
            if (!isFieldTypeFloat(rec, entity_path, field)) {
                return false;
            }
        }
    }
    return (hasXfield && hasYfield && hasZfield);
}

void PointCloudProcessor::logPointCloud2(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg
) {
    reserve(msg);

    rec.set_time_seconds(
        "timestamp",
        rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds()
    );

    if (!areFieldNamesSupported(rec, entity_path, msg)) {
        rec.log(
            entity_path,
            rerun::TextLog("Currently only PointCloud2 messages with x, y, z fields are supported")
        );
        return;
    }

    const auto& points = convertPclMsgToPosition3DVec(msg);
    if (colorMapper_.isValid()) {
        colorMapper_.calculateRerunColors(msg);
    } else if (colorMapper_.options_.colormapName) {
        rec.log(
            "/",
            rerun::TextLog(
                "Unsupported colormap specified: " + colorMapper_.options_.colormapName.value()
            )
        );
    }
    rec.log(entity_path, rerun::Points3D(points).with_colors(colorMapper_.getColors()));
}

#pragma once

#include <algorithm>
#include <optional>
#include <vector>

#include "rerun.hpp"
#include "rerun_bridge/color_mapper.hpp" // PointCloud2Options
#include "rerun_bridge/rerun_ros_interface.hpp"

class PointCloudProcessor {
    ColorMapper colorMapper_;
    std::size_t maxNumPerMsg_{};
    std::vector<rerun::Position3D> points_{};

    std::pair<std::optional<float>, std::optional<float>> getMinMax(const std::vector<float>& values
    ) const;

    void reserve(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    bool isMsgLonger(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) const;
    const std::vector<rerun::components::Position3D>& convertPclMsgToPosition3DVec(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg
    );
    void convertPclMsgToColorMapFieldVec(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, std::vector<float>& values
    ) const;
    void convertPclMsgToPosition3DVecIter(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
        std::vector<rerun::components::Position3D>& points
    ) const;
    bool areFieldNamesSupported(
        const rerun::RecordingStream& rec, const std::string& entity_path,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg
    ) const;
    bool isFieldTypeFloat(
        const rerun::RecordingStream& rec, const std::string& entity_path,
        const sensor_msgs::msg::PointField& field
    ) const;

  public:
    PointCloudProcessor(const PointCloud2Options& options);
    void logPointCloud2(
        const rerun::RecordingStream& rec, const std::string& entity_path,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg
    );
};
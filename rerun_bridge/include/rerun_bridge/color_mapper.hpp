#pragma once

#include <optional>
#include <string>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "rerun_bridge/color_maps.hpp"
#include "rerun_bridge/rerun_ros_interface.hpp"

using pclConstIter = sensor_msgs::PointCloud2ConstIterator<float>;

struct PointCloud2Options {
    std::optional<std::string> colormapName;
    std::optional<std::string> colormapField;
    std::optional<float> colormapMin;
    std::optional<float> colormapMax;
};

class ColorMapper {
    std::optional<colormapLUT> colormap_{};
    std::vector<float> fieldValues_{};
    std::vector<rerun::Color> colors_{};

    std::pair<float, float> getMinMax(const std::vector<float>& values) const;
    std::size_t calculateIdx(const float& value, const float& minValue, const float& maxValue)
        const;
    void setColormap();
    std::string toUppercase(std::string str) const;
    void convertPclMsgToColorapFieldVec(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void remapValuesToColors();

  public:
    // data from yaml file
    const PointCloud2Options options_{};

    ColorMapper(const PointCloud2Options& options);
    void calculateRerunColors(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    bool isValid() const;
    void reserve(const size_t& size);
    void clear();
    const std::vector<rerun::components::Color>& getColors() const;
};

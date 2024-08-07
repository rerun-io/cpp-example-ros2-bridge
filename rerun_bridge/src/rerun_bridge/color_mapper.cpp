#include <algorithm>
#include <cctype> // std::toupper
#include <iostream>
#include <optional>
#include <string>

#include "rerun_bridge/color_mapper.hpp"
#include "rerun_bridge/color_maps.hpp"

ColorMapper::ColorMapper(const PointCloud2Options& options) : options_{options} {
    setColormap();
}

bool ColorMapper::isValid() const {
    return (colormap_.has_value() && options_.colormapField.has_value());
}

void ColorMapper::calculateRerunColors(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    convertPclMsgToColorapFieldVec(msg);
    remapValuesToColors();
}

std::size_t ColorMapper::calculateIdx(
    const float& value, const float& minValue, const float& maxValue
) const {
    return static_cast<size_t>(255 * (value - minValue) / (maxValue - minValue));
}

std::pair<float, float> ColorMapper::getMinMax(const std::vector<float>& values) const {
    float min_value{0}, max_value{0};
    if (!options_.colormapMin) {
        min_value = *std::min_element(values.begin(), values.end());
    } else {
        min_value = *options_.colormapMin;
    }

    if (!options_.colormapMax) {
        max_value = *std::max_element(values.begin(), values.end());
    } else {
        max_value = *options_.colormapMax;
    }
    return {min_value, max_value};
}

void ColorMapper::reserve(const size_t& size) {
    colors_.reserve(size);
    fieldValues_.reserve(size);
}

void ColorMapper::clear() {
    colors_.clear();
    fieldValues_.clear();
}

std::string ColorMapper::toUppercase(std::string str) const {
    std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) {
        return std::toupper(c);
    });
    return str;
}

void ColorMapper::setColormap() {
    if (!options_.colormapName.has_value()) {
        return;
    }

    // name from YAML file
    auto colormapName = toUppercase(*options_.colormapName);
    auto it = ColormapsLUT::supportedColormaps.find(colormapName);
    if (it != ColormapsLUT::supportedColormaps.end()) {
        colormap_ = it->second;
    } else {
        std::cout << "Colormap: " << colormapName << " is not supported" << std::endl;
    }
}

void ColorMapper::convertPclMsgToColorapFieldVec(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg
) {
    auto iterColorField = pclConstIter(*msg, options_.colormapField.value());
    for (; iterColorField != iterColorField.end(); ++iterColorField) {
        fieldValues_.emplace_back(*iterColorField);
    }
}

void ColorMapper::remapValuesToColors() {
    if (!options_.colormapName || !colormap_) {
        return;
    }

    const auto [min, max] = getMinMax(fieldValues_);
    for (const auto& value : fieldValues_) {
        const auto idx = calculateIdx(value, min, max);
        colors_.emplace_back((*colormap_)[idx][0], (*colormap_)[idx][1], (*colormap_)[idx][2]);
    }
}

const std::vector<rerun::components::Color>& ColorMapper::getColors() const {
    return colors_;
}
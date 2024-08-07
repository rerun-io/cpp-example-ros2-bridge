#pragma once

#include <array>
#include <unordered_map>

using colormapLUT = std::array<std::array<float, 3>, 256>;

namespace ColormapsLUT {
    extern const std::unordered_map<std::string, colormapLUT> supportedColormaps;

    extern const colormapLUT Turbo;
    extern const colormapLUT Rainbow;
} // namespace ColormapsLUT

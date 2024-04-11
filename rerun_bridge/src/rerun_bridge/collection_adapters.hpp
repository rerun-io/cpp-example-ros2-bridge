#pragma once

#include <opencv2/opencv.hpp>
#include <rerun.hpp>

// Adapters so we can borrow an OpenCV image easily into Rerun images without copying:
template <typename TElement>
struct rerun::CollectionAdapter<TElement, cv::Mat> {
    /// Borrow for non-temporary.
    Collection<TElement> operator()(const cv::Mat& img) {
        return Collection<TElement>::borrow(
            reinterpret_cast<TElement*>(img.data),
            img.total() * img.channels()
        );
    }

    // Do a full copy for temporaries (otherwise the data might be deleted when the temporary is destroyed).
    Collection<TElement> operator()(cv::Mat&& img) {
        std::vector<TElement> img_vec(img.total() * img.channels());
        img_vec.assign(img.data, img.data + img.total() * img.channels());
        return Collection<TElement>::take_ownership(std::move(img_vec));
    }
};

inline rerun::Collection<rerun::TensorDimension> tensor_shape(const cv::Mat& img) {
    return {
        static_cast<size_t>(img.rows),
        static_cast<size_t>(img.cols),
        static_cast<size_t>(img.channels())
    };
};

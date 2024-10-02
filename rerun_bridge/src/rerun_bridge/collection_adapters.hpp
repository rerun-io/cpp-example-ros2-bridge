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

inline rerun::WidthHeight width_height(const cv::Mat& img) {
    return rerun::WidthHeight(static_cast<size_t>(img.cols), static_cast<size_t>(img.rows));
};

template <typename T>
rerun::Collection<T> img_data_as_collection(const cv::Mat& img) {
    const T* img_data = reinterpret_cast<const T*>(img.data);

    size_t img_size = img.total() * img.channels();

    std::vector<T> img_vec(img_data, img_data + img_size);

    return rerun::Collection<T>::take_ownership(std::move(img_vec));
}



#include "ImageRawFrameProducer.h"

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "../config.h"
// #pragma GCC optimize("O0")

RawFrame ImageRawFrameProducer::get_raw_frame(const long frame_id, const int camera_num) {
    vector<string> img_color_images = per_cam_img_color_images[camera_num];
    vector<string> img_depth_images = per_cam_img_depth_images[camera_num];

    int min_img_idx = per_cam_img_color_images[0].size();
    for (auto path_names : per_cam_img_color_images) {
        min_img_idx = min(min_img_idx, static_cast<int>(path_names.size()));
    }

    const auto &color_image_path = img_color_images[
        frame_id % min_img_idx];
    const auto &depth_image_path = img_depth_images[
        frame_id % min_img_idx];

    cv::Mat color_mat = cv::imread(color_image_path, cv::IMREAD_UNCHANGED);
    cv::cvtColor(color_mat, color_mat, cv::COLOR_BGR2RGB);

    cv::Mat depth_mat = cv::imread(depth_image_path, cv::IMREAD_UNCHANGED);

    const auto [min_depth, max_depth] = getCamDepthFilters(camera_num);
    depth_mat.setTo(0, depth_mat < min_depth);
    depth_mat.setTo(0, depth_mat > max_depth);

    // Move the entire image 17 pixels to the right
    if (img_dataset == TEST) {
        const cv::Mat shifted_depth_mat = cv::Mat::zeros(depth_mat.rows, depth_mat.cols, depth_mat.type()); // Initialize with zeros
        depth_mat(cv::Rect(img_align_shift, 0, depth_mat.cols - img_align_shift, depth_mat.rows))
        .copyTo(shifted_depth_mat(cv::Rect(0, 0, depth_mat.cols - img_align_shift, depth_mat.rows)));
        return RawFrame{
            .color_mat = color_mat, .depth_mat = shifted_depth_mat
        };
    }

    return RawFrame{
        .color_mat = color_mat, .depth_mat = depth_mat
    };
}

DepthConfig ImageRawFrameProducer::get_depth_config(int camera_num) {
    return per_cam_depth_configs[camera_num];
}

std::tuple<int, int> ImageRawFrameProducer::get_dimensions() {
    return std::make_tuple(img_w, img_h);
}

void ImageRawFrameProducer::clean_up() {
}

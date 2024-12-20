

#ifndef MVFRAMEPREPDATA_H
#define MVFRAMEPREPDATA_H
#include <chrono>
#include <opencv2/core/mat.hpp>

#include "DepthConfig.h"

struct MVFramePrepData {
    const cv::Mat color_mat;
    const cv::Mat depth_mat;
    const cv::Mat prev_color_mat;
    const cv::Mat prev_depth_mat;
    long frame_id;
    DepthConfig depth_config;
    std::chrono::high_resolution_clock::time_point frame_ts;
};

#endif //MVFRAMEPREPDATA_H

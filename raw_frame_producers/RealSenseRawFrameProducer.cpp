

#include "RealSenseRawFrameProducer.h"

#include <iostream>
#include <opencv2/highgui.hpp>


void RealSenseRawFrameProducer::setup_realsense_pipeline(const string &serial) {
    cfg.enable_device(serial);
    // Create a RealSense pipeline, encapsulating the actual device and sensors
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
    // Enable depth stream
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, 30);
    // Enable color stream
    pipeline.start(cfg);

    const auto depth_scale = pipeline.get_active_profile().get_device().first<
                rs2::depth_sensor>().
            get_depth_scale();
    const auto depth_intrinsics = rs2::video_stream_profile(
                pipeline.get_active_profile().get_stream(RS2_STREAM_DEPTH)
            ).get_intrinsics();
    depth_config = DepthConfig{
        .depth_scale = depth_scale,
        .cx = depth_intrinsics.ppx, .cy = depth_intrinsics.ppy,
        .fx = depth_intrinsics.fx, .fy = depth_intrinsics.fy
    };

    // Capture 30 frames to give autoexposure, etc. a chance to settle
    for (auto i = 0; i < 30; ++i) pipeline.wait_for_frames();
    setup_complete = true;
}

RawFrame RealSenseRawFrameProducer::get_raw_frame(const int camera_num) {
    // Wait for the next set of frames from the camera
    const rs2::frameset frames = pipeline.wait_for_frames();
    auto align_ = rs2::align(RS2_STREAM_COLOR);
    const auto aligned_frames = align_.process(frames);

    // Get the depth and color frames
    rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
    rs2::video_frame color_frame = aligned_frames.get_color_frame();

    const cv::Mat current_color(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3,
                                const_cast<void *>(color_frame.get_data()),
                                cv::Mat::AUTO_STEP
    );
    const cv::Mat current_depth(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16UC1,
                                const_cast<void *>(depth_frame.get_data()),
                                cv::Mat::AUTO_STEP
                                );

    auto filtered_depth_mat = current_depth.clone();


    const auto [min_depth, max_depth] = getCamDepthFilters(camera_num);
    filtered_depth_mat.setTo(0, filtered_depth_mat < min_depth);
    filtered_depth_mat.setTo(0, filtered_depth_mat > max_depth);

    return RawFrame{
        .color_mat = current_color.clone(), .depth_mat = filtered_depth_mat
    };
}

DepthConfig RealSenseRawFrameProducer::get_depth_config() {
    return depth_config;
}

std::tuple<int, int> RealSenseRawFrameProducer::get_dimensions() {
    return std::make_tuple(rs_w, rs_h);
}

void RealSenseRawFrameProducer::clean_up() {
    try {
        pipeline.stop();
        std::cout << "Cleaned up RealSense pipeline." << std::endl;
    } catch (std::exception &e) {
        cerr << "Failed to clean up RealSense pipeline. Exception: " << e.what()
                << endl;
    }
}

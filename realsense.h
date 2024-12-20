

#ifndef REALSENSE_H
#define REALSENSE_H
#include <iostream>
#include <boost/asio/ip/tcp.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/RGBDImage.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/visualization/visualizer/Visualizer.h>
#include <opencv2/opencv.hpp>

#include "config.h"

#endif //REALSENSE_H

using namespace std;
namespace o3d = open3d;


inline tuple<rs2::pipeline, rs2::align, float, rs2_intrinsics>
setup_realsense_pipeline()
{
    // Create a RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipeline;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    pipeline.start(cfg);

    auto align = rs2::align(RS2_STREAM_COLOR);
    auto depth_scale = pipeline.get_active_profile().get_device().first<
                                    rs2::depth_sensor>().
                                get_depth_scale();
    auto depth_intrinsics = rs2::video_stream_profile(
            pipeline.get_active_profile().get_stream(RS2_STREAM_DEPTH)
        ).
        get_intrinsics();

    // Capture 30 frames to give autoexposure, etc. a chance to settle
    for (auto i = 0; i < 30; ++i) pipeline.wait_for_frames();

    return make_tuple(pipeline, align, depth_scale, depth_intrinsics);
}


inline std::shared_ptr<open3d::geometry::PointCloud> get_frame_point_cloud(
    rs2::depth_frame depth_frame, rs2::video_frame color_frame,
    float depth_scale, const rs2_intrinsics& depth_intrinsics)
{
    // Get depth frame dimensions
    const int width = depth_frame.get_width();
    const int height = depth_frame.get_height();

    // Create Open3D images for depth and color
    auto depth_image = make_shared<open3d::geometry::Image>();
    depth_image->Prepare(width, height, 1, 2);
    // 1 channel, 2 bytes per pixel
    memcpy(depth_image->data_.data(), depth_frame.get_data(),
           depth_image->data_.size()
    );

    auto color_image = make_shared<open3d::geometry::Image>();
    color_image->Prepare(width, height, 3, 1);
    // 3 channels, 1 byte per pixel
    memcpy(color_image->data_.data(), color_frame.get_data(),
           color_image->data_.size()
    );


    // Create an RGBD image from the color and depth images
    auto rgbd_image = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
        *color_image,
        *depth_image,
        1.0 / depth_scale,
        3,
        false
    );

    // Create a point cloud from the RGBD image
    auto intrinsic = open3d::camera::PinholeCameraIntrinsic(
        width, height,
        depth_intrinsics.fx, depth_intrinsics.fy,
        depth_intrinsics.ppx, depth_intrinsics.ppy
    );
    auto frame_point_cloud = open3d::geometry::PointCloud::CreateFromRGBDImage(
        *rgbd_image, intrinsic
    );

    return frame_point_cloud;
}


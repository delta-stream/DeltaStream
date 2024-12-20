

#include "BlockUpdatePrepper.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

using namespace std;

FrameData BlockUpdatePrepper::prepare(const cv::Mat& current_color_rgb, const cv::Mat& current_depth,
                             const cv::Mat& prev_color_rgb, const cv::Mat& prev_depth,
                             long frame_id,
                             DepthConfig depth_config)
{
    // If this is the first frame, initialize prev_* and continue
    if (frame_id == 0)
    {
        throw runtime_error("First frame should be a keyframe");
    }
    int w = current_color_rgb.cols;
    int h = current_color_rgb.rows;

    cv::Mat prev_color_bgr;
    cv::Mat current_color_bgr;

    cv::cvtColor(current_color_rgb, current_color_bgr , cv::COLOR_RGB2BGR);
    cv::cvtColor(prev_color_rgb, prev_color_bgr , cv::COLOR_RGB2BGR);


    // Calculate absolute difference between two frames
    cv::Mat diff_image;
    cv::absdiff(prev_color_bgr, current_color_bgr, diff_image);

    // Initialize mask and depth mask
    cv::Mat mask = cv::Mat::zeros(prev_color_bgr.size(), prev_color_bgr.type());
    cv::Mat depth_mask = cv::Mat::zeros(h, w, CV_8UC1);

    std::vector<int> block_index_to_remove;

    // Loop over 16x16 blocks
    for (int y = 0; y < h; y += block_size)
    {
        for (int x = 0; x < w; x += block_size)
        {
            // Calculate the sum of absolute differences (SAD) for the block
            cv::Rect block_rect(x, y, block_size, block_size);
            cv::Mat block_diff = diff_image(block_rect);
            cv::Scalar sum_diff = cv::sum(block_diff);
            double block_diff_value = sum_diff[0] + sum_diff[1] + sum_diff[2]; // Sum over channels

            // If the block difference exceeds the threshold, process the block
            if (block_diff_value > threshold)
            {
                int block_number = (y / block_size) * (w / block_size) + (x / block_size);
                block_index_to_remove.push_back(block_number);

                // Mark the block and its neighbors
                std::vector<std::pair<int, int>> neighbors = {
                    {y, x}, // Current block
                    {y - block_size, x}, // Up
                    {y + block_size, x}, // Down
                    {y, x - block_size}, // Left
                    {y, x + block_size}, // Right
                    {y - block_size, x - block_size}, // Top-left diagonal
                    {y - block_size, x + block_size}, // Top-right diagonal
                    {y + block_size, x - block_size}, // Bottom-left diagonal
                    {y + block_size, x + block_size} // Bottom-right diagonal
                };

                for (auto& neighbor : neighbors)
                {
                    int ny = neighbor.first;
                    int nx = neighbor.second;
                    if (0 <= ny && ny < h && 0 <= nx && nx < w)
                    {
                        int n_block_number = (ny / block_size) * (w / block_size) + (nx / block_size);
                        if (std::find(block_index_to_remove.begin(), block_index_to_remove.end(),
                                      n_block_number) == block_index_to_remove.end())
                        {
                            block_index_to_remove.push_back(n_block_number);
                        }
                        cv::Rect neighbor_block(nx, ny, block_size, block_size);
                        neighbor_block &= cv::Rect(0, 0, w, h); // Ensure within image boundaries
                        current_color_bgr(neighbor_block).copyTo(mask(neighbor_block));
                        depth_mask(neighbor_block).setTo(1);
                    }
                }
            }
        }
    }

    // Create masked image
    cv::Mat masked_image;
    cv::bitwise_and(current_color_bgr, mask, masked_image);


    // Convert BGR to RGB
    cv::Mat masked_image_rgb;
    cv::cvtColor(masked_image, masked_image_rgb, cv::COLOR_BGR2RGB);



    // Mask the depth image
    cv::Mat masked_depth = cv::Mat::zeros(current_depth.size(), current_depth.type());
    current_depth.copyTo(masked_depth, depth_mask);



    // Create Open3D images from masked image and depth
    auto rgb_delta = std::make_shared<o3d::geometry::Image>();
    auto depth_delta = std::make_shared<o3d::geometry::Image>();
    rgb_delta->Prepare(w, h, 3, 1);
    depth_delta->Prepare(w, h, 1, 2);

    memcpy(rgb_delta->data_.data(), masked_image_rgb.data,
           masked_image_rgb.total() * masked_image_rgb.elemSize());
    memcpy(depth_delta->data_.data(), masked_depth.data, masked_depth.total() * masked_depth.elemSize());


    // Create RGBD image and point cloud
    auto rgbd_delta = o3d::geometry::RGBDImage::CreateFromColorAndDepth(
        *rgb_delta, *depth_delta, 1.0 / depth_config.depth_scale, 3, false);

    auto intrinsic = open3d::camera::PinholeCameraIntrinsic(
        w, h,
        depth_config.fx, depth_config.fy,
        depth_config.cx, depth_config.cy
    );


    auto point_cloud_delta = o3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd_delta, intrinsic);

    return FrameData(frame_id, point_cloud_delta, block_index_to_remove);
}

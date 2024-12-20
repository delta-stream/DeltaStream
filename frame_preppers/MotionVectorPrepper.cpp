

#include "MotionVectorPrepper.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <mutex>
// #pragma GCC optimize("O0")

#include "../FfmpegToolbox.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/motion_vector.h>
#include <libavutil/imgutils.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

using namespace std;



static AVFormatContext* fmt_ctx = nullptr;
static AVCodecContext* video_dec_ctx = nullptr;
static AVStream* video_stream = nullptr;

static int video_stream_idx = -1;
static AVFrame* frame = nullptr;
static int video_frame_count = 0;

// In-memory buffer for encoded data
static std::vector<uint8_t> encoded_buffer;
static size_t encoded_buffer_pos = 0;

tuple<map<int, DeltaMotion>, vector<int>, shared_ptr<open3d::geometry::PointCloud>> aa(
    const vector<AVMotionVector>& mvs,
    const cv::Mat& color_frame,
    const cv::Mat& depth_frame,
    const cv::Mat& prev_color_frame,
    const cv::Mat& prev_depth_frame,
    const DepthConfig& depth_config,
    long frame_id,
    int max_mv_blocks
)
{
    auto [depth_scale, cx, cy, fx, fy] = depth_config;

    const int width = color_frame.cols;
    const int height = color_frame.rows;

    // Filter
    const int blocks_per_row = width / block_size;
    auto motion_vectors_depth_filtered = vector<AVMotionVector>();

    for (const auto& mv : mvs)
    {
        auto [source, w, h, src_x, src_y, dst_x, dst_y, flags, motion_x, motion_y, motion_scale] = mv;
        const int col_index = dst_x / block_size;
        const int row_index = dst_y / block_size;

        int block_index = row_index * blocks_per_row + col_index;

        // Define source and destination rectangles
        cv::Rect src_rect(src_x, src_y, block_size, block_size);
        cv::Rect dst_rect(dst_x, dst_y, block_size, block_size);

        // Adjust rectangles to image boundaries
        src_rect &= cv::Rect(0, 0, prev_depth_frame.cols, prev_depth_frame.rows);
        dst_rect &= cv::Rect(0, 0, depth_frame.cols, depth_frame.rows);

        // Check if rectangles are valid
        if (src_rect.width > 0 && src_rect.height > 0 && dst_rect.width > 0 && dst_rect.height > 0)
        {
            cv::Mat block_prev_depth_frame = prev_depth_frame(src_rect);
            cv::Mat block_depth_frame = depth_frame(dst_rect);

            double sum1 = cv::sum(block_prev_depth_frame)[0];
            double sum2 = cv::sum(block_depth_frame)[0];

            // Check if the depth sum of the block is greater than 0
            if (sum1 > 0 || sum2 > 0)
            {
                motion_vectors_depth_filtered.push_back(mv);
            }
        }

        // Limit the number of blocks that can be updated using motion vector
        if (motion_vectors_depth_filtered.size() >= max_mv_blocks)
        {
            // cout << "Max motion vector blocks reached! " << max_mv_blocks << endl;
            break;
        }
    }


    cv::Mat compensated_frame = cv::Mat::zeros(prev_color_frame.size(), prev_color_frame.type());

    for (int y = 0; y < height; y += block_size)
    {
        for (int x = 0; x < width; x += block_size)
        {
            // Current block's center point
            int dst_x_center = x + half_block_size;
            int dst_y_center = y + half_block_size;

            // Find motion vector for this block
            const AVMotionVector* mv = nullptr;
            auto it = std::find_if(motion_vectors_depth_filtered.begin(), motion_vectors_depth_filtered.end(),
                                   [dst_x_center, dst_y_center](const AVMotionVector& m)
                                   {
                                       return m.dst_x == dst_x_center && m.dst_y == dst_y_center;
                                   });
            if (it != motion_vectors_depth_filtered.end())
            {
                mv = &(*it);
            }

            cv::Rect block_rect(x, y, block_size, block_size);
            block_rect &= cv::Rect(0, 0, width, height); // Ensure the block is within image bounds

            if (mv != nullptr)
            {
                // If there is a motion vector
                int motion_x = mv->motion_x;
                int motion_y = mv->motion_y;
                int motion_scale = mv->motion_scale;

                // Calculate estimated motion
                double actual_motion_x = static_cast<double>(motion_x) / motion_scale;
                double actual_motion_y = static_cast<double>(motion_y) / motion_scale;

                // Calculate src_x, src_y
                int src_x = static_cast<int>(x + actual_motion_x);
                int src_y = static_cast<int>(y + actual_motion_y);

                // Check whether source block is in the frame
                if (src_x >= 0 && src_x <= width - block_size &&
                    src_y >= 0 && src_y <= height - block_size)
                {
                    cv::Rect src_rect(src_x, src_y, block_size, block_size);
                    src_rect &= cv::Rect(0, 0, width, height); // Ensure the source block is within bounds

                    // Copy the block from image1 to compensated_frame
                    prev_color_frame(src_rect).copyTo(compensated_frame(block_rect));
                }
                else
                {
                    // Out of range, copy the original block
                    prev_color_frame(block_rect).copyTo(compensated_frame(block_rect));
                }
            }
            else
            {
                // If no motion vector, copy the original block
                prev_color_frame(block_rect).copyTo(compensated_frame(block_rect));
            }
        }
    }

    // Calculate absolute difference between two frames
    cv::Mat diff_image, diff_image_compensated;
    cv::absdiff(prev_color_frame, color_frame, diff_image);
    cv::absdiff(compensated_frame, color_frame, diff_image_compensated);

    // Filtering for blocks which can be covered by motion vectors

    // Initialize arrays for the predicted frame and mask
    cv::Mat mask = cv::Mat::zeros(prev_color_frame.size(), prev_color_frame.type());
    cv::Mat mask_compensated = cv::Mat::zeros(prev_color_frame.size(), prev_color_frame.type());

    cv::Mat depth_mask = cv::Mat::zeros(height, width, CV_8U);
    cv::Mat depth_mask_compensated = cv::Mat::zeros(height, width, CV_8U);


    std::vector<int> block_to_remove;
    std::vector<int> block_covered_by_mv;


    // Loop over 16x16 blocks
    for (int y = 0; y < height; y += block_size)
    {
        for (int x = 0; x < width; x += block_size)
        {
            // Calculate the sum of absolute differences (SAD) for the block
            cv::Rect block_rect(x, y, block_size, block_size);
            block_rect &= cv::Rect(0, 0, width, height); // Ensure the block is within image bounds
            cv::Mat block_diff_mat = diff_image(block_rect);
            auto cv_sum = cv::sum(block_diff_mat);
            double block_diff = cv::sum(block_diff_mat)[0] + cv::sum(block_diff_mat)[1] + cv::sum(block_diff_mat)[2];

            // If the block difference exceeds the threshold, process the block
            if (block_diff > threshold)
            {
                int block_number = (y / block_size) * (width / block_size) + (x / block_size);
                block_to_remove.push_back(block_number);

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

                for (const auto& neighbor : neighbors)
                {
                    int ny = neighbor.first;
                    int nx = neighbor.second;
                    if (0 <= ny && ny < height && 0 <= nx && nx < width)
                    {
                        int n_block_number = (ny / block_size) * (width / block_size) + (nx / block_size);
                        if (std::find(block_to_remove.begin(), block_to_remove.end(), n_block_number) == block_to_remove
                            .end())
                        {
                            block_to_remove.push_back(n_block_number);
                        }

                        cv::Rect neighbor_block_rect(nx, ny, block_size, block_size);
                        neighbor_block_rect &= cv::Rect(0, 0, width, height);
                        // Update mask
                        color_frame(neighbor_block_rect).copyTo(mask(neighbor_block_rect));
                        depth_mask(neighbor_block_rect).setTo(1);
                    }
                }
            }
        }
    }

    for (int block_number : block_to_remove)
    {
        int y = (block_number / (width / block_size)) * block_size;
        int x = (block_number % (width / block_size)) * block_size;

        cv::Rect block_rect(x, y, block_size, block_size);
        block_rect &= cv::Rect(0, 0, width, height);

        // Calculate compensated block difference
        cv::Mat block_diff_compensated_mat = diff_image_compensated(block_rect);
        double block_diff_compensated = cv::sum(block_diff_compensated_mat)[0] + cv::sum(block_diff_compensated_mat)[1]
            + cv::sum(block_diff_compensated_mat)[2];

        cv::Mat depth_block = depth_frame(block_rect);
        depth_block.convertTo(depth_block, CV_32F);
        cv::Scalar mean, stddev;
        cv::meanStdDev(depth_block, mean, stddev);
        double depth_variance = stddev.val[0] * stddev.val[0];

        bool non_zero_depth = cv::countNonZero(depth_block > 0) == depth_block.total();

        // If the compensated block difference is below the mv_threshold and meets other criteria
        if (block_diff_compensated < mv_threshold && non_zero_depth && depth_variance < variance_threshold)
        {
            block_covered_by_mv.push_back(block_number);

            // Update mask for the current block only
            color_frame(block_rect).copyTo(mask_compensated(block_rect));
            depth_mask_compensated(block_rect).setTo(1);
        }
    }


    // Make partial point cloud using mask
    cv::Mat mask_gray;
    if (mask.channels() == 3)
    {
        cv::cvtColor(mask, mask_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        mask_gray = mask;
    }

    cv::Mat masked_image;
    color_frame.copyTo(masked_image, mask_gray);

    cv::Mat mask_compensated_gray;
    if (mask_compensated.channels() == 3)
    {
        cv::cvtColor(mask_compensated, mask_compensated_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        mask_compensated_gray = mask_compensated;
    }

    cv::Mat masked_image_compensated;
    color_frame.copyTo(masked_image_compensated, mask_compensated_gray);


    // Convert to RGB if necessary
    cv::Mat masked_image_rgb;
    cv::cvtColor(masked_image, masked_image_rgb, cv::COLOR_BGR2RGB);

    cv::Mat masked_image_rgb_compensated;
    cv::cvtColor(masked_image_compensated, masked_image_rgb_compensated, cv::COLOR_BGR2RGB);


    // Convert block lists to sets
    std::set<int> block_to_remove_set(block_to_remove.begin(), block_to_remove.end());
    std::set<int> block_covered_by_mv_set(block_covered_by_mv.begin(), block_covered_by_mv.end());

    // Compute the set difference to get additional blocks
    std::set<int> additional_blocks_set;
    std::set_difference(block_to_remove_set.begin(), block_to_remove_set.end(),
                        block_covered_by_mv_set.begin(), block_covered_by_mv_set.end(),
                        std::inserter(additional_blocks_set, additional_blocks_set.begin()));

    std::vector<int> additional_blocks(additional_blocks_set.begin(), additional_blocks_set.end());
    std::vector<int> block_removes_final(block_to_remove_set.begin(), block_to_remove_set.end());

    // Combine masks for additional blocks
    cv::Mat combined_mask = cv::Mat::zeros(prev_depth_frame.size(), CV_8U);
    // Create an empty mask of the same size as prev_depth_frame

    for (int block_number : additional_blocks)
    {
        int y = (block_number / (width / block_size)) * block_size;
        int x = (block_number % (width / block_size)) * block_size;

        cv::Rect block_rect(x, y, block_size, block_size);
        block_rect &= cv::Rect(0, 0, width, height);

        combined_mask(block_rect).setTo(255);
    }

    // Apply mask to extract RGB and Depth images
    cv::Mat masked_rgb;
    color_frame.copyTo(masked_rgb, combined_mask);
    // cv::cvtColor(masked_rgb, masked_rgb, cv::COLOR_BGR2RGB);

    cv::Mat masked_depth;
    depth_frame.copyTo(masked_depth, combined_mask);

    // Convert masked_depth to float and scale to meters
    masked_depth.convertTo(masked_depth, CV_32F); //, depth_scale);

    // Check if masked_depth contains non-zero values
    double minVal, maxVal;
    cv::minMaxLoc(masked_depth, &minVal, &maxVal);
    // std::cout << "Masked depth min: " << minVal << ", max: " << maxVal << std::endl;
    if (maxVal == 0)
    {
        // std::cerr << "Error: Masked depth contains only zero values!" << std::endl;
    }


    // Convert masked images to Open3D images
    open3d::geometry::Image masked_rgb_img;
    masked_rgb_img.Prepare(width, height, 3, 1);
    memcpy(masked_rgb_img.data_.data(), masked_rgb.data, masked_rgb.total() * masked_rgb.elemSize());

    open3d::geometry::Image masked_depth_img;
    masked_depth_img.Prepare(width, height, 1, sizeof(float));
    memcpy(masked_depth_img.data_.data(), masked_depth.data, masked_depth.total() * masked_depth.elemSize());

    // Convert and display the masked depth image
    cv::Mat depth_image(height, width, CV_32F, masked_depth_img.data_.data());

    // for debugging
    cv::Mat depth_image_normalized;
    cv::normalize(depth_image, depth_image_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Create RGBD image
    auto masked_rgbd_img = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
        masked_rgb_img, masked_depth_img, 1.0 / depth_config.depth_scale, 3, false);


    auto intrinsic = open3d::camera::PinholeCameraIntrinsic(
        width, height,
         depth_config.fx, depth_config.fy,
        depth_config.cx, depth_config.cy);


    // Create point cloud
    auto additional_pc = open3d::geometry::PointCloud::CreateFromRGBDImage(
        *masked_rgbd_img, intrinsic);

    // Check if additional_pc is empty
    if (additional_pc->points_.empty())
    {
        additional_pc = std::make_shared<open3d::geometry::PointCloud>(); // Set to an empty point cloud
    }


    // Create a map from block_index to motion vector
    std::map<int, AVMotionVector> motion_vectors_dict;
    for (const AVMotionVector& mv : motion_vectors_depth_filtered)
    {
        int dst_x = mv.dst_x;
        int dst_y = mv.dst_y;
        int col_index = dst_x / block_size;
        int row_index = dst_y / block_size;
        int block_index = row_index * (width / block_size) + col_index;

        motion_vectors_dict[block_index] = mv;
    }

    // Prepare to store the motion vectors to send
    std::map<int, DeltaMotion> motion_vector_to_send;

    // Convert 2D motion vectors to 3D motion vectors
    for (int block : block_covered_by_mv)
    {
        auto mv_it = motion_vectors_dict.find(block);
        if (mv_it != motion_vectors_dict.end())
        {
            AVMotionVector mv = mv_it->second;

            int src_x = mv.src_x;
            int src_y = mv.src_y;
            int dst_x = mv.dst_x;
            int dst_y = mv.dst_y;

            int u_1 = static_cast<int>(src_x - block_size / 2.0);
            int v_1 = static_cast<int>(src_y - block_size / 2.0);
            int u_2 = static_cast<int>(dst_x - block_size / 2.0);
            int v_2 = static_cast<int>(dst_y - block_size / 2.0);

            cv::Rect block_rect_1(u_1, v_1, block_size, block_size);
            cv::Rect block_rect_2(u_2, v_2, block_size, block_size);

            block_rect_1 &= cv::Rect(0, 0, width, height);
            block_rect_2 &= cv::Rect(0, 0, width, height);

            if (block_rect_1.size() != block_rect_2.size())
            {
                continue;
            }

            cv::Mat rgb_block_1 = prev_color_frame(block_rect_1);
            cv::Mat depth_block_1 = prev_depth_frame(block_rect_1);
            cv::Mat rgb_block_2 = color_frame(block_rect_2);
            cv::Mat depth_block_2 = depth_frame(block_rect_2);

            depth_block_1.convertTo(depth_block_1, CV_32F); // convert to float
            depth_block_2.convertTo(depth_block_2, CV_32F);


            cv::Mat z_diff = (depth_block_2 - depth_block_1) / (1.0 / depth_config.depth_scale);



            auto z_diff_mean = cv::mean(z_diff);
            double delta_z = cv::mean(z_diff)[0];

            double z1 = cv::mean(depth_block_1)[0] / (1.0 / depth_config.depth_scale);
            double z2 = cv::mean(depth_block_2)[0] / (1.0 / depth_config.depth_scale);


            double x_1 = (u_1 - cx) / fx * z1;
            double y_1 = (v_1 - cy) / fy * z1;
            double x_2 = (u_2 - cx) / fx * z2;
            double y_2 = (v_2 - cy) / fy * z2;

            double delta_x = x_2 - x_1;
            double delta_y = y_2 - y_1;

            // Store in motion_vector_to_send
            motion_vector_to_send[block] = {delta_x, delta_y, delta_z, src_x, src_y};
        }
        else
        {
            // Remove blocks which do not have motion vectors
            std::erase(block_removes_final, block);
        }
    }

    return {motion_vector_to_send, block_removes_final, additional_pc};
}


FrameData MotionVectorPrepper::prepare(const cv::Mat &current_color_rgb, const cv::Mat &current_depth,
                                       const cv::Mat &prev_color_rgb, const cv::Mat &prev_depth,
                                       long frame_id,
                                       DepthConfig depth_config,
                                       chrono::high_resolution_clock::time_point frame_ts, int max_mv_blocks)
{
    auto start_mv_prepare = std::chrono::high_resolution_clock::now();
    int w = current_color_rgb.cols;
    int h = current_color_rgb.rows;

    cv::Mat current_color_bgr;
    cv::Mat prev_color_bgr;
    
    cv::cvtColor(current_color_rgb, current_color_bgr, cv::COLOR_RGB2BGR);
    cv::cvtColor(prev_color_rgb, prev_color_bgr, cv::COLOR_RGB2BGR);

    auto create_video_start = std::chrono::high_resolution_clock::now();
    FfmpegToolbox::create_video_from_images(
        current_color_bgr, prev_color_bgr,
        30, w, h
    );
    auto create_video_end = std::chrono::high_resolution_clock::now();
    auto create_video_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(create_video_end - create_video_start).count();
    // cout << "[F" << frame_id << "] FfmpegToolbox::create_video_from_images: " << create_video_duration_ms << "ms" << endl;

    auto extract_mv_start = std::chrono::high_resolution_clock::now();
    auto motion_vectors = FfmpegToolbox::extract_motion_vectors();
    auto extract_mv_end = std::chrono::high_resolution_clock::now();
    auto extract_mv_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(extract_mv_end - extract_mv_start).count();
    // cout << "[F" << frame_id << "] " << "FfmpegToolbox::extract_motion_vectors: " << extract_mv_duration_ms << "ms" << endl;

    auto aa_start = std::chrono::high_resolution_clock::now();
    const auto [motion_vector_to_send, block_removes_final, additional_pc] = aa(
        motion_vectors,
        current_color_rgb, current_depth,
        prev_color_rgb, prev_depth,
            depth_config,
            frame_id,
            max_mv_blocks

    );
    auto aa_end = std::chrono::high_resolution_clock::now();
    auto aa_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(aa_end - aa_start).count();
    // cout << "[F" << frame_id << "] aa: " << aa_duration_ms << "ms" << endl;

    auto frame_data = FrameData(
        frame_id,
        additional_pc,
        block_removes_final,
        motion_vector_to_send
    );

    auto end_mv_prepare = std::chrono::high_resolution_clock::now();
    long mv_prepare_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_mv_prepare - start_mv_prepare).count();


    frame_data.setCreatedAt(frame_ts);
    return frame_data;
}

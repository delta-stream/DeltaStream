

#ifndef BLOCKUPDATEPREPPER_H
#define BLOCKUPDATEPREPPER_H
#include <librealsense2/hpp/rs_frame.hpp>

#include <open3d/Open3D.h>           // Include Open3D
#include <opencv2/core/mat.hpp>

#include "../FrameData.h"
#include "../custom_types/DepthConfig.h"
#include "../config.h"

namespace o3d = open3d;

// inline int block_size = 16;
// inline double threshold = 10000; // Threshold for block difference

class BlockUpdatePrepper
{
public:
    static FrameData prepare(const cv::Mat& current_color_rgb, const cv::Mat& current_depth,
                             const cv::Mat& prev_color_rgb, const cv::Mat& prev_depth,
                             long frame_id,
                             DepthConfig depth_config);
};

#endif //BLOCKUPDATEPREPPER_H

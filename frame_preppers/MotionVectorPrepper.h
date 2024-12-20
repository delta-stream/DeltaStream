

#ifndef MOTIONVECTORPREPPER_H
#define MOTIONVECTORPREPPER_H

#include <librealsense2/hpp/rs_frame.hpp>

#include <open3d/Open3D.h>           // Include Open3D
#include <opencv2/core/mat.hpp>

#include "../FrameData.h"
#include "../AVBufferData.h"
#include "../DeltaMotion.h"
#include "../config.h"
#include "../custom_types/DepthConfig.h"

using namespace std;
namespace o3d = open3d;



class MotionVectorPrepper {

public:
    static FrameData prepare(const cv::Mat& current_color_rgb, const cv::Mat& current_depth,
                             const cv::Mat& prev_color_rgb, const cv::Mat& prev_depth,
                             long frame_id,
                             DepthConfig depth_config,
                                        chrono::high_resolution_clock::time_point frame_ts,
                                        int max_mv_blocks);


};



#endif //MOTIONVECTORPREPPER_H

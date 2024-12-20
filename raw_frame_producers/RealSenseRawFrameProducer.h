

#ifndef REALSENSERAWFRAMEPRODUCER_H
#define REALSENSERAWFRAMEPRODUCER_H
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <opencv2/core/mat.hpp>

#include "../custom_types/RawFrame.h"
#include "../custom_types/DepthConfig.h"
#include "../config.h"

/**
 * Returns
 * - color_mat
 * - depth_mat
 * - depth_scale
 * - depth_intrinsics
 */
class RealSenseRawFrameProducer {
private:
    rs2::pipeline pipeline;
    rs2::config cfg;
    DepthConfig depth_config = {};
    bool setup_complete = false;
    int width =rs_w;
    int height = rs_h;

public:
    RealSenseRawFrameProducer() {
    }

    void setup_realsense_pipeline(const string &serial);
    RawFrame get_raw_frame(const int camera_num);
    DepthConfig get_depth_config();

    std::tuple<int, int> get_dimensions();
    void clean_up();

};





#endif //REALSENSERAWFRAMEPRODUCER_H

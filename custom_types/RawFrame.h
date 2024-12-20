

#ifndef RAWFRAME_H
#define RAWFRAME_H
#include <opencv2/core/mat.hpp>

/*
*
* Returns
* - color_mat
* - depth_mat
* - depth_scale
* - depth_intrinsics
*/
struct RawFrame {
 cv::Mat color_mat;
 cv::Mat depth_mat;
};

#endif //RAWFRAME_H

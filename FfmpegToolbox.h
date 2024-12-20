

#ifndef FFMPEGTOOLBOX_H
#define FFMPEGTOOLBOX_H
#include <opencv2/core/mat.hpp>
#include "AVBufferData.h"

using namespace std;

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/motion_vector.h>
#include <libavutil/imgutils.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

class FfmpegToolbox
{
public:
    static void save_video_buffer_to_file(const std::string& filename);
    static void create_video_from_images(
        const cv::Mat& color_frame,
        const cv::Mat& prev_color_frame,
        int fps, int width, int height
    );

    static int read_packet(void* opaque, uint8_t* buf, int buf_size);

    static vector<AVMotionVector> extract_motion_vectors();

};

inline thread_local std::vector<uint8_t> video_buffer;


#endif //FFMPEGTOOLBOX_H

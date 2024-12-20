

#ifndef IMAGERAWFRAMEPRODUCER_H
#define IMAGERAWFRAMEPRODUCER_H


#include "../custom_types/RawFrame.h"
#include "../custom_types/DepthConfig.h"
#include <tuple>

class ImageRawFrameProducer {
public:
    static RawFrame get_raw_frame(long frame_id, int camera_num);
    static DepthConfig get_depth_config(int camera_num);
    static std::tuple<int, int> get_dimensions();

    static void clean_up();
};



#endif //IMAGERAWFRAMEPRODUCER_H

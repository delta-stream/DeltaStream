

#ifndef MULTICAMERAFRAMEDATA_H
#define MULTICAMERAFRAMEDATA_H

#include <map>

#include "../FrameData.h"
#include "../FrameDataType.h"


struct MultiCameraFrameData {
    FrameDataType type;
    long frame_id;
    std::map<int, FrameData> frame_data_per_camera;
    std::chrono::high_resolution_clock::time_point produce_ended_at;
};



#endif //MULTICAMERAFRAMEDATA_H

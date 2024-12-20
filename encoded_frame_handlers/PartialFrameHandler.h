

#ifndef PARTIALFRAMEHANDLER_H
#define PARTIALFRAMEHANDLER_H


#include <open3d/Open3D.h>

#include "../custom_types/MultiCameraDecodedFrameDTO.h"

using namespace std;

class PartialFrameHandler
{
public:
    static map<int, shared_ptr<open3d::geometry::PointCloud>> handle(
        const MultiCameraDecodedFrameDTO &mc_decoded_frame_dto,
        map<int, shared_ptr<open3d::geometry::PointCloud> > &multi_camera_prev_pc
        );
};


#endif //PARTIALFRAMEHANDLER_H

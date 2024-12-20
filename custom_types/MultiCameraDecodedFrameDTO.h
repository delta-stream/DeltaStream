

#ifndef MULTICAMERADECODEDFRAMEDTO_H
#define MULTICAMERADECODEDFRAMEDTO_H

#include "../DecodedFrameDTO.h"
#include "../FrameDataType.h"
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>

#include <map>


struct MultiCameraDecodedFrameDTO {

    FrameDataType type;
    long frame_id;
    std::map<int, DecodedFrameDTO> frame_data_per_camera;

    MultiCameraDecodedFrameDTO(const FrameDataType type, const long frame_id,
        const std::map<int, DecodedFrameDTO> &frame_data_per_camera)
        : type(type),
          frame_id(frame_id),
          frame_data_per_camera(frame_data_per_camera) {
    }
};

#endif //MULTICAMERADECODEDFRAMEDTO_H

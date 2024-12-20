

#ifndef MULTICAMERAENCODEDFRAMEDTO_H
#define MULTICAMERAENCODEDFRAMEDTO_H

#include "../EncodedFrameDTO.h"
#include "../FrameDataType.h"
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>

#include <map>


struct MultiCameraEncodedFrameDTO {

    FrameDataType type;
    int64_t frame_id;
    std::map<int, EncodedFrameDTO> frame_data_per_camera;

    // Serialization method for Cereal
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(type, frame_id, frame_data_per_camera);
    }
};

#endif //MULTICAMERAENCODEDFRAMEDTO_H

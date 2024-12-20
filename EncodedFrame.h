

#ifndef ENCODEDFRAME_H
#define ENCODEDFRAME_H

#include <draco/core/encoder_buffer.h>
#include <vector>
#include <map>
#include "FrameDataType.h"

using namespace std;

class EncodedFrame
{
public:
    FrameDataType type;
    long frame_id{};
    std::chrono::high_resolution_clock::time_point created_at;
    draco::EncoderBuffer encoder_buffer;
    std::vector<int> block_index_to_remove;
    std::map<int, tuple<double, double, double, int, int>> motion_vector;


    // Default constructor
    EncodedFrame() = default;

    // Full frame type constructor
    EncodedFrame(const long id,
                 draco::EncoderBuffer buffer)
        : frame_id(id), encoder_buffer(move(buffer))
    {
        type = FrameDataType::Full;
    }


    // Partial frame type constructor
    EncodedFrame(const long id,
                 draco::EncoderBuffer& buffer,
                 const std::vector<int>& block_index_to_remove)
        : frame_id(id), encoder_buffer(move(buffer)), block_index_to_remove(block_index_to_remove)
    {
        type = FrameDataType::Partial;
    }

    // Motion vector type constructor
    EncodedFrame(const long id,
                 draco::EncoderBuffer& buffer,
                 const std::vector<int>& block_index_to_remove,
                    const map<int, tuple<double, double, double, int, int>>& motion_vector
                 )
        : frame_id(id), encoder_buffer(move(buffer)), block_index_to_remove(block_index_to_remove), motion_vector(motion_vector)
    {
        type = FrameDataType::MotionVector;
    }
};
#endif //ENCODEDFRAME_H

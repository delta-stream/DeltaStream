

#ifndef ENCODEDFRAMEDTO_H
#define ENCODEDFRAMEDTO_H

// #include <vector>
#include <cstdint>
#include <cstring>
#include <draco/core/encoder_buffer.h>
#include <arpa/inet.h>
#include "FrameDataType.h"
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>


// Helper functions for 64-bit endianness conversion
#if defined(_WIN32)
#include <winsock2.h>
#include <stdlib.h>

uint64_t htobe64(uint64_t host_64bits) {
    return htonll(host_64bits);
}

uint64_t be64toh(uint64_t big_endian_64bits) {
    return ntohll(big_endian_64bits);
}
#else
#include <endian.h>
#endif

using namespace std;

class EncodedFrameDTO
{
public:
    // Fields
    FrameDataType type;
    int64_t frame_id;
    std::chrono::high_resolution_clock::time_point created_at;
    vector<char> encoder_buffer_data; // Serialized data of encoder_buffer
    vector<int32_t> block_index_to_remove;
    map<int32_t, tuple<double,double,double, int32_t, int32_t>> motion_vector;

    // Constructors
    EncodedFrameDTO() = default;


    // Serialization method for Cereal
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(type, frame_id, encoder_buffer_data, block_index_to_remove , motion_vector);
    }
};

#endif //ENCODEDFRAMEDTO_H

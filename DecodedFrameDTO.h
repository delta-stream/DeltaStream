

#ifndef DECODEDFRAMEDTO_H
#define DECODEDFRAMEDTO_H

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
#include <chrono>
#include <open3d/geometry/PointCloud.h>

#endif

using namespace std;

class DecodedFrameDTO
{
public:
    // Fields
    FrameDataType type;
    long frame_id;
    std::chrono::high_resolution_clock::time_point created_at;
    shared_ptr<open3d::geometry::PointCloud> pointcloud;
    vector<int> block_index_to_remove;
    map<int, tuple<double,double,double, int, int>> motion_vector;

    // Constructors
    DecodedFrameDTO() = default;

    DecodedFrameDTO(FrameDataType type, long frame_id,
        const std::chrono::high_resolution_clock::time_point &created_at,
        const shared_ptr<open3d::geometry::PointCloud> &pointcloud,
        const vector<int> &block_index_to_remove,
        const map<int, tuple<double, double, double, int, int>> &motion_vector)
        : type(type),
          frame_id(frame_id),
          created_at(created_at),
          pointcloud(pointcloud),
          block_index_to_remove(block_index_to_remove),
          motion_vector(motion_vector) {
    }
};

#endif //DECODEDFRAMEDTO_H



#ifndef FRAMEDATA_H
#define FRAMEDATA_H
#include <open3d/geometry/PointCloud.h>

#include "FrameDataType.h"
#include <memory>
#include <chrono>

#include "DeltaMotion.h"

class FrameData
{
    // Data members are private by default in a class.
private:
    FrameDataType type;
    long frame_id;
    std::chrono::high_resolution_clock::time_point created_at = std::chrono::high_resolution_clock::now();
    std::shared_ptr<open3d::geometry::PointCloud> pointcloud;
    std::vector<int> block_index_to_remove;
    std::map<int, DeltaMotion> motion_vector;

public:
    // Default constructor
    FrameData() = default;

    // Full frame type constructor
    FrameData(const long id,
              const std::shared_ptr<open3d::geometry::PointCloud>& pc)
        : frame_id(id), pointcloud(pc)
    {
        type = FrameDataType::Full;
    }

    // Partial frame type constructor
    FrameData(const long id,
              const std::shared_ptr<open3d::geometry::PointCloud>& pc_delta,
              const std::vector<int>& block_index_to_remove)
        : frame_id(id), pointcloud(pc_delta), block_index_to_remove(block_index_to_remove)
    {
        type = FrameDataType::Partial;
    }

    // Motion vector type constructor
    FrameData(const long id,
              const std::shared_ptr<open3d::geometry::PointCloud>& pc_delta,
              const std::vector<int>& block_index_to_remove,
              const std::map<int, DeltaMotion>& motion_vector
    )
        : frame_id(id), pointcloud(pc_delta), block_index_to_remove(block_index_to_remove), motion_vector(motion_vector)
    {
        type = FrameDataType::MotionVector;
    }

    void setCreatedAt(std::chrono::high_resolution_clock::time_point ts) { created_at = ts; }

    // Getter methods to access private members
    [[nodiscard]] long getFrameId() const { return frame_id; }
    [[nodiscard]] FrameDataType getType() const { return type; }
    [[nodiscard]] std::vector<int> getBlockIndexToRemove() const { return block_index_to_remove; }
    [[nodiscard]] std::chrono::high_resolution_clock::time_point getCreatedAt() const { return created_at; }
    [[nodiscard]] std::shared_ptr<open3d::geometry::PointCloud> getPointCloud() const { return pointcloud; }
    [[nodiscard]] std::map<int, DeltaMotion> getMotionVector() const { return motion_vector; }
};

#endif //FRAMEDATA_H

// FrameMapManager.h
#ifndef FRAME_MAP_MANAGER_H
#define FRAME_MAP_MANAGER_H

#include <unordered_map>
#include <mutex>
#include <memory>
#include "./custom_types/MultiCameraEncodedFrame.h"

// Forward declaration of EncodedFrame
class EncodedFrame;

// Define the FrameMapManager class
class FrameMapManager {
public:
    // Constructor
    FrameMapManager() = default;

    // Disable copy constructor and copy assignment
    FrameMapManager(const FrameMapManager&) = delete;
    FrameMapManager& operator=(const FrameMapManager&) = delete;

    // Inserts an EncodedFrame into the map
    void InsertFrame(long key, const std::shared_ptr<MultiCameraEncodedFrame> frame);

    // Retrieves an EncodedFrame from the map
    std::shared_ptr<MultiCameraEncodedFrame> GetFrame(long key);

    // Removes an EncodedFrame from the map
    void RemoveFrame(long key);

    // Checks if a frame exists in the map
    bool FrameExists(const long key);

    long GetSize() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return frame_map_.size();
    }
private:
    std::unordered_map<long, std::shared_ptr<MultiCameraEncodedFrame>> frame_map_;
    std::mutex map_mutex_;
};

#endif // FRAME_MAP_MANAGER_H



#ifndef DECODEDFRAMEDTOMANAGER_H
#define DECODEDFRAMEDTOMANAGER_H

#include <unordered_map>
#include <mutex>
#include <memory>
#include "custom_types/MultiCameraDecodedFrameDTO.h"


class DecodedFrameDTOManager {
public:
    // Constructor
    DecodedFrameDTOManager() = default;

    // Disable copy constructor and copy assignment
    DecodedFrameDTOManager(const DecodedFrameDTOManager&) = delete;
    DecodedFrameDTOManager& operator=(const DecodedFrameDTOManager&) = delete;

    // Inserts an EncodedFrame into the map
    void InsertFrame(long key, const std::shared_ptr<MultiCameraDecodedFrameDTO> frame);

    // Retrieves an EncodedFrame from the map
    std::shared_ptr<MultiCameraDecodedFrameDTO> GetFrame(long key);

    // Removes an EncodedFrame from the map
    void RemoveFrame(long key);

    // Checks if a frame exists in the map
    bool FrameExists(long key);

    long GetSize() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return frame_map_.size();
    }
private:
    std::unordered_map<long, std::shared_ptr<MultiCameraDecodedFrameDTO>> frame_map_;
    std::mutex map_mutex_;
};



#endif //DECODEDFRAMEDTOMANAGER_H

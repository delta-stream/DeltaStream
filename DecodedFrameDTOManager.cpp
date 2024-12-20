

#include "DecodedFrameDTOManager.h"


void DecodedFrameDTOManager::InsertFrame(const long key, const std::shared_ptr<MultiCameraDecodedFrameDTO> frame) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    frame_map_[key] = frame;
}

std::shared_ptr<MultiCameraDecodedFrameDTO> DecodedFrameDTOManager::GetFrame(const long key) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (const auto it = frame_map_.find(key); it != frame_map_.end()) {
        return it->second;
    }
    return nullptr; // Key not found
}

void DecodedFrameDTOManager::RemoveFrame(const long key) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    frame_map_.erase(key);
}

bool DecodedFrameDTOManager::FrameExists(const long key) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return frame_map_.contains(key);
}


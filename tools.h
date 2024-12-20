

#ifndef TOOLS_H
#define TOOLS_H
#include <chrono>
#include <iostream>
#include <ostream>

#include <filesystem>

namespace fs = std::filesystem;


inline void print_fps_every_30_frames(
    const string& msg_prefix,
    long &frames,
    std::chrono::time_point<std::chrono::high_resolution_clock> &start_time
) {
    if (frames % 30 == 0) {
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        auto fps = frames / elapsed.count();
        std::cout << msg_prefix << fps << std::endl;

        // Reset for the next calculation
        frames = 0;
        start_time = std::chrono::high_resolution_clock::now();
    }
}


inline std::string formatTimePoint(const std::chrono::high_resolution_clock::time_point timePoint) {
    // Convert time_point to a time_t
    const auto durationSinceEpoch = timePoint.time_since_epoch();
    const auto secondsSinceEpoch = std::chrono::duration_cast<std::chrono::seconds>(durationSinceEpoch);
    const std::time_t timeT = secondsSinceEpoch.count();

    // Format to HH:mm:ss
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&timeT), "%H:%M:%S");

    // Add milliseconds
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(durationSinceEpoch) % 1000;
    oss << "." << std::setfill('0') << std::setw(3) << milliseconds.count();

    return oss.str();
}

inline uint64_t timeSinceEpochMillisec() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}



#endif //TOOLS_H
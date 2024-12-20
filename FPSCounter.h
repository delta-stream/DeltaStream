

#ifndef FPSCOUNTER_H
#define FPSCOUNTER_H
#include <numeric>


class FPSCounter {
private:
    std::vector<double> frameTimes; // Store frame durations
    size_t maxFrames; // Sliding window size
    size_t index; // Current index for circular buffer
    bool filled; // Whether the buffer is filled

public:
    FPSCounter(const size_t windowSize = 60) : maxFrames(windowSize), index(0), filled(false) {
        frameTimes.resize(windowSize, 0.0);
    }

    void update(const double deltaTime) {
        frameTimes[index] = deltaTime; // Store the frame time
        index = (index + 1) % maxFrames;

        if (index == 0) filled = true; // Buffer filled once wrapped
    }

    double getSmoothedFPS() const {
        const size_t count = filled ? maxFrames : index; // Use only filled frames
        const double totalTime = std::accumulate(frameTimes.begin(), frameTimes.begin() + count, 0.0);
        return totalTime > 0.0 ? count / totalTime : 0.0; // FPS = Frames / Total Time
    }
};



#endif //FPSCOUNTER_H

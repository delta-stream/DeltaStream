

#ifndef MOVINGAVERAGE_H
#define MOVINGAVERAGE_H
#include <queue>


class MovingAverage {
private:
    std::queue<double> window;
    int windowSize;
    double sum;

public:
    // Constructor
    MovingAverage(const double size) : windowSize(size), sum(0.0) {}

    // Add a new value and return the moving average
    double next(const double value) {
        sum += value;
        window.push(value);

        // If window size exceeds the limit, remove the oldest value
        if (window.size() > windowSize) {
            sum -= window.front();
            window.pop();
        }

        // Calculate and return the average
        return sum / window.size();
    }

};



#endif //MOVINGAVERAGE_H

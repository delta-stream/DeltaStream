

#include "MVFrameLatencyReport.h"

void MVFrameLatencyReport::push(long frame_id, long mv_latency,
    long decode_latency) {
    std::lock_guard<std::mutex> lock(reports_mutex_);
    latency_reports_.push_back(std::make_tuple(frame_id, mv_latency,
        decode_latency));
}

void MVFrameLatencyReport::clear() {
    std::lock_guard<std::mutex> lock(reports_mutex_);
    latency_reports_.clear();
}

std::vector<std::tuple<long, long, long>> MVFrameLatencyReport::get_reports() {
    std::lock_guard<std::mutex> lock(reports_mutex_);
    return latency_reports_;
}

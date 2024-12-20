

#ifndef MVFRAMELATENCYREPORT_H
#define MVFRAMELATENCYREPORT_H
#include <mutex>
#include <vector>


class MVFrameLatencyReport {
private:
    std::vector<std::tuple<long, long, long>> latency_reports_;
    std::mutex reports_mutex_;
public:
    MVFrameLatencyReport() = default;

    void push(long frame_id, long mv_latency, long decode_latency);
    void clear();
    std::vector<std::tuple<long, long, long>> get_reports();
};



#endif //MVFRAMELATENCYREPORT_H

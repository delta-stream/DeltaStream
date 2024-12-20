

#ifndef BENCHMARK_H
#define BENCHMARK_H
#include <chrono>
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include "FrameDataType.h"


struct ServerBenchmark
{
    long frame_id = -1;
    FrameDataType frame_type = UNKNOWN;
    long frame_created_at = -1;
    long pc_produce_ms = -1;
    long draco_encode_ms = -1;
    long serialize_ms = -1;
    long tcp_send_ms = -1;
    long frame_sent_at = -1;
    long total_server_processing_time_ms = -1;
    long total_payload_bytes = -1;
    long pc_payload_bytes = -1;

    ServerBenchmark() = default;

    explicit ServerBenchmark(const long frame_id): frame_id(frame_id){};

};

struct ClientBenchmark
{
    long frame_id;
    long receive_ms = -1; // time took to receive the frame
    long received_at = -1;
    long deserialize_ms = -1;
    long decode_ms = -1;
    long pre_render_ms = -1;
    long render_ms = -1;
    long rendered_at = -1;
    double fps = -1.0;

    ClientBenchmark() = default;

    explicit ClientBenchmark(const long frame_id)
        : frame_id(frame_id) {
    }
};



#endif //BENCHMARK_H

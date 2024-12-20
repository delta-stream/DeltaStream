

#ifndef SERVERBENCHMARKMAP_H
#define SERVERBENCHMARKMAP_H
#include <map>
#include <mutex>

#include "Benchmark.h"
#include "FrameDataType.h"

class ServerBenchmarkMap
{
private:
    std::map<long, ServerBenchmark> benchmark_map;
    std::mutex map_mutex;

public:
    void insert(long key, const ServerBenchmark& value)
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        benchmark_map[key] = value;
    }

    bool find(long key, ServerBenchmark& result)
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        auto it = benchmark_map.find(key);
        if (it != benchmark_map.end())
        {
            result = it->second;
            return true;
        }
        return false;
    }

    void erase(long key)
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        benchmark_map.erase(key);
    }

    void mergeRawFrames(const std::map<long, ServerBenchmark>& other)
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        for (const auto& [frame_id, other_benchmark] : other)
        {
            benchmark_map[frame_id].frame_id = other_benchmark.frame_id;
            benchmark_map[frame_id].frame_created_at = other_benchmark.frame_created_at;
        }
    }

    void mergeProduceFrames(const std::map<long, ServerBenchmark>& other)
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        for (const auto& [frame_id, other_benchmark] : other)
        {
            benchmark_map[frame_id].frame_type = other_benchmark.frame_type;
            benchmark_map[frame_id].pc_produce_ms = other_benchmark.pc_produce_ms;
        }
    }

    void mergeEncodeFrames(const std::map<long, ServerBenchmark>& other)
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        for (const auto& [frame_id, other_benchmark] : other)
        {
            benchmark_map[frame_id].draco_encode_ms = other_benchmark.draco_encode_ms;
        }
    }
    void mergeSerializeFrames(const std::map<long, ServerBenchmark>& other)
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        for (const auto& [frame_id, other_benchmark] : other)
        {
            benchmark_map[frame_id].pc_payload_bytes = other_benchmark.pc_payload_bytes;
            benchmark_map[frame_id].serialize_ms = other_benchmark.serialize_ms;
        }
    }
    void mergeTCPSend(const std::map<long, ServerBenchmark>& other)
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        for (const auto& [frame_id, other_benchmark] : other)
        {
            benchmark_map[frame_id].tcp_send_ms = other_benchmark.tcp_send_ms;
            benchmark_map[frame_id].frame_sent_at = other_benchmark.frame_sent_at;
            benchmark_map[frame_id].total_payload_bytes = other_benchmark.total_payload_bytes;
            benchmark_map[frame_id].total_server_processing_time_ms = other_benchmark.total_server_processing_time_ms;
        }
    }

    void saveAsCsv(const std::string &filename) {
        std::ofstream csv_file(filename);

        // Check if the file is open
        if (!csv_file.is_open())
        {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }

        // Write the header row
        csv_file << "frame_id,frame_type,frame_created_at,pc_produce_ms,"
                << "draco_encode_ms,serialize_ms,tcp_send_ms,frame_sent_at,"
                << "pc_payload_bytes,total_payload_bytes\n";

        // Write each ServerBenchmark to the file
        for (const auto& [frame_id, benchmark] : benchmark_map)
        {
            auto frame_type_str = benchmark.frame_type == Full ? "Full" :
                benchmark.frame_type == Partial ? "Partial" : "MotionVector";
            csv_file
                << benchmark.frame_id << ","
                << frame_type_str << ","
                << benchmark.frame_created_at << ","
                << benchmark.pc_produce_ms << ","
                << benchmark.draco_encode_ms << ","
                << benchmark.serialize_ms << ","
                << benchmark.tcp_send_ms << ","
                << benchmark.frame_sent_at << ","
                << benchmark.pc_payload_bytes << ","
                << benchmark.total_payload_bytes << "\n";
        }

        // Close the file
        csv_file.close();
    }
};



#endif //SERVERBENCHMARKMAP_H

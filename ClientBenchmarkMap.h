

#ifndef CLIENTBENCHMARKMAP_H
#define CLIENTBENCHMARKMAP_H
#include <map>
#include <mutex>

#include "Benchmark.h"


class ClientBenchmarkMap {

private:
    std::map<long, ClientBenchmark> benchmark_map;
    std::mutex map_mutex;

public:

    void mergeReceiveFrames(const std::map<long, ClientBenchmark>& other)
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        for (const auto& [frame_id, other_benchmark] : other)
        {
            benchmark_map[frame_id].frame_id = other_benchmark.frame_id;
            benchmark_map[frame_id].receive_ms = other_benchmark.receive_ms;
            benchmark_map[frame_id].received_at = other_benchmark.received_at;
            benchmark_map[frame_id].deserialize_ms = other_benchmark.deserialize_ms;
        }
    }

    void mergeDecodePC(const std::map<long, ClientBenchmark>& other)
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        for (const auto& [frame_id, other_benchmark] : other)
        {
            benchmark_map[frame_id].decode_ms = other_benchmark.decode_ms;
        }
    }

    void mergeRender(const std::map<long, ClientBenchmark>& other)
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        for (const auto& [frame_id, other_benchmark] : other)
        {
            benchmark_map[frame_id].render_ms = other_benchmark.render_ms;
            benchmark_map[frame_id].rendered_at = other_benchmark.rendered_at;
            benchmark_map[frame_id].fps = other_benchmark.fps;
        }
    }
    void mergePreRender(const std::map<long, ClientBenchmark>& other)
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        for (const auto& [frame_id, other_benchmark] : other)
        {
            benchmark_map[frame_id].pre_render_ms = other_benchmark.pre_render_ms;
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
        csv_file << "frame_id,receive_ms,received_at,deserialize_ms,decode_ms,pre_render_ms,render_ms,rendered_at,fps\n";

        // Write each ServerBenchmark to the file
        for (const auto& [frame_id, benchmark] : benchmark_map)
        {
            csv_file
                << benchmark.frame_id << ","
                << benchmark.receive_ms << ","
                << benchmark.received_at << ","
                << benchmark.deserialize_ms << ","
                << benchmark.decode_ms << ","
                << benchmark.pre_render_ms << ","
                << benchmark.render_ms << ","
                << benchmark.rendered_at << ","
                << benchmark.fps << "\n";
        }

        // Close the file
        csv_file.close();
    }
};



#endif //CLIENTBENCHMARKMAP_H

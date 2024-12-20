#include <librealsense2/rs.hpp>      // Include RealSense Cross Platform API
#include <open3d/Open3D.h>           // Include Open3D
#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <boost/asio.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind/bind.hpp>
#include <boost/lockfree/queue.hpp>
#include <draco/compression/decode.h>
#include <draco/core/decoder_buffer.h>
#include <fstream>
#include <queue>
#include <draco/io/file_utils.h>

#include "Benchmark.h"
#include "ServerBenchmarkMap.h"
#include "draco.h"
#include "FrameDataType.h"
#include "EncodedFrame.h"
#include "EncodedFrameDTO.h"
#include "FrameData.h"
#include "FrameMapManager.h"
#include "MovingAverage.hpp"
#include "realsense.h"
#include "tools.h"
#include "custom_types/ClientFeedback.h"
#include "custom_types/MultiCameraEncodedFrameDTO.h"
#include "custom_types/MultiCameraFrameData.h"
#include "custom_types/MVFramePrepData.h"
#include "frame_preppers/BlockUpdatePrepper.h"
#include "frame_preppers/MotionVectorPrepper.h"
#include "raw_frame_producers/ImageRawFrameProducer.h"
#include "raw_frame_producers/RealSenseRawFrameProducer.h"

extern "C" {
#include <libavutil/log.h>
}

using namespace std;

namespace o3d = open3d;
using boost::asio::ip::tcp;

// #pragma GCC optimize("O0")

std::atomic<long> encoded_frames = 0;

queue<MultiCameraFrameData> open3d_pc_queue;
std::mutex queue_mutex;
std::condition_variable queue_cv;
queue<tuple<int, promise<pair<int, shared_ptr<FrameData>>>, shared_ptr<MVFramePrepData>>> mv_prep_queue;
std::mutex mv_prep_queue_mutex;
std::condition_variable mv_prep_queue_cv;
boost::asio::thread_pool pool(4);

queue<tuple<long, vector<pair<cv::Mat, cv::Mat>>>> raw_frame_queue;
std::mutex raw_frame_queue_mutex;
std::condition_variable raw_frame_queue_cv;

queue<MultiCameraEncodedFrame> encoded_frame_queue;
std::mutex encoded_frame_queue_mutex;
std::condition_variable encoded_frame_queue_cv;

queue<pair<int, vector<char>>> serialized_dto_queue;
std::mutex serialized_dto_queue_mutex;
std::condition_variable serialized_dto_queue_cv;



// Global atomic flag to indicate when to stop the server
std::atomic<bool> stop_server(false);

// Signal handler for Control-C
void signal_handler(const int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        cout << "SIGINT/SIGTERM received. Stopping server..." << endl;
        stop_server.store(true);
    }
}

void enqueue_raw_frames(map<int, RealSenseRawFrameProducer> &per_cam_rs_raw_frame_producer, ServerBenchmarkMap &benchmark_map) {
    long frame_id = 0;
    map<long, ServerBenchmark> benchmarks;
    while (!stop_server) {
        auto st = std::chrono::high_resolution_clock::now();

        vector<pair<cv::Mat,cv::Mat>> per_cam_images;
        for (int camera_num=0; camera_num< num_cameras;camera_num++) {
            auto [color_mat, depth_mat] = img_mode
                                          ? ImageRawFrameProducer::get_raw_frame(
                                              frame_id, camera_num
                                          )
                                          : per_cam_rs_raw_frame_producer[camera_num].
                                          get_raw_frame(camera_num);
            per_cam_images.push_back(make_pair(color_mat, depth_mat));
        }
        long raw_frame_queue_size;

        benchmarks[frame_id] = ServerBenchmark(frame_id);
        benchmarks[frame_id].frame_created_at = timeSinceEpochMillisec();

        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            raw_frame_queue.push(make_tuple(frame_id, per_cam_images));
            raw_frame_queue_size = raw_frame_queue.size();
        }
        raw_frame_queue_cv.notify_one();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - st).count();

        // cout << "[F" << frame_id << "] Enqueued raw frame in " << duration_ms << "ms @ " << formatTimePoint(end) << endl;
        // cout << "Raw frame queue size: " << raw_frame_queue_size << endl;

        frame_id += 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(33- duration_ms)); // Sleep for 33ms
    }
    benchmark_map.mergeRawFrames(benchmarks);
}

void produce_frames_to_encode(ServerBenchmarkMap &benchmark_map, atomic<int> &max_mv_blocks) {
    map<int, DepthConfig> per_cam_depth_config;
    map<int, open3d::camera::PinholeCameraIntrinsic> per_cam_camera_intrinsics;
    map<long, ServerBenchmark> benchmarks;
    map<int, RealSenseRawFrameProducer> rs_per_cam_producer;
    map<int, string> rs_per_cam_serial;

    if (!img_mode) {
        auto ctx = rs2::context();
        int camera_num = 0;
        for (auto device : ctx.query_devices()) {
            rs_per_cam_serial[camera_num] = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            cout << "Found device with serial: " << rs_per_cam_serial[camera_num] << endl;
            camera_num++;
        }
    }

    for (int i = 0; i < num_cameras; ++i) {
        if (!img_mode) {
            RealSenseRawFrameProducer real_sense_raw_frame_producer;
            real_sense_raw_frame_producer.setup_realsense_pipeline(rs_per_cam_serial[i]);
            rs_per_cam_producer[i] = real_sense_raw_frame_producer;
        }
        per_cam_depth_config[i] = img_mode? getCamDepthConfig(i) : rs_per_cam_producer[i].get_depth_config();

        per_cam_camera_intrinsics[i] = open3d::camera::PinholeCameraIntrinsic(
            frame_width, frame_height,
            per_cam_depth_config[i].fx, per_cam_depth_config[i].fy,
            per_cam_depth_config[i].cx, per_cam_depth_config[i].cy
        );
    }



    long frames = 0;
    auto start_time = std::chrono::high_resolution_clock::now(); // Start time

    // Variables to hold previous frames
    map<int, cv::Mat> per_cam_prev_color_mat;
    map<int, cv::Mat> per_cam_prev_depth_mat;

    thread raw_frame_enqueue_thread(enqueue_raw_frames, ref(rs_per_cam_producer), ref(benchmark_map));


    while (!stop_server.load()) {
        frames += 1;
        try {
            std::unique_lock<std::mutex> lock(raw_frame_queue_mutex);

            if (!raw_frame_queue_cv.wait_for(lock, std::chrono::seconds(1),
                                   [] { return !raw_frame_queue.empty(); }
            )) {
                // Timeout occurred, continue the loop
                lock.unlock();
                continue;
            }

            auto raw_frame = raw_frame_queue.front();
            raw_frame_queue.pop();
            lock.unlock();

            auto frame_id = get<0>(raw_frame);
            auto per_cam_images = get<1>(raw_frame);
            // cv::Mat depth_mat = get<2>(raw_frame);
            benchmarks[frame_id] = ServerBenchmark(frame_id);


            bool is_keyframe = frame_id % keyframe_frequency == 0;
            const chrono::high_resolution_clock::time_point frame_ts =
                    std::chrono::high_resolution_clock::now();
            const auto produce_frame_start = chrono::high_resolution_clock::now();
            long open3d_pc_queue_size;


            if (is_keyframe) {
                map<int, FrameData> mc_frame_point_clouds;
                benchmarks[frame_id].frame_type = Full;

                int i = 0;
                for (auto [color_mat, depth_mat] : per_cam_images) {
                    const auto depth_config = per_cam_depth_config[i];
                    const auto color_image = std::make_shared<
                    o3d::geometry::Image>();
                    color_image->Prepare(frame_width, frame_height, color_mat.channels(),
                                         color_mat.elemSize1()
                    );
                    memcpy(color_image->data_.data(), color_mat.data,
                           color_mat.total() * color_mat.elemSize()
                    );

                    const auto depth_image = std::make_shared<
                        o3d::geometry::Image>();
                    depth_image->Prepare(frame_width, frame_height, depth_mat.channels(),
                                         depth_mat.elemSize1()
                    );
                    memcpy(depth_image->data_.data(), depth_mat.data,
                           depth_mat.total() * depth_mat.elemSize()
                    );

                    auto rgbd_image =
                            open3d::geometry::RGBDImage::CreateFromColorAndDepth(
                                *color_image,
                                *depth_image,
                                1.0 / depth_config.depth_scale,
                                3,
                                false
                            );

                    const auto camera_intrinsics = per_cam_camera_intrinsics[i];
                    auto cam_point_cloud =
                            open3d::geometry::PointCloud::CreateFromRGBDImage(
                                *rgbd_image, camera_intrinsics
                            );
                    // cam_point_cloud = cam_point_cloud->UniformDownSample(2);
                    mc_frame_point_clouds.insert({i, FrameData(frame_id, cam_point_cloud)});
                    i++;
                }


                {
                    MultiCameraFrameData mc_frame_data{
                        .type = Full, .frame_id = frame_id,
                        .frame_data_per_camera = mc_frame_point_clouds,
                        .produce_ended_at = std::chrono::high_resolution_clock::now()
                    };


                    std::lock_guard<std::mutex> lock(queue_mutex);
                    open3d_pc_queue.push(mc_frame_data);
                    open3d_pc_queue_size = open3d_pc_queue.size();
                }
                queue_cv.notify_one();


            } else if (use_motion_vector) {

                benchmarks[frame_id].frame_type = MotionVector;
                MultiCameraFrameData mc_frame_data{
                    .type = MotionVector, .frame_id = frame_id,
                    .frame_data_per_camera = {},
                    .produce_ended_at = std::chrono::high_resolution_clock::now()
                };
                vector<future<pair<int, shared_ptr<FrameData>>>> futures;

                auto non_atomic_max_mv_blocks = max_mv_blocks.load();

                for (int i = 0; i < num_cameras; ++i) {
                    auto [color_mat, depth_mat] = per_cam_images[i];
                    auto prev_color_mat = per_cam_prev_color_mat[i];
                    auto prev_depth_mat = per_cam_prev_depth_mat[i];

                    const auto depth_config = per_cam_depth_config[i];
                    futures.push_back(async(launch::async, [i, color_mat, depth_mat, prev_color_mat, prev_depth_mat, frame_id, depth_config, frame_ts, non_atomic_max_mv_blocks]() {
                        return make_pair(i, make_shared<FrameData>(
                            MotionVectorPrepper::prepare(
                                                        color_mat, depth_mat,
                                                        prev_color_mat, prev_depth_mat,
                                                        frame_id, depth_config, frame_ts, non_atomic_max_mv_blocks
                                                    )
                        ));
                    }));
                }

                for (auto& future : futures) {
                    auto [camera_num, mv_processed_pc] = future.get();
                    mc_frame_data.frame_data_per_camera[camera_num] = *mv_processed_pc;
                }

                {
                    std::lock_guard<std::mutex> lock(queue_mutex);
                    open3d_pc_queue.push(mc_frame_data);
                    open3d_pc_queue_size = open3d_pc_queue.size();
                }
                queue_cv.notify_one();
            } else {
                benchmarks[frame_id].frame_type = Partial;
                map<int, FrameData> mc_frame_point_clouds;
                for (int i = 0; i < num_cameras; ++i) {
                    auto [color_mat, depth_mat] = per_cam_images[i];
                    auto prev_color_mat = per_cam_prev_color_mat[i];
                    auto prev_depth_mat = per_cam_prev_depth_mat[i];
                    auto depth_config = per_cam_depth_config[i];

                    mc_frame_point_clouds.insert({i, BlockUpdatePrepper::prepare(
                        color_mat, depth_mat,
                        prev_color_mat, prev_depth_mat,
                        frame_id, depth_config
                    )});
                }

                MultiCameraFrameData mc_frame_data{
                    .type = Partial, .frame_id = frame_id,
                    .frame_data_per_camera = mc_frame_point_clouds,
                    .produce_ended_at = std::chrono::high_resolution_clock::now()
                };
                {
                    std::lock_guard<std::mutex> lock(queue_mutex);
                    open3d_pc_queue.push(mc_frame_data);
                    open3d_pc_queue_size = open3d_pc_queue.size();
                }
                queue_cv.notify_one();
            }


            for (int i=0; i<per_cam_images.size(); i++) {
                per_cam_prev_color_mat[i] = per_cam_images[i].first;
                per_cam_prev_depth_mat[i] = per_cam_images[i].second;
            }


            auto produce_frame_end = chrono::high_resolution_clock::now();
            auto duration_ms = chrono::duration_cast<chrono::milliseconds>(produce_frame_end - produce_frame_start).count();

            benchmarks[frame_id].pc_produce_ms = duration_ms;
            // cout << "[F" << frame_id << "] produce_frame: " << duration_ms << "ms" << endl;
            // cout << "open3d_pc_queue size: " << open3d_pc_queue_size << endl;

        } catch (std::exception &e) {
            cerr << "[Frame prepper] Exception: " << e.what() << endl;
        }
        // print_fps_every_30_frames("Producer FPS: ", frames, start_time);
    }

    if (!img_mode) {
        for (int i=0; i< num_cameras; i++)
            rs_per_cam_producer[i].clean_up();
    }

    raw_frame_enqueue_thread.join();

    benchmark_map.mergeProduceFrames(benchmarks);
    cout << "Frame prepper thread stopped." << endl;
}


size_t GetPointCloudSize(const shared_ptr<open3d::geometry::PointCloud> &point_cloud) {
    size_t size = 0;

    // Points
    size += point_cloud->points_.size() * sizeof(Eigen::Vector3d); // Each point is a 3D vector (double)

    // Normals (if present)
    if (!point_cloud->normals_.empty()) {
        size += point_cloud->normals_.size() * sizeof(Eigen::Vector3d);
    }

    // Colors (if present)
    if (!point_cloud->colors_.empty()) {
        size += point_cloud->colors_.size() * sizeof(Eigen::Vector3d);
    }

    // Add additional attributes here if needed

    return size;
}

void encode_draco_frames(FrameMapManager &frame_manager, ServerBenchmarkMap &benchmark_map) {
    map<long, ServerBenchmark> benchmarks;
    while (!stop_server.load()) {
        try {
            std::unique_lock<std::mutex> lock(queue_mutex);

            if (!queue_cv.wait_for(lock, std::chrono::seconds(1),
                                   [] { return !open3d_pc_queue.empty(); }
            )) {
                // Timeout occurred, continue the loop
                lock.unlock();
                continue;
            }

            const auto encode_start = std::chrono::high_resolution_clock::now();
            auto mc_frame_data = open3d_pc_queue.front();
            open3d_pc_queue.pop();
            lock.unlock();

            auto frame_id = mc_frame_data.frame_id;
            benchmarks[frame_id] = ServerBenchmark(frame_id);

            const auto wait_btw_produce_and_encode = chrono::duration_cast<chrono::milliseconds>(encode_start - mc_frame_data.produce_ended_at).count();

            MultiCameraEncodedFrame mc_encoded_frame{ .type = mc_frame_data.type, .frame_id = mc_frame_data.frame_id };

            vector<future<pair<int, shared_ptr<EncodedFrame>>>> futures;

            long pc_size = 0;
            bool found_empty_pc = false;

            open3d::io::WritePointCloudOption options;
            options.write_ascii = open3d::io::WritePointCloudOption::IsAscii::Ascii; // Ensure ASCII format
            for (const auto& [camera_num, frame_data] : mc_frame_data.frame_data_per_camera) {
                pc_size += GetPointCloudSize(frame_data.getPointCloud());
                if (frame_data.getPointCloud()->IsEmpty()) {
                    found_empty_pc = true;
                    break;
                }

                futures.push_back(async(launch::async, [camera_num, frame_data]() {
                    const auto frame_id = frame_data.getFrameId();
                    const auto open3d_frame_point_cloud = frame_data.getPointCloud();

                    const auto draco_frame_point_cloud =
                            convert_open3d_pc_to_draco_pc(open3d_frame_point_cloud);
                    auto draco_encoded_frame_point_cloud =
                            encode_point_cloud(*draco_frame_point_cloud);

                    const auto encoded_frame = make_shared<EncodedFrame>();
                    encoded_frame->type = frame_data.getType();
                    encoded_frame->frame_id = frame_id;
                    encoded_frame->created_at = frame_data.getCreatedAt();
                    encoded_frame->encoder_buffer = move(draco_encoded_frame_point_cloud);

                    if (frame_data.getType() == Partial) {
                        encoded_frame->block_index_to_remove = frame_data.
                                getBlockIndexToRemove();
                    } else if (frame_data.getType() == MotionVector) {
                        encoded_frame->block_index_to_remove = frame_data.
                                getBlockIndexToRemove();

                        for (const auto &[block_index, delta_motion]: frame_data.
                             getMotionVector()) {
                            encoded_frame->motion_vector[block_index] = make_tuple(
                                delta_motion.delta_x, delta_motion.delta_y,
                                delta_motion.delta_z, delta_motion.src_x,
                                delta_motion.src_y
                            );
                             }
                    }

                    return make_pair(camera_num, encoded_frame);
                }));
            }

            // if (found_empty_pc) {
            //     cout << "Found empty PC in Frame " << to_string(frame_id) << endl;
            //     continue;
            // }

            long draco_pc_size = 0;
            // Collect results from futures
            for (auto& future : futures) {
                auto [camera_num, encoded_frame] = future.get();
                mc_encoded_frame.frame_data_per_camera[camera_num] = encoded_frame;
                draco_pc_size += encoded_frame->encoder_buffer.size();
            }

            {
                std::lock_guard<std::mutex> encoded_frame_queue_lock(encoded_frame_queue_mutex);
                encoded_frame_queue.push(mc_encoded_frame);
            }
            encoded_frame_queue_cv.notify_one();

            ++encoded_frames;

            const auto encode_end = std::chrono::high_resolution_clock::now();
            const auto encode_duration_ms = chrono::duration_cast<std::chrono::milliseconds>(encode_end - encode_start).count();
            // cout << "[F" << mc_encoded_frame.frame_id << "] draco_encode: " << encode_duration_ms << "ms" << endl;
            benchmarks[mc_encoded_frame.frame_id].draco_encode_ms = encode_duration_ms;


            auto frame_created_at = std::chrono::high_resolution_clock::now();
            if (mc_encoded_frame.frame_data_per_camera.size()>0) {
                frame_created_at = mc_encoded_frame.frame_data_per_camera.at(0)->created_at;
            }
            auto frame_sent_at = std::chrono::high_resolution_clock::now();
            const auto frame_latency = chrono::duration_cast<std::chrono::milliseconds>(frame_sent_at - frame_created_at).count();

        }
        catch (runtime_error &e) {
            cerr << "Error encoding: " << e.what() << endl;
        }
    }

    benchmark_map.mergeEncodeFrames(benchmarks);
    cout << "Frame encoder thread stopped." << endl;
}


void serialize_dto(ServerBenchmarkMap &benchmark_map) {
    auto frames = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    map<long, ServerBenchmark> benchmarks;

    while (!stop_server.load()) {
        try {
            std::unique_lock<std::mutex> lock(encoded_frame_queue_mutex);

            if (!encoded_frame_queue_cv.wait_for(lock, std::chrono::seconds(1),
                                   [] { return !encoded_frame_queue.empty(); }
            )) {
                // Timeout occurred, continue the loop
                lock.unlock();
                continue;
            }

            auto mc_encoded_frame = encoded_frame_queue.front();
            encoded_frame_queue.pop();
            lock.unlock();

            auto start_serialize = std::chrono::high_resolution_clock::now();
            auto frame_id = mc_encoded_frame.frame_id;
            benchmarks[frame_id] = ServerBenchmark(frame_id);

            MultiCameraEncodedFrameDTO mc_dto{
                .type = mc_encoded_frame.type,
                .frame_id = mc_encoded_frame.frame_id,
                .frame_data_per_camera = {}
            };
            benchmarks[frame_id].pc_payload_bytes = 0;
            for (const auto& [camera_num, encoded_frame] : mc_encoded_frame.frame_data_per_camera) {
                auto dto = EncodedFrameDTO();
                dto.type = encoded_frame->type;
                dto.frame_id = encoded_frame->frame_id;
                dto.created_at = encoded_frame->created_at;

                // Get the data from encoder_buffer
                const char *data = encoded_frame->encoder_buffer.data();
                size_t pc_payload_size = encoded_frame->encoder_buffer.size();
                dto.encoder_buffer_data.assign(data, data + pc_payload_size);


                dto.block_index_to_remove = encoded_frame->block_index_to_remove;

                dto.motion_vector = encoded_frame->motion_vector;

                mc_dto.frame_data_per_camera[camera_num] = dto;


                benchmarks[frame_id].pc_payload_bytes += pc_payload_size;
            }

            std::stringstream ss; {
                cereal::BinaryOutputArchive oarchive(ss);
                oarchive(mc_dto);
            }
            // Now ss.str() contains the serialized data
            std::string serialized_str = ss.str();
            auto total_payload_size = serialized_str.size() * sizeof(char);


            {
                std::lock_guard<std::mutex> serialized_dto_queue_lock(serialized_dto_queue_mutex);
                serialized_dto_queue.emplace(make_pair(frame_id, vector<char>(serialized_str.begin(), serialized_str.end())));
            }
            serialized_dto_queue_cv.notify_one();

            auto end_serialize = std::chrono::high_resolution_clock::now();
            auto serialize_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_serialize - start_serialize).count();
            benchmarks[frame_id].serialize_ms = serialize_duration;

            // cout << "[F" << frame_id << "] Serialize: " << serialize_duration << "ms" << endl;
        }
        catch (runtime_error& e) {
            cerr << "Error in serializing: " << e.what() << endl;
        }
    }
    benchmark_map.mergeSerializeFrames(benchmarks);
}

void send_to_client(tcp::socket &socket, boost::asio::io_context::strand &strand, ServerBenchmarkMap &benchmark_map) {
    auto frames = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    map<long, ServerBenchmark> benchmarks;

    while (!stop_server.load()) {
        try{
            std::unique_lock<std::mutex> lock(serialized_dto_queue_mutex);

            if (!serialized_dto_queue_cv.wait_for(lock, std::chrono::seconds(1),
                                   [] { return !serialized_dto_queue.empty(); }
            )) {
                // Timeout occurred, continue the loop
                lock.unlock();
                continue;
            }

            auto [frame_id, serialized_data] = serialized_dto_queue.front();
            serialized_dto_queue.pop();
            lock.unlock();

            benchmarks[frame_id] = ServerBenchmark(frame_id);

            auto start_tcp_send = std::chrono::high_resolution_clock::now();
            boost::system::error_code ec;

            // Send the size of the data first
            auto total_size = static_cast<uint32_t>(serialized_data.size());
            uint32_t total_size_network = htonl(total_size);
            std::vector<boost::asio::const_buffer> buffers;
            buffers.emplace_back(boost::asio::buffer(
                    &total_size_network, sizeof(total_size_network)
                )
            );
            buffers.emplace_back(boost::asio::buffer(serialized_data));

            // Send the data
            boost::asio::write(socket, buffers, ec);

            auto end_tcp_send = std::chrono::high_resolution_clock::now();
            auto tcp_send_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_tcp_send - start_tcp_send).count();
            auto bandwidth_bps = total_size * 8 / (tcp_send_duration / 1000.0);
            // cout << "[F" << frame_id << "] tcp_send: " << tcp_send_duration << "ms, " << total_size / 1000.0 << " KB => " <<

            frames += 1;

            benchmarks[frame_id].tcp_send_ms = tcp_send_duration;
            auto end_tcp_send_epoch_ms = chrono::duration_cast<std::chrono::milliseconds>(end_tcp_send.time_since_epoch()).count();
            benchmarks[frame_id].frame_sent_at = end_tcp_send_epoch_ms;
            benchmarks[frame_id].total_payload_bytes = total_size;

            benchmarks[frame_id].total_server_processing_time_ms = end_tcp_send_epoch_ms - benchmarks[frame_id].frame_created_at;

            //benchmarking
            auto encoded_frames_loaded = encoded_frames.load();

            if (encoded_frames_loaded % 30 == 0) {
                auto end_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed =
                        end_time - start_time;
                auto encoder_fps = encoded_frames_loaded / elapsed.count();
                auto total_fps = frames / elapsed.count();

                std::cout << "Server FPS: " << total_fps << std::endl;

                encoded_frames.store(0);
                frames = 0;
                start_time = std::chrono::high_resolution_clock::now();
            }
        }
        catch (runtime_error& e) {
            cerr << "Error while sending: " << e.what() << endl;
        }


    }
    benchmark_map.mergeTCPSend(benchmarks);
}

void receive_client_feedback(tcp::socket &socket, atomic<int> &max_mv_blocks) {
    auto seconds_btw_max_mv_blocks_adjust = 10;
    auto next_max_mv_blocks_adjust_time = std::chrono::high_resolution_clock::now() + std::chrono::seconds(seconds_btw_max_mv_blocks_adjust);
    MovingAverage client_fps_moving_avg(30);

    while (!stop_server.load()) {
        try {
            boost::system::error_code ec;
            uint32_t total_size_network = 0;
            boost::asio::read(socket, boost::asio::buffer(&total_size_network, sizeof(total_size_network)), ec);
            uint32_t total_size = ntohl(total_size_network);

            if (ec)
            {
                std::cerr << "Error reading length prefix: " << ec.message() << std::endl;
                break;
            }

            // Read the serialized data
            std::vector<char> buffer(total_size);
            boost::asio::read(socket, boost::asio::buffer(buffer.data(), total_size), ec);

            if (ec)
            {
                std::cerr << "Error during binary data receive: " << ec.message() << std::endl;
                break;
            }
            ClientFeedback dto;
            std::string serialized_str(buffer.begin(), buffer.end());

            if (serialized_str.empty())
            {
                std::cerr << "Error: received empty data" << std::endl;
                break;
            }

            // Create a std::istringstream from the string
            std::istringstream is(serialized_str, std::ios::binary); {
                cereal::BinaryInputArchive iarchive(is);
                iarchive(dto);
            }

            auto prev_max_mv_blocks = max_mv_blocks.load();
            if (prev_max_mv_blocks > 0 &&
                dto.fps < 28 &&
                std::chrono::high_resolution_clock::now() > next_max_mv_blocks_adjust_time
                ) {
                max_mv_blocks.store(prev_max_mv_blocks * 0.8);
                next_max_mv_blocks_adjust_time = std::chrono::high_resolution_clock::now() + std::chrono::seconds(seconds_btw_max_mv_blocks_adjust);
                cout << "Client FPS (" << dto.fps << ") is less than 28. max MV blocks:" << prev_max_mv_blocks << " -> " << max_mv_blocks.load() << endl;
            }

        }
        catch (runtime_error& e) {
            cerr << "Error while receiving client feedback: " << e.what() << endl;
        }
    }
}


int main(int argc, char* argv[]) {
    // Suppress FFmpeg logs
    av_log_set_level(AV_LOG_QUIET);

    // Register signal handler for Control-C
    // std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string default_port = "12345";
    std::string port_str = (argc > 1) ? argv[1] : default_port;

    long frames = 0;
    auto start_time = std::chrono::high_resolution_clock::now(); // Start time

    FrameMapManager frame_manager = FrameMapManager();
    atomic<int> current_max_mv_blocks(initial_max_motion_vector_blocks);

    boost::asio::io_context io_context;
    tcp::socket socket(io_context);
    boost::asio::io_context::strand strand(io_context);
    // boost::asio::signal_set signals(io_context, SIGINT, SIGTERM);

    int actual_port = port_str.empty() ? stoi(default_port) : std::stoi(port_str);
    tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), actual_port));
    std::cout << "Server is running on port " << actual_port << ". Waiting for connection..." <<
            std::endl;

    acceptor.accept(socket);
    std::cout << "Client connected. Executing RealSense processing..." <<
            std::endl;


    auto benchmark_map = ServerBenchmarkMap();

    thread producer_thead(produce_frames_to_encode, ref(benchmark_map), ref(current_max_mv_blocks));

    thread encoder_thread(encode_draco_frames, ref(frame_manager), ref(benchmark_map));
    thread encoder_thread2(encode_draco_frames, ref(frame_manager), ref(benchmark_map));
    thread encoder_thread3(encode_draco_frames, ref(frame_manager), ref(benchmark_map));


    thread serializer_thread1(serialize_dto, ref(benchmark_map));
    thread serializer_thread2(serialize_dto, ref(benchmark_map));
    thread serializer_thread3(serialize_dto, ref(benchmark_map));
    thread serializer_thread4(serialize_dto, ref(benchmark_map));

    thread receive_client_feedback_thread(receive_client_feedback, ref(socket), ref(current_max_mv_blocks));



    send_to_client(socket, strand, benchmark_map);


    // Clean up and exit
    boost::system::error_code ec;
    socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    if (ec) {
        cerr << "Error during TCP Socket shutdown: " << ec.message() << endl;
    } else {
        cout << "TCP Socket shutdown successfully." << endl;
    }
    producer_thead.join();
    encoder_thread.join();
    encoder_thread2.join();
    encoder_thread3.join();
    serializer_thread1.join();
    serializer_thread2.join();
    serializer_thread3.join();
    serializer_thread4.join();

    receive_client_feedback_thread.join();

    // benchmark_map.saveAsCsv(benchmark_output_folder +"/" + getBenchmarkFileName() +"_server_benchmark.csv");

    std::cout << "Server stopped." << std::endl;


    return 0;
}

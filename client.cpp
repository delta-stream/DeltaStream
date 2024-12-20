#include "client.h"

#include <future>
#include <iostream>
#include <queue>
#include <boost/asio/connect.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/ip/tcp.hpp>

#include "Benchmark.h"
#include "ClientBenchmarkMap.h"
#include "draco.h"
#include "FrameDataType.h"
#include "tools.h"
#include "config.h"
#include "DecodedFrameDTOManager.h"
#include "FPSCounter.h"
#include "custom_types/ClientFeedback.h"
#include "custom_types/MultiCameraEncodedFrameDTO.h"
#include "encoded_frame_handlers/PartialFrameHandler.h"


using namespace std;
namespace o3d = open3d;


auto INIT_DELAY_MS = 3000;

long tcp_received_frames = 0;
long tcp_receive_duration_ms = 0;

boost::asio::io_context io_context_;
boost::asio::ip::tcp::socket socket_(io_context_);

queue<MultiCameraEncodedFrameDTO> encoded_frame_dto_queue;
std::mutex encoded_frame_dto_queue_mutex;
std::condition_variable encoded_frame_dto_queue_cv;

queue<pair<long, shared_ptr<o3d::geometry::PointCloud>>> render_pc_queue;
std::mutex render_pc_queue_mutex;
std::condition_variable render_pc_queue_cv;

queue<ClientFeedback> client_feedback_queue;
std::mutex client_feedback_queue_mutex;
std::condition_variable client_feedback_queue_cv;

// Global atomic flag to indicate when to stop the server
std::atomic<bool> stop_client(false);

// Signal handler for Control-C
void signal_handler(const int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        cout << "SIGINT/SIGTERM received. Stopping client..." << endl;
        stop_client.store(true);
    }
}


void receiveEncodedFrame(ClientBenchmarkMap &benchmark_map)
{
    boost::system::error_code ec;
    // Read the total message size
    uint32_t total_size_network = 0;

    map<long, ClientBenchmark> benchmarks;
    while (!stop_client) {
        auto start_receive = std::chrono::high_resolution_clock::now();
        boost::asio::read(socket_, boost::asio::buffer(&total_size_network, sizeof(total_size_network)), ec);
        uint32_t total_size = ntohl(total_size_network);

        if (ec)
        {
            std::cerr << "Error reading length prefix: " << ec.message() << std::endl;
            break;
        }

        // Read the serialized data
        std::vector<char> buffer(total_size);
        boost::asio::read(socket_, boost::asio::buffer(buffer.data(), total_size), ec);

        if (ec)
        {
            std::cerr << "Error during binary data receive: " << ec.message() << std::endl;
            break;
        }
        auto end_receive = std::chrono::high_resolution_clock::now();
        auto receive_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_receive - start_receive).count();

        auto frame_received_at = timeSinceEpochMillisec();

        // Deserialize to DTO and convert to EncodedFrame
        // EncodedFrameDTO dto = EncodedFrameDTO::deserialize(buffer);

        auto deserialize_start = std::chrono::high_resolution_clock::now();
        MultiCameraEncodedFrameDTO dto;
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
        auto deserialize_end = std::chrono::high_resolution_clock::now();
        auto deserialize_duration = std::chrono::duration_cast<std::chrono::milliseconds>(deserialize_end - deserialize_start).count();
        benchmarks[dto.frame_id] = ClientBenchmark(dto.frame_id);
        benchmarks[dto.frame_id].receive_ms = receive_duration;
        benchmarks[dto.frame_id].received_at = frame_received_at;
        benchmarks[dto.frame_id].deserialize_ms = deserialize_duration;

        {
            std::lock_guard<std::mutex> lock(encoded_frame_dto_queue_mutex);
            encoded_frame_dto_queue.push(dto);
            // cout << "encoded_frame_dto_queue size: " << encoded_frame_dto_queue.size() << endl;
        }
        encoded_frame_dto_queue_cv.notify_one();


    }
    benchmark_map.mergeReceiveFrames(benchmarks);

}

void sendClientFeedback(boost::asio::ip::tcp::socket &socket_) {
    while (!stop_client) {
        std::unique_lock<std::mutex> lock(client_feedback_queue_mutex);

        if (!client_feedback_queue_cv.wait_for(lock, std::chrono::milliseconds(5),
                                               [] { return !client_feedback_queue.empty(); }
        )) {
            // Timeout occurred
        } else {
            auto client_feedback = client_feedback_queue.front();
            client_feedback_queue.pop();
            lock.unlock();

            std::stringstream ss; {
                cereal::BinaryOutputArchive oarchive(ss);
                oarchive(client_feedback);
            }
            // Now ss.str() contains the serialized data
            std::string res_serialized_str = ss.str();

            boost::system::error_code res_ec;

            // Send the size of the data first
            auto res_total_size = static_cast<uint32_t>(res_serialized_str.size());
            uint32_t res_total_size_network = htonl(res_total_size);
            std::vector<boost::asio::const_buffer> buffers;
            buffers.emplace_back(boost::asio::buffer(
                    &res_total_size_network, sizeof(res_total_size_network)
                )
            );
            buffers.emplace_back(boost::asio::buffer(res_serialized_str));

            // Send the data
            boost::asio::write(socket_, buffers, res_ec);
        }
    }
}

void decode_pointclouds(
    DecodedFrameDTOManager& decoded_frame_dto_manager,
    ClientBenchmarkMap &benchmark_map
) {
    map<long, ClientBenchmark> benchmarks;
    while (!stop_client) {
        std::unique_lock<std::mutex> lock(encoded_frame_dto_queue_mutex);

        if (!encoded_frame_dto_queue_cv.wait_for(lock, std::chrono::seconds(1),
                               [] { return !encoded_frame_dto_queue.empty(); }
        )) {
            // Timeout occurred, continue the loop
            lock.unlock();
            continue;
        }

        auto mc_encoded_frame_dto = encoded_frame_dto_queue.front();
        encoded_frame_dto_queue.pop();
        // cout << "encoded_frame_dto_queue size: " << encoded_frame_dto_queue.size() << endl;
        lock.unlock();

        auto decode_start = std::chrono::high_resolution_clock::now();
        auto multi_camera_current_pc = map<int, DecodedFrameDTO>();

        // Vector to hold future results
        std::vector<future<std::pair<int, DecodedFrameDTO>>> futures;

        for (const auto& [camera_num, encoded_frame_dto] : mc_encoded_frame_dto.frame_data_per_camera) {
            futures.push_back(std::async(std::launch::async, [camera_num, encoded_frame_dto]() {
                const auto buffer = encoded_frame_dto.encoder_buffer_data;

                auto decoder_buffer = draco::DecoderBuffer();
                decoder_buffer.Init(reinterpret_cast<const char*>(buffer.data()), buffer.size());

                const auto draco_decoded_frame_point_cloud = decode_point_cloud(decoder_buffer);
                auto decoded_pc = convert_draco_point_cloud_to_open3d_point_cloud(draco_decoded_frame_point_cloud);

                return std::make_pair(camera_num, DecodedFrameDTO(
                    encoded_frame_dto.type,
                    encoded_frame_dto.frame_id,
                    encoded_frame_dto.created_at,
                    decoded_pc,
                    encoded_frame_dto.block_index_to_remove,
                    encoded_frame_dto.motion_vector
                ));
            }));
        }
        auto mc_decoded_frame_dto = make_shared<MultiCameraDecodedFrameDTO>(MultiCameraDecodedFrameDTO(
            mc_encoded_frame_dto.type,
            mc_encoded_frame_dto.frame_id,
            {}
        ));
        // Collect results from futures
        for (auto& future : futures) {
            auto [camera_num, decoded_frame_dto] = future.get();
            auto encoded_frame_dto = mc_encoded_frame_dto.frame_data_per_camera[camera_num];
            mc_decoded_frame_dto->frame_data_per_camera[camera_num] = decoded_frame_dto;
        }
        auto decode_end = std::chrono::high_resolution_clock::now();
        auto decode_duration = std::chrono::duration_cast<std::chrono::milliseconds>(decode_end - decode_start).count();
        benchmarks[mc_encoded_frame_dto.frame_id].decode_ms = decode_duration;

        decoded_frame_dto_manager.InsertFrame(mc_encoded_frame_dto.frame_id, mc_decoded_frame_dto);
    }
    benchmark_map.mergeDecodePC(benchmarks);
}


shared_ptr<open3d::geometry::PointCloud> handle_motion_vector_per_camera(
    const DecodedFrameDTO &dto,
    shared_ptr<open3d::geometry::PointCloud> prev_pc,
    shared_ptr<open3d::geometry::PointCloud> new_pc,
    const int camera_num
) {
    auto block_indices_to_remove = dto.block_index_to_remove;
    auto block_mv = dto.motion_vector;
    auto pcd_points = prev_pc->points_;
    auto pcd_colors = prev_pc->colors_;
    size_t num_points = pcd_points.size();

    // Create Eigen matrices for points and colors
    Eigen::MatrixXd prev_pc_mat(num_points, 3);
    Eigen::MatrixXd pcd_color(num_points, 3);

    for (long idx = 0; idx < num_points; ++idx)
    {
        prev_pc_mat(idx, 0) = pcd_points[idx][0];
        prev_pc_mat(idx, 1) = pcd_points[idx][1];
        prev_pc_mat(idx, 2) = pcd_points[idx][2];

        pcd_color(idx, 0) = pcd_colors[idx][0];
        pcd_color(idx, 1) = pcd_colors[idx][1];
        pcd_color(idx, 2) = pcd_colors[idx][2];
    }

    auto depth_config = getCamDepthConfig(camera_num);

    // Compute u and v
    Eigen::VectorXd x = prev_pc_mat.col(0);
    Eigen::VectorXd y = prev_pc_mat.col(1);
    Eigen::VectorXd z = prev_pc_mat.col(2);

    Eigen::VectorXd u = (x.array() / z.array()) * depth_config.fx + depth_config.cx;
    Eigen::VectorXd v = (y.array() / z.array()) * depth_config.fy + depth_config.cy;

    Eigen::VectorXi u_int = u.array().round().cast<int>();
    Eigen::VectorXi v_int = v.array().round().cast<int>();


    // Initialize containers for updated points and colors
    std::vector<Eigen::Vector3d> mv_updated_points;
    std::vector<Eigen::Vector3d> mv_updated_colors;

    std::vector<Eigen::Vector3d> points_to_move_by_mv;
    // Update points using motion vectors from dto.motion_vector
    for (const auto& [block_idx, delta] : dto.motion_vector)
    {
        auto [delta_x, delta_y, delta_z, src_x, src_y] = delta;
        int u_1 = src_x - block_size / 2;
        int v_1 = src_y - block_size / 2;

        // Filter indices where points are within the block
        std::vector<size_t> filtered_indices;
        for (size_t idx = 0; idx < u.size(); ++idx)
        {
            if (u(idx) >= u_1 && u(idx) < u_1 + block_size &&
                v(idx) >= v_1 && v(idx) < v_1 + block_size)
            {
                filtered_indices.push_back(idx);
            }
        }

        // Update the points
        for (size_t idx : filtered_indices)
        {
            Eigen::Vector3d point = prev_pc_mat.row(idx);
            points_to_move_by_mv.emplace_back(point[0], point[1], point[2]);

            point[0] += delta_x;
            point[1] += delta_y;
            point[2] += delta_z;


            mv_updated_points.push_back(point);
            mv_updated_colors.push_back(pcd_color.row(idx));
        }
    }

    auto mv_updated_point_cloud = open3d::geometry::PointCloud();
    auto b4_mv_update_pc = open3d::geometry::PointCloud();
    // Merging
    if (!mv_updated_points.empty())
    {
        mv_updated_point_cloud.points_ = mv_updated_points;
        mv_updated_point_cloud.colors_ = mv_updated_colors;

        b4_mv_update_pc.points_ = points_to_move_by_mv;
        b4_mv_update_pc.colors_ = mv_updated_colors;
    }
    auto mv_updated_point_cloud_srd_ptr = make_shared<open3d::geometry::PointCloud>(mv_updated_point_cloud);
    auto mv_point_cloud_srd_ptr = make_shared<open3d::geometry::PointCloud>(b4_mv_update_pc);

    // additional point cloud == delta point cloud in the paper
    // Merge updated point cloud and additional point cloud
    open3d::geometry::PointCloud final_pc;

    if (!mv_updated_point_cloud.points_.empty())
    {
        // Append points and colors from mv_updated_point_cloud
        final_pc.points_.insert(final_pc.points_.end(),
                                mv_updated_point_cloud.points_.begin(),
                                mv_updated_point_cloud.points_.end());
        final_pc.colors_.insert(final_pc.colors_.end(),
                                mv_updated_point_cloud.colors_.begin(),
                                mv_updated_point_cloud.colors_.end());
    }

    if (!new_pc->points_.empty()) {
        // Append points and colors from additional_pc
        final_pc.points_.insert(final_pc.points_.end(),
                                new_pc->points_.begin(),
                                new_pc->points_.end());
        final_pc.colors_.insert(final_pc.colors_.end(),
                                new_pc->colors_.begin(),
                                new_pc->colors_.end());
    }


    // Convert block_removes_final to an unordered_set for efficient lookup
    std::unordered_set<int> block_to_remove(block_indices_to_remove.begin(), block_indices_to_remove.end());

    // Calculate block indices for all points
    int blocks_per_row = frame_width / block_size;

    Eigen::VectorXi block_indices = ((v.array() / block_size).cast<int>() * blocks_per_row +
        (u.array() / block_size).cast<int>());

    // Determine indices of points to remove
    std::vector<bool> remove_indices(prev_pc_mat.rows(), false);
    for (int i = 0; i < block_indices.size(); ++i)
    {
        if (block_to_remove.contains(block_indices[i]))
        {
            remove_indices[i] = true;
        }
    }

    // Get remaining points and colors
    std::vector<Eigen::Vector3d> remaining_pcd;
    std::vector<Eigen::Vector3d> remaining_pcd_color;

    for (int i = 0; i < prev_pc_mat.rows(); ++i)
    {
        if (!remove_indices[i])
        {
            remaining_pcd.emplace_back(prev_pc_mat(i, 0), prev_pc_mat(i, 1), prev_pc_mat(i, 2));
            remaining_pcd_color.emplace_back(pcd_color(i, 0), pcd_color(i, 1), pcd_color(i, 2));
        }
    }

    // Create remaining point cloud
    open3d::geometry::PointCloud remaining_pc;
    remaining_pc.points_ = remaining_pcd;
    remaining_pc.colors_ = remaining_pcd_color;


    // Combine final_pc and remaining_pc to get the updated point cloud
    auto new_new_pc = open3d::geometry::PointCloud();

    if (!final_pc.points_.empty()) {
        // Append points and colors from final_pc
        new_new_pc.points_.insert(new_new_pc.points_.end(),
                                  final_pc.points_.begin(),
                                  final_pc.points_.end());
        new_new_pc.colors_.insert(new_new_pc.colors_.end(),
                                  final_pc.colors_.begin(),
                                  final_pc.colors_.end());
    }

    if (!remaining_pc.points_.empty()) {
        // Append points and colors from remaining_pc
        new_new_pc.points_.insert(new_new_pc.points_.end(),
                                  remaining_pc.points_.begin(),
                                  remaining_pc.points_.end());
        new_new_pc.colors_.insert(new_new_pc.colors_.end(),
                                  remaining_pc.colors_.begin(),
                                  remaining_pc.colors_.end());
    }

    return make_shared<open3d::geometry::PointCloud>(new_new_pc);
}

map<int, shared_ptr<open3d::geometry::PointCloud>> handle_motion_vector(
    const MultiCameraDecodedFrameDTO &mc_decoded_frame_dto,
    map<int, shared_ptr<open3d::geometry::PointCloud>> &multi_camera_prev_pc
)
{
    auto multi_camera_pcs = map<int, shared_ptr<open3d::geometry::PointCloud>>();
    // Vector to hold future results
    vector<future<pair<int, shared_ptr<open3d::geometry::PointCloud>>>> futures;

    for (const auto& [camera_num, dto] : mc_decoded_frame_dto.frame_data_per_camera) {
        shared_ptr<open3d::geometry::PointCloud> prev_pc = multi_camera_prev_pc
                [camera_num];
        shared_ptr<open3d::geometry::PointCloud> new_pc =
            mc_decoded_frame_dto.frame_data_per_camera.at(camera_num).pointcloud;


        futures.push_back(async(launch::async, [camera_num, dto, prev_pc, new_pc]() {
            return make_pair(camera_num, handle_motion_vector_per_camera(dto, prev_pc, new_pc,camera_num));
        }));
    }

    // Collect results from futures
    for (auto& future : futures) {
        auto [camera_num, mv_processed_pc] = future.get();
        multi_camera_pcs[camera_num] = mv_processed_pc;
    }

    return multi_camera_pcs;
}

void handle_receive(
    const MultiCameraDecodedFrameDTO& mc_decoded_frame_dto,
    map<int, shared_ptr<open3d::geometry::PointCloud>>& multi_camera_prev_pc
)
{
    const auto frame_id = mc_decoded_frame_dto.frame_id;
    const auto st = std::chrono::high_resolution_clock::now();

    auto multi_camera_current_pc = mc_decoded_frame_dto;

    map<int, shared_ptr<open3d::geometry::PointCloud>> multi_camera_processed_current_pc;

    if (mc_decoded_frame_dto.type == Full)
    {
        for (const auto& [camera_num, decoded_frame_dto] : mc_decoded_frame_dto.frame_data_per_camera) {
            multi_camera_processed_current_pc[camera_num] = decoded_frame_dto.pointcloud;
        }
    }
    else if (mc_decoded_frame_dto.type == Partial)
    {
        try {
            multi_camera_processed_current_pc = PartialFrameHandler::handle(
                mc_decoded_frame_dto, multi_camera_prev_pc
                );
        }catch (runtime_error& e) {
            cerr << "Error handling partial frame: " << e.what() << endl;
        }
    }
    else if (mc_decoded_frame_dto.type == MotionVector)
    {
        try {
            const auto mv_start = std::chrono::high_resolution_clock::now();
            multi_camera_processed_current_pc = handle_motion_vector(
                mc_decoded_frame_dto, multi_camera_prev_pc
                );

            const auto end = std::chrono::high_resolution_clock::now();
            const auto mv_duration_ms = chrono::duration_cast<std::chrono::milliseconds>(end - mv_start).count();


            long point_count = 0;
            for (const auto& [camera_num, pc] : multi_camera_processed_current_pc) {
                point_count += pc->points_.size();
            }


            cout << "[F" << frame_id << "] handle_motion_vector: " << mv_duration_ms << "ms" << endl;
        }catch (runtime_error& e) {
            cerr << "Error handling partial frame: " << e.what() << endl;
        }
    }


    auto this_frame_pc = std::make_shared<open3d::geometry::PointCloud>();
    for (const auto& [camera_num, pc] : multi_camera_processed_current_pc) {
        auto copy = std::make_shared<open3d::geometry::PointCloud>();

        copy->points_ = pc->points_;
        copy->colors_ = pc->colors_;

        if (camera_num == 1) {
            // TODO: Define Transformation Matrix to align this camera's point cloud with the first camera
            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

            // Apply transformation
            copy->Transform(transformation_matrix);
        }else if (camera_num == 2) {
            // TODO: Define Transformation Matrix to align this camera's point cloud with the first camera
            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

            // Apply transformation
            copy->Transform(transformation_matrix);
        }else if (camera_num == 3) {
            // TODO: Define Transformation Matrix to align this camera's point cloud with the first camera
            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

            // Apply transformation
            copy->Transform(transformation_matrix);
        }


        this_frame_pc->points_.insert(this_frame_pc->points_.end(),
                              copy->points_.begin(),
                              copy->points_.end());
        this_frame_pc->colors_.insert(this_frame_pc->colors_.end(),
                              copy->colors_.begin(),
                              copy->colors_.end());

        multi_camera_prev_pc[camera_num] = pc;
    }

    {
        std::lock_guard<std::mutex> lock(render_pc_queue_mutex);
        render_pc_queue.push(make_pair(frame_id, this_frame_pc));
    }
    render_pc_queue_cv.notify_one();

    const auto hr_end = std::chrono::high_resolution_clock::now();
    const auto hr_duration_ms = chrono::duration_cast<std::chrono::milliseconds>(hr_end - st).count();

    // frame_trace_map[frame_id].spans.push_back(FrameTraceSpan("handle_motion_vector", mv_duration_ms));

    cout << "[F" << frame_id << "] handle_receive: " << hr_duration_ms << "ms" << endl;
}

void render_pcs(ClientBenchmarkMap &benchmark_map) {
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();

    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Received PointCloud", 1280, 720);
    visualizer.AddGeometry(cloud);
    bool first_frame = true;
    long frames = 0;
    double fps = 0.0;
    map<long, ClientBenchmark> benchmarks;
    FPSCounter fpsCounter(120); // Smooth over 60 frames
    auto lastTime = std::chrono::high_resolution_clock::now();

    auto start_time = std::chrono::high_resolution_clock::now();
    while (!stop_client) {
        std::unique_lock<std::mutex> lock(render_pc_queue_mutex);

        if (!render_pc_queue_cv.wait_for(lock, std::chrono::seconds(1),
                               [] { return !render_pc_queue.empty(); }
        )) {
            // Timeout occurred, continue the loop
            lock.unlock();
            continue;
        }

        auto [frame_id, pc_to_render] = render_pc_queue.front();
        render_pc_queue.pop();
        lock.unlock();


        auto render_start = std::chrono::high_resolution_clock::now();

        if (pc_to_render->IsEmpty()) {
            frames++;
            continue;
        }
        cloud->points_ = pc_to_render->points_;
        cloud->colors_ = pc_to_render->colors_;

        if (first_frame)
        {
            visualizer.AddGeometry(cloud);
            first_frame = false;
        }
        else
        {
            visualizer.UpdateGeometry(cloud);
        }

        visualizer.PollEvents();
        visualizer.UpdateRender();


        auto currentTime = std::chrono::high_resolution_clock::now();
        double deltaTime = std::chrono::duration<double>(currentTime - lastTime).count();
        lastTime = currentTime;
        fpsCounter.update(deltaTime);

        fps = fpsCounter.getSmoothedFPS();
        cout << "FPS: " << to_string(fps) << endl;


        auto render_end = std::chrono::high_resolution_clock::now();
        auto render_duration = std::chrono::duration_cast<std::chrono::milliseconds>(render_end - render_start).count();

        cout << "Points: " << cloud->points_.size() << endl;
        frames++;

        cout << "Points: " << cloud->points_.size() << endl;

        benchmarks[frame_id].render_ms = render_duration;
        benchmarks[frame_id].rendered_at = timeSinceEpochMillisec();
        benchmarks[frame_id].fps = fps;


        {
            std::lock_guard<std::mutex> client_feedback_lock(client_feedback_queue_mutex);
            client_feedback_queue.emplace(frame_id, fps);
        }
        client_feedback_queue_cv.notify_one();

    }
    benchmark_map.mergeRender(benchmarks);
    visualizer.DestroyVisualizerWindow();

}

void run(
    DecodedFrameDTOManager& decoded_frame_dto_manager,
    ClientBenchmarkMap &benchmark_map)
{
    auto multi_camera_prev_pc = map<int, shared_ptr<open3d::geometry::PointCloud>>();
    std::vector<uint8_t> accumulated_buffer;
    map<long, ClientBenchmark> benchmarks;

    try
    {
        long frames = 0;
        long next_frame_id_to_consume = 0;
        while (!stop_client)
        {
            auto processing_start_time = std::chrono::high_resolution_clock::now();
            if (!decoded_frame_dto_manager.FrameExists(next_frame_id_to_consume)) {
                continue;
                }

            auto mc_decoded_frame_dto = decoded_frame_dto_manager.GetFrame(
                next_frame_id_to_consume
            );

            cout << "decoded_frame_dto_manager size: " << decoded_frame_dto_manager.GetSize() << endl;

            handle_receive(*mc_decoded_frame_dto, multi_camera_prev_pc);

            auto processing_end_time = std::chrono::high_resolution_clock::now();
            frames++;

            auto processing_duration_ms = chrono::duration_cast<chrono::milliseconds>(processing_end_time - processing_start_time).count();
            benchmarks[mc_decoded_frame_dto->frame_id].pre_render_ms = processing_duration_ms;

            decoded_frame_dto_manager.RemoveFrame(next_frame_id_to_consume);
            next_frame_id_to_consume += 1;

        }
        benchmark_map.mergePreRender(benchmarks);
    }
    catch (std::exception& e)
    {
        cerr << "Exception (run): " << e.what() << endl;
    }
}





int main(int argc, char* argv[])
{
    std::signal(SIGTERM, signal_handler);
    std::this_thread::sleep_for(std::chrono::milliseconds(INIT_DELAY_MS));

    // Define default values
    std::string default_host = "127.0.0.1";
    std::string default_port = "12345";

    // Use provided arguments or default values
    std::string host = (argc > 1) ? argv[1] : default_host;
    std::string port = (argc > 2) ? argv[2] : default_port;

    std::cout << "Connecting to " << host << " on port " << port << std::endl;
    int max_retries = 10;
    long wait_before_retry_ms = 5000;

    DecodedFrameDTOManager frame_manager = DecodedFrameDTOManager();


    auto benchmark_map = ClientBenchmarkMap();

    thread decoder_thread1;
    thread decoder_thread2;
    thread decoder_thread3;
    thread decoder_thread4;

    thread run_thread;
    thread render_thread;
    thread send_client_feedback_thread;

    for (int i = 0; i < max_retries; i++)
    {
        try
        {
            boost::asio::ip::tcp::resolver resolver(io_context_);
            boost::asio::connect(socket_, resolver.resolve(host, port));
            std::cout << "Connected to " << host << " on port " << port << std::endl;

            decoder_thread1 = thread(decode_pointclouds, std::ref(frame_manager), ref(benchmark_map));
            decoder_thread2 = thread(decode_pointclouds, std::ref(frame_manager), ref(benchmark_map));
            decoder_thread3 = thread(decode_pointclouds, std::ref(frame_manager), ref(benchmark_map));
            decoder_thread4 = thread(decode_pointclouds, std::ref(frame_manager), ref(benchmark_map));

            run_thread = thread(run, std::ref(frame_manager), ref(benchmark_map));
            render_thread = thread(render_pcs, ref(benchmark_map));
            send_client_feedback_thread = thread(sendClientFeedback, ref(socket_));
            // client client(host, port);
            receiveEncodedFrame(benchmark_map);
            break;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
            std::cerr << "Retrying in " << wait_before_retry_ms << " ms..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_before_retry_ms));
        }
    }


    decoder_thread1.join();
    decoder_thread2.join();
    decoder_thread3.join();
    decoder_thread4.join();


    render_thread.join();
    run_thread.join();
    send_client_feedback_thread.join();

    // benchmark_map.saveAsCsv(benchmark_output_folder +"/_client_benchmark.csv");
    // cout << "Client benchmark saved." << endl;

    return 0;
}



#include "PartialFrameHandler.h"
#include "../config.h"
#include "../custom_types/MultiCameraDecodedFrameDTO.h"
#include "../custom_types/MultiCameraEncodedFrameDTO.h"

using namespace std;
namespace o3d = open3d;

map<int, shared_ptr<open3d::geometry::PointCloud>> PartialFrameHandler::handle(
    const MultiCameraDecodedFrameDTO &mc_decoded_frame_dto,
    map<int, shared_ptr<open3d::geometry::PointCloud>> &multi_camera_prev_pc
)
{
    auto multi_camera_pcs = map<int, shared_ptr<open3d::geometry::PointCloud>>();

    for (auto [camera_num, encoded_frame_dto] : mc_decoded_frame_dto.frame_data_per_camera) {
        auto block_index_to_remove = encoded_frame_dto.block_index_to_remove;
        if (encoded_frame_dto.pointcloud->IsEmpty()) {
            multi_camera_pcs[camera_num]->points_ = multi_camera_prev_pc.at(camera_num)->points_;
            multi_camera_pcs[camera_num]->colors_ = multi_camera_prev_pc.at(camera_num)->colors_;
            continue;
        }

        if (auto search = multi_camera_prev_pc.find(camera_num); search != multi_camera_prev_pc.end()) {
        }
        else {
            std::cout << "Not found\n";
            continue;
        }
        auto pcd_points = multi_camera_prev_pc.at(camera_num)->points_;
        auto pcd_colors = multi_camera_prev_pc.at(camera_num)->colors_;
        size_t num_points = pcd_points.size();

        // Create Eigen matrices for points and colors
        Eigen::MatrixXd points(num_points, 3);
        Eigen::MatrixXd colors(num_points, 3);

        for (long idx = 0; idx < num_points; ++idx)
        {
            points(idx, 0) = pcd_points[idx][0];
            points(idx, 1) = pcd_points[idx][1];
            points(idx, 2) = pcd_points[idx][2];

            colors(idx, 0) = pcd_colors[idx][0];
            colors(idx, 1) = pcd_colors[idx][1];
            colors(idx, 2) = pcd_colors[idx][2];
        }

        // Compute u and v
        Eigen::VectorXd x = points.col(0);
        Eigen::VectorXd y = points.col(1);
        Eigen::VectorXd z = points.col(2);

        auto depth_config = getCamDepthConfig(camera_num);

        Eigen::VectorXd u = (x.array() / z.array()) * depth_config.fx + depth_config.cx;
        Eigen::VectorXd v = (y.array() / z.array()) * depth_config.fy + depth_config.cy;

        Eigen::VectorXi u_int = u.array().round().cast<int>();
        Eigen::VectorXi v_int = v.array().round().cast<int>();

        // Compute block indices
        Eigen::VectorXi block_indices = (v_int.array() / block_size) * (frame_width / block_size) + (u_int.array() / block_size);

        // Convert block_index_to_remove to a std::unordered_set for efficient lookup
        std::unordered_set<int> block_index_to_remove_set(block_index_to_remove.begin(), block_index_to_remove.end());

        // Determine which points to remove
        std::vector<bool> remove_indices(num_points);
        for (long idx = 0; idx < num_points; ++idx)
        {
            remove_indices[idx] = (block_index_to_remove_set.contains(block_indices(idx)));
        }

        // Remaining point cloud after removal
        std::vector<Eigen::Vector3d> remaining_points;
        std::vector<Eigen::Vector3d> remaining_colors;

        for (size_t idx = 0; idx < num_points; ++idx)
        {
            if (!remove_indices[idx])
            {
                remaining_points.push_back(pcd_points[idx]);
                remaining_colors.push_back(pcd_colors[idx]);
            }
        }

        auto remaining_point_count = remaining_points.size();

        auto open3d_decoded_frame_point_cloud = mc_decoded_frame_dto.frame_data_per_camera.at(camera_num).pointcloud;

        // Append new points from point_cloud_delta
        auto delta_points = open3d_decoded_frame_point_cloud->points_;
        auto delta_colors = open3d_decoded_frame_point_cloud->colors_;
        remaining_points.insert(remaining_points.end(), delta_points.begin(), delta_points.end());
        remaining_colors.insert(remaining_colors.end(), delta_colors.begin(), delta_colors.end());

        auto new_pc = open3d::geometry::PointCloud();

        new_pc.points_.insert(new_pc.points_.end(),
                              remaining_points.begin(),
                              remaining_points.end());
        new_pc.colors_.insert(new_pc.colors_.end(),
                              remaining_colors.begin(),
                              remaining_colors.end());

        multi_camera_pcs[camera_num] = make_shared<open3d::geometry::PointCloud>(new_pc);
    }


    multi_camera_prev_pc.clear();
    for (const auto& [camera_num, processed_pc] : multi_camera_pcs) {
        multi_camera_prev_pc[camera_num] = processed_pc;
    }


    // cout << "[Partial Update] PrevFramePoints: " << num_points << ", RemainingPoints: " << remaining_point_count <<
    //     ", DeltaPoints: " << delta_points.size() << ", NewFramePoints: " << cloud->points_.size() <<
    //         ", Blocks to delete: " << encoded_frame_dto.block_index_to_remove .size() << endl;

    return multi_camera_pcs;
}



#ifndef DRACO_H
#define DRACO_H

// #pragma GCC optimize("O0")
#include <memory>
#include <draco/compression/decode.h>
#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/compression/expert_encode.h>
#include <draco/core/decoder_buffer.h>
#include <open3d/geometry/PointCloud.h>

#include "logging.h"

using namespace std;

shared_ptr<open3d::geometry::PointCloud>
convert_draco_point_cloud_to_open3d_point_cloud(
    const unique_ptr<draco::PointCloud> &draco_point_cloud
) {
    auto start = std::chrono::high_resolution_clock::now();
    auto open3d_cloud = std::make_shared<open3d::geometry::PointCloud>();

    const auto pos_att = draco_point_cloud->GetNamedAttribute(
        draco::GeometryAttribute::POSITION
    );


    std::vector<Eigen::Vector3d>
            points(draco_point_cloud->num_points());
    for (draco::AttributeValueIndex i(0);
         i < draco_point_cloud->num_points(); ++i) {
        draco::Vector3f pos;
        pos_att->GetValue(draco::AttributeValueIndex(i), &pos[0]);
        points[i.value()] = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    }
    open3d_cloud->points_ = std::move(points);


    // Colors
    const auto color_attribute = draco_point_cloud->GetNamedAttribute(
        draco::GeometryAttribute::COLOR
    );
    std::vector<Eigen::Vector3d>
            colors(draco_point_cloud->num_points());

    for (draco::AttributeValueIndex i(0);
         i < draco_point_cloud->num_points(); ++i) {
        draco::Vector3f col;
        float colors_points[3];
        color_attribute->GetValue(draco::AttributeValueIndex(i), &colors_points);

        colors[i.value()] = Eigen::Vector3d(colors_points[0], colors_points[1], colors_points[2]);
    }
    open3d_cloud->colors_ = std::move(colors);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    if (DEBUG_LOG) {
        // std::cout << "[Draco PC -> Open3d PC] duration: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << " ms" << std::endl;
    }

    return open3d_cloud;
}

inline unique_ptr<draco::PointCloud> decode_point_cloud(
    draco::DecoderBuffer &decoder_buffer
) {
    auto start = std::chrono::high_resolution_clock::now();

    draco::Decoder decoder;
    auto draco_decoded_point_cloud = decoder.DecodePointCloudFromBuffer(
        &decoder_buffer
    ).value();


    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    if (DEBUG_LOG) {
        // std::cout << "[Decode Draco PC] duration: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << " ms" << std::endl;
    }

    return draco_decoded_point_cloud;
}

draco::EncoderBuffer
encode_point_cloud(const draco::PointCloud &draco_cloud) {
    auto start = std::chrono::high_resolution_clock::now();
    draco::Encoder encoder;
    draco::EncoderBuffer buffer;

    encoder.SetSpeedOptions(7, 10);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 11);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::COLOR, 8);


    std::unique_ptr<draco::ExpertEncoder> expert_encoder;
    expert_encoder.reset(new draco::ExpertEncoder(draco_cloud));

    expert_encoder->Reset(encoder.CreateExpertEncoderOptions(draco_cloud));

    const draco::Status status = expert_encoder->EncodeToBuffer(&buffer);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    if (DEBUG_LOG) {
        // std::cout << "[Encode PC] duration: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << " ms" << std::endl;
    }

    if (!status.ok()) {
        throw runtime_error(
            "Error encoding point cloud: " + status.error_msg_string()
        );
    }


    return buffer;
}

std::shared_ptr<draco::PointCloud> convert_open3d_pc_to_draco_pc(
    const shared_ptr<open3d::geometry::PointCloud> &open3d_cloud
) {
    auto start = std::chrono::high_resolution_clock::now();


    draco::PointCloudBuilder builder;
    builder.Start(open3d_cloud->points_.size());

    // Add position attribute
    const int pos_att_id = builder.AddAttribute(
        draco::GeometryAttribute::POSITION,
        3,
        draco::DT_FLOAT32
    );

    for (draco::PointIndex i(0); i < open3d_cloud->points_.size(); ++i) {
        const Eigen::Vector3d &point = open3d_cloud->points_[i.value()];
        const auto x = static_cast<float>(point(0));
        const auto y = static_cast<float>(point(1));
        const auto z = static_cast<float>(point(2));


        // vector<double> pos = {x, y, z};
        float pos[3] = {x,y,z};

        builder.SetAttributeValueForPoint(pos_att_id, i,
        pos
        );
    }


    // Add color attribute
    const int color_att_id = builder.AddAttribute(
        draco::GeometryAttribute::COLOR,
        3,
        draco::DataType::DT_FLOAT32
    );

    // std::ofstream open3d_colors_file("open3d_colors.txt");
    for (draco::PointIndex i(0); i < open3d_cloud->colors_.size(); ++i) {
        const Eigen::Vector3d &color = open3d_cloud->colors_[i.value()];

        float colors[3] = {
            static_cast<float>(color(0)),
            static_cast<float>(color(1)),
            static_cast<float>(color(2))
        };

        builder.SetAttributeValueForPoint(color_att_id, i,
                                          colors
        );
    }

    auto pc = builder.Finalize(false);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    if (DEBUG_LOG) {
        // std::cout << "[Open3D PC -> Draco PC] duration: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << " ms" << std::endl;
    }

    // return pc;
    return move(pc);
}

#endif //DRACO_H

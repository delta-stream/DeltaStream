

#ifndef CONFIG_H
#define CONFIG_H
#include <condition_variable>
#include <filesystem>
#include <cstdlib>
#include <map>
#include <string>

#include "custom_types/DepthConfig.h"

namespace fs = std::filesystem;

using namespace std;

inline int INF_MAX_DEPTH = 100000;

enum StreamMode {
    LIVE_SCAN_3D,
    METASTREAM,
    DELTASTREAM,
};

inline map<StreamMode, string> stream_mode_str_map = {
    {LIVE_SCAN_3D, "livescan3d"},
    {METASTREAM, "metastream"},
    {DELTASTREAM, "deltastream"}
};

enum Dataset {
    TEST
};
inline map<Dataset, string> img_dataset_str_map = {
    {TEST, "test"}
};

inline constexpr StreamMode stream_mode = DELTASTREAM;
inline int num_cameras = 1; // change this value according to the number of cameras
// adjust min/max depth filter as needed
inline vector<pair<int, int>> depth_filters_per_cam = {
    make_pair(0, stream_mode == LIVE_SCAN_3D ? INF_MAX_DEPTH : 1500), // Camera 1 min/max depth filter
    make_pair(0, stream_mode == LIVE_SCAN_3D ? INF_MAX_DEPTH : 2000), // Camera 2 min/max depth filter
    make_pair(0, stream_mode == LIVE_SCAN_3D ? INF_MAX_DEPTH : 1000), // Camera 3 min/max depth filter
    make_pair(0, stream_mode == LIVE_SCAN_3D ? INF_MAX_DEPTH : 1000) // Camera 4 min/max depth filter
};
inline int rs_w = 640;
inline int rs_h = 480;

inline bool img_mode = false;
inline constexpr Dataset img_dataset = TEST;
inline int img_w = 640;
inline int img_h = 480;


inline int frame_width = img_mode ? img_w : rs_w;
inline int frame_height = img_mode ? img_h : rs_h;
inline int block_size = 16;
inline int half_block_size = block_size / 2;
inline double threshold = 10000; // Threshold for block difference
inline double mv_threshold = 5000;
inline double variance_threshold = 100;

inline string getBenchmarkFileName() {
    return "_" + img_dataset_str_map[img_dataset] + "_" + stream_mode_str_map[stream_mode];
}

inline map<Dataset, vector<pair<int, int> > > img_depth_filters_per_cam = {
{
        TEST, {
            make_pair(0, stream_mode == LIVE_SCAN_3D ? INF_MAX_DEPTH : 2000),
            make_pair(0, stream_mode == LIVE_SCAN_3D ? INF_MAX_DEPTH : 2000),
            make_pair(0, stream_mode == LIVE_SCAN_3D ? INF_MAX_DEPTH : 2000),
            make_pair(0, stream_mode == LIVE_SCAN_3D ? INF_MAX_DEPTH : 2000),
        }
    }
};


inline const char *env_benchmark_output_folder = std::getenv(
    "BENCHMARK_OUTPUT_FOLDER"
);
inline string benchmark_output_folder = env_benchmark_output_folder
                                            ? std::string(
                                                env_benchmark_output_folder
                                            )
                                            : ".";


inline int keyframe_frequency = (stream_mode == LIVE_SCAN_3D || stream_mode == METASTREAM) ? 1 : 5; // Send keyframe every x frames
inline bool use_motion_vector = stream_mode == DELTASTREAM;
inline int initial_max_motion_vector_blocks = (img_mode ? img_w : rs_w) * (img_mode ? img_h : rs_h) /block_size/block_size;




inline pair<int,int> getCamDepthFilters(const int cam_num) {
    if (img_mode)
        return img_depth_filters_per_cam[img_dataset][cam_num];
    else
        return depth_filters_per_cam[cam_num];
}


inline vector<string> get_filenames_in_folder(const string &folder_path) {
    vector<string> file_paths;
    for (const auto &entry: fs::directory_iterator(folder_path)) {
        if (entry.is_regular_file()) {
            // Check if it is a regular file
            file_paths.push_back(fs::absolute(entry.path()).string());
        }
    }

    // Sort file paths by the number in the file name
    std::sort(file_paths.begin(), file_paths.end(),
              [](const string &a, const string &b) {
                  auto extract_number = [](const string &path) {
                      size_t last_underscore = path.find_last_of('_');
                      size_t last_dot = path.find_last_of('.');
                      return std::stoi(path.substr(
                              last_underscore + 1,
                              last_dot - last_underscore - 1
                          )
                      );
                  };
                  return extract_number(a) < extract_number(b);
              }
    );

    return file_paths;
}

inline int img_align_shift = 17; // Number of pixels to shift
inline const char *env_cam1_img_color_images_folder = std::getenv(
    "IMG_CAM1_COLOR_IMAGES_FOLDER"
);
inline const char *env_cam1_img_depth_images_folder = std::getenv(
    "IMG_CAM1_DEPTH_IMAGES_FOLDER"
);
inline const char *env_cam2_img_color_images_folder = std::getenv(
    "IMG_CAM2_COLOR_IMAGES_FOLDER"
);
inline const char *env_cam2_img_depth_images_folder = std::getenv(
    "IMG_CAM2_DEPTH_IMAGES_FOLDER"
);
inline const char *env_cam3_img_color_images_folder = std::getenv(
    "IMG_CAM3_COLOR_IMAGES_FOLDER"
);
inline const char *env_cam3_img_depth_images_folder = std::getenv(
    "IMG_CAM3_DEPTH_IMAGES_FOLDER"
);
inline const char *env_cam4_img_color_images_folder = std::getenv(
    "IMG_CAM4_COLOR_IMAGES_FOLDER"
);
inline const char *env_cam4_img_depth_images_folder = std::getenv(
    "IMG_CAM4_DEPTH_IMAGES_FOLDER"
);


inline string cam1_img_color_images_folder = env_cam1_img_color_images_folder ? std::string(env_cam1_img_color_images_folder) : "/home/user/Documents/test/color";
inline string cam1_img_depth_images_folder = env_cam1_img_depth_images_folder ? std::string(env_cam1_img_depth_images_folder) : "/home/user/Documents/test/depth";
inline string cam2_img_color_images_folder = env_cam2_img_color_images_folder ? std::string(env_cam2_img_color_images_folder) : "/home/user/Documents/test/color";
inline string cam2_img_depth_images_folder = env_cam2_img_depth_images_folder ? std::string(env_cam2_img_depth_images_folder) : "/home/user/Documents/test/depth";
inline string cam3_img_color_images_folder = env_cam3_img_color_images_folder ? std::string(env_cam3_img_color_images_folder) : "/home/user/Documents/test/color";
inline string cam3_img_depth_images_folder = env_cam3_img_depth_images_folder ? std::string(env_cam3_img_depth_images_folder) : "/home/user/Documents/test/depth";
inline string cam4_img_color_images_folder = env_cam4_img_color_images_folder ? std::string(env_cam4_img_color_images_folder) : "/home/user/Documents/test/color";
inline string cam4_img_depth_images_folder = env_cam4_img_depth_images_folder ? std::string(env_cam4_img_depth_images_folder) : "/home/user/Documents/test/depth";

inline vector<vector<string>> per_cam_img_color_images = {
    get_filenames_in_folder(cam1_img_color_images_folder),
    get_filenames_in_folder(cam2_img_color_images_folder),
    get_filenames_in_folder(cam3_img_color_images_folder),
    get_filenames_in_folder(cam4_img_color_images_folder)
};

inline vector<vector<string>> per_cam_img_depth_images = {
    get_filenames_in_folder(cam1_img_depth_images_folder),
    get_filenames_in_folder(cam2_img_depth_images_folder),
    get_filenames_in_folder(cam3_img_depth_images_folder),
    get_filenames_in_folder(cam4_img_depth_images_folder)
};

inline vector<DepthConfig> rs_per_cam_depth_configs;

// Add depth settings for each camera.
inline vector<DepthConfig> per_cam_depth_configs{
    // D455
    DepthConfig{
        .depth_scale = 0.0010000000474974513,
        .cx = 324.4732971191406,
        .cy = 239.3256072998047,
        .fx = 388.68609619140625,
        .fy = 388.68609619140625
    },
    // D455
    DepthConfig{
        .depth_scale = 0.0010000000474974513,
        .cx = 324.4732971191406,
        .cy = 239.3256072998047,
        .fx = 388.68609619140625,
        .fy = 388.68609619140625
    },
    // D415
    DepthConfig{
        .depth_scale = 0.0010000000474974513,
        .cx = 307.0904846191406,
        .cy = 245.5484161376953,
        .fx = 595.336669921875,
        .fy = 595.336669921875
    },
    // D435I (SN: 845112071364)
    DepthConfig{
        .depth_scale = 0.0010000000474974513,
        .cx = 319.537841796875,
        .cy = 234.16282653808594,
        .fx = 384.0450744628906,
        .fy = 384.0450744628906
    },
};


inline DepthConfig getCamDepthConfig(const int cam_num) {
    return per_cam_depth_configs[cam_num];
}





#endif //CONFIG_H

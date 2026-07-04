#ifndef __PARAMS_LOADER_H_
#define __PARAMS_LOADER_H_

#include <opencv2/core/persistence.hpp>
#include <string>
#include <iostream>
#include "../method_math.h"

namespace Ten::kfs_locator {

/**
 * @brief 所有可通过 YAML 配置的运行时参数
 *
 * 使用方式：
 *   KfsParams params;
 *   params.load("/path/to/params.yaml");
 *   然后将 params 传入 Ten_set_detect、Ten_set_plane、Ten_set_result 构造函数
 */
struct KfsParams
{
    // ========== detect (set_detect.h) ==========
    int   cloud_depth_min      = 200;
    int   cloud_depth_max      = 2000;
    std::string yolo_model_path;
    std::string yolo_device    = "cpu";
    float yolo_conf_threshold  = 0.75f;
    float yolo_nms_threshold   = 0.75f;
    float yolo_score_threshold = 0.75f;

    // ========== plane (set_plane.h) ==========
    float box_size            = 0.35f;
    float leaf_size_xy        = 0.010f;
    float leaf_size_z         = 0.005f;
    float distance_threshold  = 0.016f;
    int   max_iterations      = 1000;
    float cluster_tolerance   = 0.012f;
    int   min_plane_inliers   = 100;

    // ========== result (set_result.h) ==========
    double ema_alpha = 0.15;

    // ========== bias_to_target (set_bias_to_result 输入) ==========
    Ten::XYZRPY bias_to_target;

    /**
     * @brief 从 YAML 文件加载所有参数
     * @param yaml_path  YAML 文件绝对路径
     * @return 是否加载成功
     */
    bool load(const std::string& yaml_path)
    {
        cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            std::cerr << "[KfsParams] 无法打开 YAML: " << yaml_path << std::endl;
            return false;
        }

        // ---- detect ----
        {
            cv::FileNode fn = fs["detect"];
            if (!fn.empty())
            {
                read_if_exists(fn, "cloud_depth_min",      cloud_depth_min);
                read_if_exists(fn, "cloud_depth_max",      cloud_depth_max);
                read_if_exists(fn, "yolo_model_path",      yolo_model_path);
                read_if_exists(fn, "yolo_device",          yolo_device);
                read_if_exists(fn, "yolo_conf_threshold",  yolo_conf_threshold);
                read_if_exists(fn, "yolo_nms_threshold",   yolo_nms_threshold);
                read_if_exists(fn, "yolo_score_threshold", yolo_score_threshold);
            }
        }

        // ---- plane ----
        {
            cv::FileNode fn = fs["plane"];
            if (!fn.empty())
            {
                read_if_exists(fn, "box_size",            box_size);
                read_if_exists(fn, "leaf_size_xy",        leaf_size_xy);
                read_if_exists(fn, "leaf_size_z",         leaf_size_z);
                read_if_exists(fn, "distance_threshold",  distance_threshold);
                read_if_exists(fn, "max_iterations",      max_iterations);
                read_if_exists(fn, "cluster_tolerance",   cluster_tolerance);
                read_if_exists(fn, "min_plane_inliers",   min_plane_inliers);
            }
        }

        // ---- result ----
        {
            cv::FileNode fn = fs["result"];
            if (!fn.empty())
            {
                read_if_exists(fn, "ema_alpha", ema_alpha);
            }
        }

        // ---- bias_to_target ----
        {
            cv::FileNode fn = fs["bias_to_target"];
            if (!fn.empty())
            {
                read_if_exists(fn, "x",     bias_to_target._xyz._x);
                read_if_exists(fn, "y",     bias_to_target._xyz._y);
                read_if_exists(fn, "z",     bias_to_target._xyz._z);
                read_if_exists(fn, "roll",  bias_to_target._rpy._roll);
                read_if_exists(fn, "pitch", bias_to_target._rpy._pitch);
                read_if_exists(fn, "yaw",   bias_to_target._rpy._yaw);
            }
        }

        fs.release();
        std::cout << "[KfsParams] 参数加载成功: " << yaml_path << std::endl;
        return true;
    }

private:
    // ---- 辅助：仅当键存在时才读取 ----
    template<typename T>
    static void read_if_exists(const cv::FileNode& fn, const std::string& key, T& val)
    {
        if (!fn[key].empty())
            fn[key] >> val;
    }

    // std::string 特化
    static void read_if_exists(const cv::FileNode& fn, const std::string& key, std::string& val)
    {
        if (!fn[key].empty())
            fn[key] >> val;
    }
};

}  // namespace Ten::kfs_locator

#endif

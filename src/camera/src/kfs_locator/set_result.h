#ifndef __SET_RESULT_H_
#define __SET_RESULT_H_
#include "set_plane.h"
#include "set_detect.h"
#include "../camera.h"


namespace Ten::kfs_locator
{
struct result
{
    double bia_radian = 0.0;     // 目标面的法向量到预设面的法向量的角度
    double x = 0.0;         
    double y = 0.0;           
    double z = 0.0; 
};


class Ten_set_result
{
public:
    Ten_set_result(rs2_intrinsics color_intr)
    :   input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        filter_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        plane_cloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
        color_intr_ = color_intr;
    }

    // 执行流程 ---------------------------------------------------------------
    // 1 设置初始点云列表
    bool preprocess(Ten::camera_frame& frame);

    // 2 get_input_clouds()

    // 3 核心提取面函数
    bool postprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr = nullptr);

    // 4 设置结果
    result set_result();
    // 执行流程 ---------------------------------------------------------------

    // 取接口
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> get_input_clouds()
    {
        return  input_clouds;
    }
    const Ten::kfs_locator::Plane_Info get_plane_info() 
    {
        return plane_info;
    }
    const std::string get_state() 
    {
        return state;
    }
    const std::vector<cv::Rect> get_rois()
    {
        return rois_;
    }

private:
    // 参数
    rs2_intrinsics color_intr_;
    // 处理器
    Ten::kfs_locator::Ten_set_detect SET_DETECT_;
    Ten::kfs_locator::Ten_set_plane SET_PLANE_;
    // 中间变量
    Ten::kfs_locator::Plane_Info plane_info;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> input_clouds;
    std::vector<cv::Rect> rois_;                // 存储最新一次检测的 ROI 框
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud;
    Eigen::Vector3d key_center;     // 体中心点

    // 时域滤波（抑制RANSAC帧间抖动）
    Eigen::Vector3d filtered_normal_ = Eigen::Vector3d::UnitZ(); // EMA 滤波后的法向量
    bool normal_initialized_ = false;                            // 首帧初始化标记
    static constexpr double kEmaAlpha = 0.15;                    // EMA 系数：越小越平滑（0.05~0.3）

    std::string state = "off";

    // 功能函数 计算偏差角度， 确保plane_info已被写入信息
    double calc_deviation_angle(const Eigen::Vector3d& target_plane);

    // 法向量 EMA 时域滤波，抑制 RANSAC 帧间抖动
    void apply_normal_ema();
};      // class Ten_set_result

inline bool Ten_set_result::preprocess(Ten::camera_frame& frame)
{
    if (frame.bgr_image.empty() || frame.depth_image.empty())
    {
        state = "frame.bgr_image.empty() || frame.depth_image.empty()";
        return false;
    }

    // 设置点云
    rois_ = SET_DETECT_.set_roi_detect(frame.bgr_image);
    bool is_set = SET_DETECT_.set_pcl_clouds(frame.depth_image, color_intr_, rois_, input_clouds);
    if (!is_set) 
    {
        state = "set_pcl_clouds error!";
        return false;
    }
    state = "preprocess ok";
    return true;
}

inline bool Ten_set_result::postprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr)
{
    // 选择输入点云
    if (input_cloud_ptr)
    {
        input_cloud = input_cloud_ptr;
    }
    else
    {
        if (input_clouds.empty())
        {
            state = "postprocess: input_clouds is empty";
            return false;
        }

        const auto& front_cloud = input_clouds.front();
        if (!front_cloud)
        {
            state = "postprocess: input_clouds front cloud is nullptr";
            return false;
        }
        input_cloud = front_cloud;
    }

    // 滤波器
    bool is_filtted = SET_PLANE_.cloud_filter(input_cloud,filter_cloud);      // 设置点云
    if (!is_filtted)
    {
        state = "cloud_filter error!";
        return false;
    }
    // 提取平面和中心点，法向量
    bool is_plane_flitted = SET_PLANE_.Plane_fitter(filter_cloud, plane_cloud, plane_info);
    if (!is_plane_flitted)
    {
        state = "Plane_fitter error!";
        return false;
    }

    // 法向量时域滤波
    apply_normal_ema();

    // 方形拟合
    SET_PLANE_.compute_Center(plane_cloud,plane_info);
    // 赋值底面中心点
    key_center = SET_PLANE_.cal_base_point(plane_info);
    state = "ok";
    return true;
}

inline void Ten_set_result::apply_normal_ema()
{
    if (!normal_initialized_)
    {
        filtered_normal_ = plane_info.plane_normal.normalized();
        normal_initialized_ = true;
    }
    else
    {
        // 先统一方向（防止法向量翻转导致 EMA 振荡）
        Eigen::Vector3d raw_n = plane_info.plane_normal.normalized();
        if (raw_n.dot(filtered_normal_) < 0.0)
            raw_n = -raw_n;
        filtered_normal_ = (kEmaAlpha * raw_n + (1.0 - kEmaAlpha) * filtered_normal_).normalized();
    }
    // 用滤波后的法向量替换原始值
    plane_info.plane_normal = filtered_normal_;
}

inline double Ten_set_result::calc_deviation_angle(const Eigen::Vector3d& target_plane)
{
    Eigen::Vector3d normal = plane_info.plane_normal;
    normal.normalize();

    // 计算向量点积
    double dot_product = normal.dot(target_plane);
    // 修复浮点误差，限制在 [-1, 1] 避免 acos 出现 NaN
    dot_product = std::max(-1.0, std::min(1.0, dot_product));

    // 原始夹角（弧度，范围 0 ~ M_PI）
    double rad_angle = std::acos(dot_product);
    // 1. 取最小夹角，约束到 0 ~ M_PI/2（对应 0° ~ 90°）
    rad_angle = std::min(rad_angle, M_PI - rad_angle);

    // 2. 大于 45°(M_PI/4) 则使用 90°(M_PI/2) - 当前弧度
    if (rad_angle > M_PI / 4)
    {
        rad_angle = M_PI / 2 - rad_angle;
    }

    return rad_angle;
}

inline result Ten_set_result::set_result()
{
    result out;

    // ── 1. 底面方程：n·(X - key_center) = 0 ──
    //     顶面法向量 = 底面法向量（方块上下底面平行）
    const Eigen::Vector3d n = plane_info.plane_normal.normalized();

    // ── 2. 将光心（原点）投影到底面 ──
    //     投影公式：P_proj = (key_center·n) * n
    const double d = key_center.dot(n);               // 光心到底面的有向距离
    const Eigen::Vector3d P_proj = d * n;             // 光心在底面上的投影点

    // ── 3. 在底面上建立正交 2D 坐标系 ──
    //     x_axis：相机 X 轴 (1,0,0) 投影到底面 → 面内水平方向
    //     y_axis：相机 Y/Z 投影到底面 → 共线（正交于 x_axis 的面内方向）
    //             用 n × x_axis 保证正交
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - n.x() * n;
    x_axis.normalize();
    Eigen::Vector3d y_axis = n.cross(x_axis);         // 右手系，在底面内 ⊥ x_axis

    // ── 4. 计算底面中心在投影点坐标系中的坐标 ──
    const Eigen::Vector3d offset = key_center - P_proj; // 底面内：投影点 → 底面中心
    out.x = offset.dot(x_axis);
    out.y = offset.dot(y_axis);
    out.z = d;                                        // 光心到底面的距离（深度）

    // ── 5. 偏差角度 ──
    out.bia_radian = calc_deviation_angle(Eigen::Vector3d::UnitZ());

    return out;
}

} // namespace Ten::kfs_locator

#endif
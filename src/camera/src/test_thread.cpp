#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>
#include <atomic>
#include <sstream>
#include <thread>
#include "threadpool.h"
#include "camera.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "./kfs_locator/set_result.h"
#include "./kfs_locator/debug_pcl.h"

void test()
{
    Ten::Ten_camera& _CAMERA_ = Ten::Ten_camera::GetInstance();
    _CAMERA_.reset_camera_depth(640, 480,30);
    rs2_intrinsics color_intr = _CAMERA_.get_color_intrinsics();    // 彩色内参 → 绘图用
    Ten::kfs_locator::Ten_set_result plane_fiter(color_intr);
    Ten::XYZRPY bias;

    while (ros::ok())
    {
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

        Ten::camera_frame frame = _CAMERA_.camera_read_depth();
        // 前处理
        bool is_pre_ok = plane_fiter.preprocess(frame);
        // std::cout << "state: " <<  plane_fiter.get_state() << std::endl;
        if (is_pre_ok)
        {
            // 后处理
            bool is_post_ok = plane_fiter.postprocess();        // 置空输入点云，使用点云列表中的第一个点云
            // std::cout << "state: " <<  plane_fiter.get_state() << std::endl;
            if (is_post_ok)
            {
                // 设置结果
                Ten::kfs_locator::result out = plane_fiter.set_result(bias);
                // std::cout << "angle: " <<  ((out.bia_radian) * 180.0 / M_PI) << std::endl;
                // std::cout << "res.x: " <<  out.x << std::endl;
                // std::cout << "res.y: " <<  out.y << std::endl;
                // std::cout << "res.z: " <<  out.z << std::endl;
            }
        }

        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_time = end_time - start_time;
        std::cout << "Elapsed time: " << elapsed_time.count()* 1000  << " ms" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
    }
}


void test_thread()
{
    Ten::Ten_camera& _CAMERA_ = Ten::Ten_camera::GetInstance();
    _CAMERA_.reset_camera_depth(640, 480,30);
    rs2_intrinsics color_intr = _CAMERA_.get_color_intrinsics();    // 彩色内参 → 绘图用
    Ten::kfs_locator::Ten_set_result plane_fiter(color_intr);
    Ten::XYZRPY bias;

    Ten::threadpool_test::ThreadPool pool(4,8); // 创建一个线程池，最多同时运行4个线程,8个任务队列最大长度

    while (ros::ok())
    {
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

        Ten::camera_frame frame = _CAMERA_.camera_read_depth();

        std::chrono::steady_clock::time_point pre_start_time = std::chrono::steady_clock::now();
        // 前处理
        bool is_pre_ok = plane_fiter.preprocess(frame);
        if (is_pre_ok)
        {
            // 提交后处理任务到线程池
            pool.enqueue([&plane_fiter, &bias]() {
                bool is_post_ok = plane_fiter.postprocess();        // 置空输入点云，使用点云列表中的第一个点云
                if (is_post_ok)
                {
                    // 设置结果
                    Ten::kfs_locator::result out = plane_fiter.set_result(bias);
                }
            });
        }

        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_time = end_time - pre_start_time;
        std::chrono::duration<double> pre_elapsed_time = pre_start_time - start_time;
        std::cout << "get_frame time: " << pre_elapsed_time.count()* 1000  << " ms" << std::endl;
        std::cout << "process time: " << elapsed_time.count()* 1000  << " ms" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_thread");
    test_thread();
    return 0;
}
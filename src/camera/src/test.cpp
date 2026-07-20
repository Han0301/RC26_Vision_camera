#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <ros/ros.h>
#include "camera.h"
#include "apriltag_detect/apriltag_detector.h"
#include "apriltag_detect/apriltag_debug.h"

// ---------- 主循环 ----------
void test()
{
    // D435 彩色流: 1920×1080 最高 30fps, 1280×720 最高 90fps
    Ten::Ten_camera& camera = Ten::Ten_camera::GetInstance(1280, 720, 60);
    Ten::apriltag_detect::AprilTagDetector Apriltag_d;

    ros::NodeHandle nh;
    Ten::apriltag_detect::AprilTagDebug debug(Apriltag_d, nh, "/camera/apriltag_debug");

    // FPS 统计
    ros::Time last_time = ros::Time::now();
    double fps = 0.0;

    while (ros::ok())
    {
        // 1. 读取相机帧
        cv::Mat frame = camera.camera_read();
        if (frame.empty())
        {
            ros::Duration(0.01).sleep();
            continue;
        }

        // 2. 计算 FPS
        ros::Time now = ros::Time::now();
        double dt = (now - last_time).toSec();
        if (dt > 0.0)
            fps = 0.9 * fps + 0.1 / dt;  // 指数平滑
        last_time = now;

        // 3. AprilTag 检测
        bool detected = Apriltag_d.detect(frame);

        // 终端打印检测状态 (仅状态变化时打印)
        {
            static bool last_detected = false;
            if (detected != last_detected)
            {
                if (detected)
                    std::cout << "[DETECT] found " << Apriltag_d.detections().size()
                              << " tag(s)" << std::endl;
                else
                    std::cout << "[DETECT] no tag" << std::endl;
                last_detected = detected;
            }
        }

        // 4. 发布调试图像（绘制检测框 + FPS 等覆盖信息）
        debug.publishDebugImage(frame, Apriltag_d.detections(), fps);

        // 5. ROS 回调
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    test();
    return 0;
}
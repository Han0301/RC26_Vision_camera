#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <stdexcept>
#include <ros/ros.h>
#include "camera.h"
#include "./PnP/pnp_main.h"
#include "./PnP/pnp_debug.h"

void test_pnp(ros::NodeHandle& nh)
{
    Ten::Ten_camera& _CAMERA_ = Ten::Ten_camera::GetInstance();
    _CAMERA_.reset_camera_depth(640, 480, 30);
    
    rs2_intrinsics color_intr = _CAMERA_.get_color_intrinsics();

    Ten::KFS::kfsPnpRosNode pnp_hander(color_intr);
    Ten::KFS::DebugDrawer pnp_debug;

    pnp_debug.init();  

    while (ros::ok())
    {
        Ten::camera_frame frame = _CAMERA_.camera_read_depth();
        if (frame.bgr_image.empty() || frame.depth_image.empty())
        {
            std::cout << "图像为空！" << std::endl;
            continue;
        }

        Ten::KFS::kfsPnpOutput pnp_result = pnp_hander.processOneFrame(frame.bgr_image, frame.depth_image);
        pnp_debug.draw(frame.bgr_image, pnp_result, color_intr);

        double x, y, z, yaw;
        Ten::XYZRPY center = pnp_hander.get_lastest_center();
        std::cout << "解算结果：" << x << " " << y << " " << z << std::endl;

        char key = cv::waitKey(1);
        if (key == 27) break;

        ros::spinOnce();
    }

    pnp_debug.shutdown();
    cv::destroyAllWindows();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_pnp_node");
    ros::NodeHandle nh;
    test_pnp(nh);
    return 0;
}
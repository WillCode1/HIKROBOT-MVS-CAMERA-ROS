#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "hikrobot_camera.hpp"

// 剪裁选项
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
    #define FIT_min_x 420
    #define FIT_min_y 70
    #define FIT_max_x 2450
    #define FIT_max_y 2000
#endif

using namespace std;
using namespace cv;

// 全局变量定义（在命名空间内）
namespace camera {
    int g_width;
    int g_height;
    cv::Mat frame;
    bool frame_empty = 0;
    pthread_mutex_t mutex;
}

int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hikrobot_camera");

    // 创建相机对象（传入节点）
    camera::Camera MVS_cap(node);

    // 图像传输发布器
    image_transport::ImageTransport main_cam_image(node);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hikrobot_camera/rgb", 1000);

    // 消息对象
    sensor_msgs::msg::Image image_msg;
    sensor_msgs::msg::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;

    // 主循环频率 (100 Hz)
    rclcpp::Rate loop_rate(100);

    while (rclcpp::ok())
    {
        loop_rate.sleep();
        // 处理回调（例如服务、参数更新等）
        rclcpp::spin_some(node);

        cv::Mat src;
        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }

#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x, FIT_min_y, FIT_max_x - FIT_min_x, FIT_max_y - FIT_min_y);
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else
        cv_ptr->image = src;
#endif

        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = node->now();  // 使用 ROS2 时间
        image_msg.header.frame_id = "hikrobot_camera";

        camera_info_msg.header.frame_id = image_msg.header.frame_id;
        camera_info_msg.header.stamp = image_msg.header.stamp;

        image_pub.publish(image_msg, camera_info_msg);
    }

    rclcpp::shutdown();
    return 0;
}
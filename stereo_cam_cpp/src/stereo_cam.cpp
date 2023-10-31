#include <iostream>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikcam.hpp"

enum class StereoCam: int
{
    LEFT_FOWARD,
    RIGHT_FOWARD,
    LEFT_DOWN,
    RIGHT_DOWN
};

void GetImageAndPublish(std::vector<image_transport::CameraPublisher>& publisher, HikCam & camera, int flag, rclcpp::Time t){
    cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  
    bool success;
    sensor_msgs::msg::CameraInfo camera_info_msg;

    camera.getSingleImage(cv_ptr->image, success, flag);
    if (success)
    {
        sensor_msgs::msg::Image image_msg;
        //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = t; 
        //image_msg.header.frame_id = to_string(idRear);
        camera_info_msg.header.frame_id = image_msg.header.frame_id;
        camera_info_msg.header.stamp = image_msg.header.stamp;
        publisher[flag].publish(image_msg, camera_info_msg);
    }
    else
        throw std::runtime_error{"fails to render image"};
}


int main(int argc, char **argv){
    //********** variables    **********/
    HikCam mycamRig;
    if(!mycamRig.init()){
        return -1;
    }

    std::string stopic[4] = {"left_forward", "right_forward", "left_down", "right_down"};
    //********** rosnode init **********/
    rclcpp::init(argc, argv);
    auto node_camera_stereo = rclcpp::Node::make_shared("stereo_cam");
    bool btrigger_fisheyes = true;
    int camera_rate = 10;
    node_camera_stereo->declare_parameter("btrigger_fisheyes", btrigger_fisheyes);
    node_camera_stereo->declare_parameter("camera_rate", camera_rate);

    image_transport::ImageTransport main_cam_image(node_camera_stereo);
    std::vector<image_transport::CameraPublisher> image_publishers;
    for (int i = 0; i < MAX_CAMERA_NUM; ++i){
        image_publishers.push_back(main_cam_image.advertiseCamera("/stereo_cam"+stopic[i], 1000));
    }

    //********** 10 Hz        **********/
    rclcpp::Rate loop_rate(camera_rate);
    while (rclcpp::ok()){
        if (btrigger_fisheyes){   
            bool vbsuccess[MAX_CAMERA_NUM];
            std::thread* vthreads[MAX_CAMERA_NUM];
            const rclcpp::Time curTime = node_camera_stereo->now();
            const auto tp_1 = std::chrono::steady_clock::now();
            for (int i = 0; i < MAX_CAMERA_NUM; ++i)
            {
                vthreads[i]= new std::thread(GetImageAndPublish, std::ref(image_publishers), std::ref(mycamRig), i, curTime);
            }
            for (auto imgThread: vthreads)
            {
                imgThread->join();
            }
            const auto tp_2 = std::chrono::steady_clock::now();
            auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            // ROS_INFO("capturing taking: %f", track_time);
        }
        else
        {
            const auto tp_1 = std::chrono::steady_clock::now();
            GetImageAndPublish(image_publishers, mycamRig, (int)StereoCam::RIGHT_FOWARD, node_camera_stereo->now());
            const auto tp_2 = std::chrono::steady_clock::now();
            auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            // ROS_INFO("capturing image #1 taking: %f", track_time);
        }
        loop_rate.sleep();
        rclcpp::spin_some(node_camera_stereo);
    }
}
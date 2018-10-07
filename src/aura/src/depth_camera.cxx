#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>
#include <std_msgs/Float64MultiArray.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
#include <map>
#include <algorithm>

const static std::string name_space = "robot0";


void get_depth_image(const sensor_msgs::Image &img) {
    cv::Mat frame;
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
    frame = img_ptr->image;
    cv::imshow("f", frame);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth_camera");
    ros::NodeHandle node_handle;
    ros::Subscriber depth_subscriber = node_handle.subscribe("/" + name_space + "/camera_depth/depth/image_raw", 10,
                                                             get_depth_image);
    ros::spin();
}
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


std::string name_space = "aura1";
ros::Publisher dead_victim_publisher;
cv::Scalar lower_color1(0, 0, 0);
cv::Scalar upper_color1(0, 20, 20);
cv::Scalar lower_color2(0, 0, 210);
cv::Scalar upper_color2(0, 0, 255);


void get_image(const sensor_msgs::Image &img) {
    cv::Mat frame, frame2;
    cv::UMat frame1, frame_blur, frame_hsv1, frame_hsv2, mask1, mask2, final_frame1, final_frame2, frame_edge, frame_gray;
    cv::UMat mask3, mask4, mask5;
    std::vector<std::vector<cv::Point>> contours;
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    frame = img_ptr->image;
    cv::GaussianBlur(frame, frame_blur, cv::Size(5, 5), 0);
    cv::cvtColor(frame_blur, frame_hsv1, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv1, lower_color1, upper_color1, mask1);
    cv::bitwise_not(frame, frame1);
    cv::medianBlur(frame1, frame1, 3);
    cv::cvtColor(frame1, frame_hsv2, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv2, lower_color2, upper_color2, mask2);
    cv::bitwise_or(mask1, mask2, mask3);
    cv::medianBlur(mask3, mask3, 3);
    cv::bitwise_xor(mask1, mask2, mask4);
    cv::bitwise_or(mask4, mask3, mask5);
    cv::medianBlur(mask5, mask5, 3);
    cv::bitwise_and(frame1, frame1, final_frame2, mask5);
    cv::GaussianBlur(final_frame2, final_frame2, cv::Size(3, 3), 0);
    cv::cvtColor(final_frame2, frame_gray, cv::COLOR_BGR2GRAY);
    cv::Canny(frame_gray, frame_edge, 100, 200);
    cv::findContours(frame_edge, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    final_frame2.copyTo(frame2);
    cv::waitKey(1);
    cv::imshow("dead frame1", frame_edge);
    if (contours.empty())
        return;
    std::map<double, std::vector<cv::Point>> contours_area;
    std::vector<double> areas;
    for (auto const &cnt : contours) {
        double d = cv::contourArea(cnt);
        contours_area[d] = cnt;
        areas.push_back(d);
    }
    std::sort(areas.rbegin(), areas.rend());
    std::vector<cv::Point> main_contour = contours_area[areas[0]];
    cv::Rect main_rect = cv::boundingRect(main_contour);
    if (main_rect.height < 24) return;
    std_msgs::Float64MultiArray info_array;
    info_array.data.push_back(main_rect.x);
    info_array.data.push_back(main_rect.y);
    info_array.data.push_back(main_rect.width);
    info_array.data.push_back(main_rect.height);
    dead_victim_publisher.publish(info_array);
    cv::rectangle(frame2, main_rect, cv::Scalar(255, 0, 0), 2);
    cv::imshow("dead frame", frame2);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "dead_victim_detector");
    ros::NodeHandle node_handler;
    node_handler.getParam(ros::this_node::getName() + "/robotname", name_space);
    dead_victim_publisher = node_handler.advertise<std_msgs::Float64MultiArray>("/" + name_space + "/victims/dead",
                                                                                1000);
    ros::Subscriber rgb_subscriber = node_handler.subscribe("/" + name_space + "/camera_ros/image", 1000, get_image);
    ros::spin();
}

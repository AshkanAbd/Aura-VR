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
ros::Publisher dead_victim_publisher;
cv::Scalar lower_color1(0, 0, 0);
cv::Scalar upper_color1(0, 50, 50);
cv::Scalar lower_color2(0, 0, 100);
cv::Scalar upper_color2(0, 50, 255);


void get_image(const sensor_msgs::Image &img) {
    cv::Mat frame, frame1, frame_blur, frame_hsv1, frame_hsv2, mask1, mask2, final_frame1, final_frame2, frame_edge, frame_gray;
    std::vector<std::vector<cv::Point>> contours;
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    frame = img_ptr->image;
    cv::medianBlur(frame, frame_blur, 5);
    cv::cvtColor(frame_blur, frame_hsv1, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv1, lower_color1, upper_color1, mask1);
    cv::bitwise_not(frame, frame1);
    cv::cvtColor(frame1, frame_hsv2, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv2, lower_color2, upper_color2, mask2);
    cv::bitwise_and(frame1, frame1, final_frame1, mask1);
    cv::bitwise_and(final_frame1, final_frame1, final_frame2, mask2);
    cv::cvtColor(final_frame2, frame_gray, cv::COLOR_BGR2GRAY);
    cv::Laplacian(frame_gray, frame_edge, -1);
    cv::findContours(frame_edge, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//    cv::waitKey(1);
//    cv::imshow("dead frame", final_frame2);
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
//    if (main_rect.height < 36) return;
    std_msgs::Float64MultiArray info_array;
    info_array.data.push_back(main_rect.x);
    info_array.data.push_back(main_rect.y);
    info_array.data.push_back(main_rect.width);
    info_array.data.push_back(main_rect.height);
    dead_victim_publisher.publish(info_array);
    cv::rectangle(final_frame2, main_rect, cv::Scalar(255, 0, 0), 2);
//    cv::imshow("dead frame", final_frame2);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "dead_victim_detector_" + name_space);
    ros::NodeHandle node_handler;
    dead_victim_publisher = node_handler.advertise<std_msgs::Float64MultiArray>("/" + name_space + "/victims/dead", 1000);
    ros::Subscriber rgb_subscriber = node_handler.subscribe("/" + name_space + "/camera_depth/rgb/image", 1000,
                                                            get_image);
    ros::spin();
}

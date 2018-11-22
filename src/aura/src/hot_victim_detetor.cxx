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
cv::Mat normal_img;
cv::Mat thermal_img;
ros::Publisher hot_victim_publisher;
cv::Scalar lower_color(0, 30, 30);
cv::Scalar upper_color(20, 255, 255);

void get_normal_image(const sensor_msgs::Image &img) {
    cv::Mat frame, frame1, frame_gray, frame_thresh, frame_filter, frame2, frame_hsv, mask, final_frame2;
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    frame = img_ptr->image;
    cv::bilateralFilter(frame, frame_filter, 9, 75, 75);
    cv::cvtColor(frame_filter, frame_gray, cv::COLOR_BGR2GRAY);
    cv::threshold(frame_gray, frame_thresh, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    frame.copyTo(frame1);
    cv::bitwise_and(frame1, frame1, frame2, frame_thresh);
    cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv, lower_color, upper_color, mask);
    cv::bitwise_and(frame2, frame2, final_frame2, mask);
    normal_img = final_frame2;
}

void get_thermal_image(const sensor_msgs::Image &img) {
    cv::Mat frame, frame1, frame_thresh1, frame_thresh2, frame_gray;
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    frame = img_ptr->image;
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    cv::bilateralFilter(frame_gray, frame1, 9, 75, 75);
    cv::threshold(frame1, frame_thresh1, 180, 255, cv::THRESH_BINARY);
    cv::threshold(frame_thresh1, frame_thresh2, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    thermal_img = frame_thresh2;
}

void process_img() {
    while (true) {
        if (normal_img.data == nullptr || thermal_img.data == nullptr) {
            continue;
        }
        try {
            cv::Mat frame, frame1, frame2, normal_gray, normal_thresh, thermal_img1, frame_edge;
            std::vector<std::vector<cv::Point>> contours;
            cv::cvtColor(normal_img, normal_gray, cv::COLOR_BGR2GRAY);
            cv::threshold(normal_gray, normal_thresh, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
            cv::pyrUp(thermal_img, thermal_img1);
            cv::bitwise_and(thermal_img1, thermal_img1, frame1, normal_thresh);
            cv::medianBlur(frame1, frame2, 5);
            cv::Laplacian(frame2, frame_edge, -1);
            cv::findContours(frame_edge, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            std::map<double, std::vector<cv::Point>> contours_area;
//            cv::waitKey(1);
//            cv::imshow("hot frame", frame2);
            if (contours.empty()) continue;
            std::vector<double> areas;
            for (const auto &contour : contours) {
                double d = cv::contourArea(contour);
                contours_area[d] = contour;
                areas.push_back(d);
            }
            std::sort(areas.rbegin(), areas.rend());
            std::vector<cv::Point> main_contour = contours_area[areas[0]];
            cv::Rect main_rect = cv::boundingRect(main_contour);
//            if (main_rect.height < 36) continue;
            std_msgs::Float64MultiArray info_array;
            info_array.data.push_back(main_rect.x);
            info_array.data.push_back(main_rect.y);
            info_array.data.push_back(main_rect.width);
            info_array.data.push_back(main_rect.height);
            hot_victim_publisher.publish(info_array);
            cv::rectangle(frame2, main_rect, cv::Scalar(255, 0, 0), 2);
//            cv::imshow("hot frame", frame2);
        } catch (std::exception e) {
            std::cout << e.what() << std::endl;
        }
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "hot_victim_detector_" + name_space);
    ros::NodeHandle node_handle;
    ros::Subscriber rgb_subscriber = node_handle.subscribe("/" + name_space + "/camera_ros/image", 1000,
                                                           get_normal_image);
    ros::Subscriber thermal_subscriber = node_handle.subscribe("/" + name_space + "/camera/thermal/image_raw", 1000,
                                                               get_thermal_image);
    hot_victim_publisher = node_handle.advertise<std_msgs::Float64MultiArray>("/" + name_space + "/victims/hot", 1000);
    std::thread process_thread(process_img);
    ros::spin();
}

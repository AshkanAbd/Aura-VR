#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <map>
#include <cstddef>
#include "xtensor/xio.hpp"
#include "xtensor/xview.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xtensor.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xadapt.hpp"


#define ul unsigned long

const std::string robot1 = "aura1";
const std::string robot2 = "aura2";
const std::string robot3 = "aura3";
const std::string robot4 = "aura4";
const int robot1_id = 1;
const int robot2_id = 2;
const int robot3_id = 3;
const int robot4_id = 4;

// find type
std::vector<int8_t> _v = {1, 2, 3, 4, 5, 6, 7};
std::vector<size_t> _s = {7};
auto _x = xt::adapt(_v, _s);
typedef decltype(_x) core_type;
typedef decltype(_x.storage()) storage_type;
typedef decltype(_x.shape()) size_type;
//

core_type core_map(_v, _s);
ul size = -1;
bool start = false;
nav_msgs::OccupancyGrid map_info;
nav_msgs::Odometry odometry;
std::map<ul, int> robot_evolution;
std::map<ul, int> tolerance_zero;
std::map<ul, int> tolerance_one;


void build_core_map(const nav_msgs::OccupancyGrid &map/*, const nav_msgs::Odometry &odom*/, int robot_id);

void publish_core(ros::Publisher core_publisher, ros::Rate rate);

void get_map(const nav_msgs::OccupancyGrid &map, const nav_msgs::Odometry &odom);

void get_map(const nav_msgs::OccupancyGrid &map/*, const nav_msgs::Odometry &odom*/) {
//    std::string frame_id = odom.header.frame_id;
//    int robot_id = 0;
//    if (frame_id.find(robot1) != std::string::npos) {
//        robot_id = robot1_id;
//    } else if (frame_id.find(robot2) != std::string::npos) {
//        robot_id = robot2_id;
//    } else if (frame_id.find(robot3) != std::string::npos) {
//        robot_id = robot3_id;
//    } else {
//        robot_id = robot4_id;
//    }
//    odometry = odom;
    map_info = map;
    build_core_map(map/*, odom*/, 1);
}

void build_core_map(const nav_msgs::OccupancyGrid &map/*, const nav_msgs::Odometry &odom*/, int robot_id) {
    std::vector<std::size_t> vec_size = {map.data.size()};
    if (!start) {
        size = map.data.size();
        core_map = (xt::adapt(map.data, vec_size));
        std::cout << "copy" << std::endl;
        start = true;
    } else {
        try {
            auto new_map = xt::adapt(map.data, vec_size);
            std::vector<ul> zeros_vec = xt::where((new_map > -1) && (new_map < 1))[0];
            ul *zeros = (ul *) malloc(zeros_vec.size() * sizeof(ul));
            std::copy(zeros_vec.begin(), zeros_vec.end(), zeros);
            std::vector<ul> ones_vec = xt::where(new_map > 90)[0];
            ul *ones = (ul *) malloc(ones_vec.size() * sizeof(ul));
            std::copy(ones_vec.begin(), ones_vec.end(), ones);
            ul i = 0;
            for (; i < zeros_vec.size(); i++) {
                ul coordinate = *(zeros + i);
                if (core_map.operator()(coordinate) == -1) {
                    core_map.operator()(coordinate) = 0;
                    robot_evolution[coordinate] = robot_id;
                } else if (core_map.operator()(coordinate) == 100) {
                    if (robot_evolution.find(coordinate) != robot_evolution.end()
                        && robot_evolution[coordinate] == robot_id) {
                        core_map.operator()(coordinate) = 0;
                    } else {
                        tolerance_zero[coordinate]++;
                    }
                }
            }
            for (i = 0; i < ones_vec.size(); i++) {
                ul coordinate = *(ones + i);
                if (core_map.operator()(coordinate) == -1) {
                    core_map.operator()(coordinate) = 100;
                    robot_evolution[coordinate] = robot_id;
                } else if (core_map.operator()(coordinate) == 0) {
                    if (robot_evolution.find(coordinate) != robot_evolution.end()
                        && robot_evolution[coordinate] == robot_id) {
                        core_map.operator()(coordinate) = 100;
                    } else {
                        tolerance_one[coordinate]++;
                    }
                }
            }
            ul *end = (ul *) malloc((tolerance_one.size() + tolerance_zero.size()) * sizeof(ul));
            ul end_i = 0;
            for (const auto pair:tolerance_one) {
                if (pair.second > 10 && core_map.operator()(pair.first) == 0) {
                    core_map.operator()(pair.first) = 100;
                    *(end + end_i) = pair.first;
                    end_i++;
                }
            }
            for (const auto pair:tolerance_zero) {
                if (pair.second > 10 && core_map.operator()(pair.first) == 100) {
                    core_map.operator()(pair.first) = 0;
                    *(end + end_i) = pair.first;
                    end_i++;
                }
            }
            for (i = 0; i < end_i; i++) {
                ul pair = *(end + i);
                if (tolerance_zero.find(pair) != tolerance_zero.end()) {
                    tolerance_zero.erase(pair);
                }
                if (tolerance_one.find(pair) != tolerance_one.end()) {
                    tolerance_one.erase(pair);
                }
            }
            std::cout << "build" << std::endl;
        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;
        }
    }
}

void publish_core(const ros::Publisher core_publisher, ros::Rate rate) {
    while (ros::ok()) {
        if (!start) {
            rate.sleep();
            continue;
        }
        try {
            nav_msgs::OccupancyGrid data_map;
            data_map.header = map_info.header;
            data_map.info = map_info.info;
            int8_t *data = core_map.storage().data();
            std::vector<int8_t> d(data, data + size);
            data_map.data = d;
            core_publisher.publish(data_map);
            rate.sleep();
        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "core_builder");
    //todo get parameter from roslaunch
    ros::NodeHandle handle;
    ros::Rate rate(5);
    ros::Publisher core_publisher = handle.advertise<nav_msgs::OccupancyGrid>("/core/map", 1000);
    ros::Subscriber subscriber = handle.subscribe("/aura1/map", 1000, get_map);
//    message_filters::Subscriber<nav_msgs::OccupancyGrid> robot1_map(handle, "/" + robot1 + "/map", 1000);
//    message_filters::Subscriber<nav_msgs::OccupancyGrid> robot2_map(handle, "/" + robot2 + "/map", 1000);
//    message_filters::Subscriber<nav_msgs::OccupancyGrid> robot3_map(handle, "/" + robot3 + "/map", 1000);
//    message_filters::Subscriber<nav_msgs::OccupancyGrid> robot4_map(handle, "/" + robot4 + "/map", 1000);
//    message_filters::Subscriber<nav_msgs::Odometry> robot1_odom(handle, "/" + robot1 + "/odom", 1000);
//    message_filters::Subscriber<nav_msgs::Odometry> robot2_odom(handle, "/" + robot2 + "/odom", 1000);
//    message_filters::Subscriber<nav_msgs::Odometry> robot3_odom(handle, "/" + robot3 + "/odom", 1000);
//    message_filters::Subscriber<nav_msgs::Odometry> robot4_odom(handle, "/" + robot4 + "/odom", 1000);
//    message_filters::TimeSynchronizer<nav_msgs::OccupancyGrid, nav_msgs::Odometry> sync1(robot1_map, robot1_odom, 1000);
//    message_filters::TimeSynchronizer<nav_msgs::OccupancyGrid, nav_msgs::Odometry> sync2(robot2_map, robot2_odom, 1000);
//    message_filters::TimeSynchronizer<nav_msgs::OccupancyGrid, nav_msgs::Odometry> sync3(robot3_map, robot3_odom, 1000);
//    message_filters::TimeSynchronizer<nav_msgs::OccupancyGrid, nav_msgs::Odometry> sync4(robot4_map, robot4_odom, 1000);
//    sync1.registerCallback(&get_map);
//    sync2.registerCallback(&get_map);
//    sync3.registerCallback(&get_map);
//    sync4.registerCallback(&get_map);
    std::thread publish_thread(publish_core, core_publisher, rate);
    ros::spin();
}

#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <map>
#include <sstream>
#include <set>
#include <vector>
#include <cstring>
#include <math.h>
#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <eigen3/Eigen/Eigen>
#include <Python.h>

#define ul unsigned long
#define int_pair std::pair<int,int>

namespace py = pybind11;


py::int_ get_euclidean_distance(int x1, int y1, int x2, int y2) {
    return py::int_(static_cast<int>(sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2))));
}

void builder(Eigen::Ref<Eigen::VectorXd> core_map, const std::vector<ul> &coordinates, const int &robot_pose,
             py::dict &&node_map, const int &condition1, const int &condition2, const std::string &robot_id,
             int map_width) {
    ul *arr = (ul *) malloc(sizeof(ul) * coordinates.size());
    int robot_x = robot_pose / map_width;
    int robot_y = robot_pose % map_width;
    std::copy(coordinates.begin(), coordinates.end(), arr);
    for (ul i = 0; i < coordinates.size(); ++i) {
        ul coordinate = *(arr + i);
        int coordinate_x = static_cast<int>(coordinate / map_width);
        int coordinate_y = static_cast<int>(coordinate % map_width);
        auto coordinate_i = py::int_(coordinate);
        py::int_ distance = get_euclidean_distance(robot_x, robot_y, coordinate_x, coordinate_y);
        auto value = py::make_tuple(robot_id, distance);
        if (core_map(coordinate) == -1) {
            core_map(coordinate) = condition1;
            node_map[coordinate_i] = value;
        } else if (core_map(coordinate) == condition2) {
            py::tuple old = node_map[coordinate_i];
            py::int_ old_dis(old[1]);
            if (old_dis > distance) {
                core_map(coordinate) = condition1;
                node_map[coordinate_i] = value;
            }
        } else {
            py::tuple old = node_map[coordinate_i];
            py::int_ old_dis(old[1]);
            if (old_dis > distance) {
                core_map(coordinate) = condition1;
                node_map[coordinate_i] = value;
            }
        }
    }
}

std::vector<std::string> get_topics(const std::string &map_topic, const std::string &core_topic) {
    std::array<char, 128> buffer{};
    std::vector<std::string> result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen("rostopic list", "r"), pclose);
    ul index;
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        std::string buf(buffer.data());
        if ((index = buf.find(map_topic)) != std::string::npos) {
            if (buf[index - 1] == '/' && buf[index + 3] == '\n') {
                if (buf.find(core_topic) == std::string::npos) {
                    result.push_back(buf.substr(buf.find('/') + 1, index - 2));
                }
            }
        }
    }
    return result;
}


std::vector<int> get_neighbors(int x, int col_count) {
    std::vector<int> des;
    des.emplace_back(x - col_count);
    des.emplace_back(x + 1);
    des.emplace_back(x + col_count);
    des.emplace_back(x - 1);
    return des;
}

void detect_frontiers(int start, py::set &&frontier_set, Eigen::Ref<Eigen::VectorXd> core_map, int col_count) {
    std::vector<int> queue;
    std::vector<int> visited;
    int start_node(start);
    queue.push_back(start_node);
    int current;
    while (!queue.empty()) {
        current = queue.front();
        queue.erase(queue.begin());
        for (const auto neighbor : get_neighbors(current, col_count)) {
            if ((std::find(queue.begin(), queue.end(), neighbor) == queue.end()) &&
                (std::find(visited.begin(), visited.end(), neighbor) == visited.end())) {
                if (core_map(neighbor) == 0) {
                    queue.push_back(neighbor);
                } else if (core_map(neighbor) == -1) {
                    core_map(neighbor) = 255;
                    frontier_set.add(neighbor);
                }
            }
        }
        visited.push_back(current);
    }
}


PYBIND11_MODULE(functions, m) {
    m.def("builder", &builder, "Core_builder");

    m.def("get_topics", &get_topics, "Get_Topics");

    m.def("detect_frontiers", &detect_frontiers, "BFS_Algorithm");
}

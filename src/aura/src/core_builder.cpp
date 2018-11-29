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

namespace py = pybind11;

using namespace std;

void builder(Eigen::Ref<Eigen::VectorXd> core_map, const std::vector<ul> &coordinates, const int &robot_pose,
             py::dict &&node_map, const int &condition1, const int &condition2, const std::string &robot_id) {
    ul *arr = (ul *) malloc(sizeof(ul) * coordinates.size());
    std::copy(coordinates.begin(), coordinates.end(), arr);
    for (ul i = 0; i < coordinates.size(); ++i) {
        ul coordinate = *(arr + i);
        auto coordinate_i = py::int_(coordinate);
        ul distance = static_cast<unsigned long>(abs(static_cast<int>(coordinate - robot_pose)));
        auto value = py::make_tuple(robot_id, distance);
        if (core_map(coordinate) == -1) {
            core_map(coordinate) = condition1;
            node_map[coordinate_i] = value;
        } else if (core_map(coordinate) == condition2) {
            py::tuple old = node_map[coordinate_i];
            py::str old_id1(old[0]);
            std::string old_id(old_id1);
            if (old_id == robot_id) {
                core_map(coordinate) = condition1;
                node_map[coordinate_i] = value;
            } else {
                if (old[1].ref_count() > distance) {
                    core_map(coordinate) = condition1;
                    node_map[coordinate_i] = value;
                }
            }
        } else {
            py::tuple old = node_map[coordinate_i];
            if (old[1].ref_count() > distance) {
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

PYBIND11_MODULE(core_builder, m) {
    m.def("builder", &builder, "Core_builder");

    m.def("get_topics", &get_topics, "Get_Topics");
}

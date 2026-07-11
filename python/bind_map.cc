// bind_map.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "Map.h"

namespace py = pybind11;
using namespace ORB_SLAM3;

void bind_map(py::module &m) {
    py::class_<Map>(m, "Map")
        .def("get_all_keyframes", &Map::GetAllKeyFrames,
             py::return_value_policy::reference)
        .def("keyframes_in_map", &Map::KeyFramesInMap)
        .def("add_keyframe", &Map::AddKeyFrame, py::arg("kf"))
        .def("erase_keyframe", &Map::EraseKeyFrame, py::arg("kf"))

        .def("get_all_map_points", &Map::GetAllMapPoints,
             py::return_value_policy::reference)
        .def("map_points_in_map", &Map::MapPointsInMap)
        .def("add_map_point", &Map::AddMapPoint, py::arg("mp"))
        .def("erase_map_point", &Map::EraseMapPoint, py::arg("mp"))

        .def("get_id", &Map::GetId)
        .def("is_imu_initialized", &Map::isImuInitialized);
}

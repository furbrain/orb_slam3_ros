// bind_mappoint.cpp
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "MapPoint.h"

namespace py = pybind11;
using namespace ORB_SLAM3;

void bind_mappoint(py::module &m) {
    py::class_<MapPoint>(m, "MapPoint")
        .def_readonly("id", &MapPoint::mnId)
        .def_readonly("first_kf_id", &MapPoint::mnFirstKFid)
        .def("get_world_pos", [](MapPoint &mp) {
            return mp.GetWorldPos(); // goes through mMutexPos internally
        })
        .def("set_world_pos", [](MapPoint &mp, const Eigen::Vector3f &pos) {
            mp.SetWorldPos(pos); // goes through mMutexPos internally
        }, py::arg("pos"))
        .def("get_normal", [](MapPoint &mp) {
            return mp.GetNormal();
        })
        .def("observations", [](MapPoint &mp) {
            return mp.Observations();
        })
        .def("get_observations", [](MapPoint &mp) {
            // returns std::map<KeyFrame*, std::tuple<int,int>> in ORB-SLAM3
            return mp.GetObservations();
        }, py::return_value_policy::reference)
        .def("is_bad", &MapPoint::isBad)
        .def("min_distance_invariance", &MapPoint::GetMinDistanceInvariance)
        .def("max_distance_invariance", &MapPoint::GetMaxDistanceInvariance)
        .def("get_descriptor", [](MapPoint &mp) {
            return mp.GetDescriptor();
        })
        .def("add_observation", &MapPoint::AddObservation)
        .def("erase_observation", &MapPoint::EraseObservation);
}

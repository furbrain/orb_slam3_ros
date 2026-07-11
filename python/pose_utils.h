// python/pose_utils.h
#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <Eigen/Dense>
#include "Thirdparty/Sophus/sophus/geometry.hpp"

namespace py = pybind11;

inline py::object to_rigid_transform(const Sophus::SE3f &pose) {
    static py::object RigidTransform =
        py::module_::import("scipy.spatial.transform").attr("RigidTransform");
    return RigidTransform.attr("from_matrix")(pose.matrix().cast<double>());
}

inline py::object to_rotation(const Eigen::Quaternionf &pose) {
    static py::object Rotation =
        py::module_::import("scipy.spatial.transform").attr("Rotation");
    return Rotation.attr("from_quaternion")(pose.cast<double>());
}
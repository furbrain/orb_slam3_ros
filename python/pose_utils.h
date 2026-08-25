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
    return Rotation.attr("from_matrix")(pose.toRotationMatrix().cast<double>());
}

inline Sophus::SE3f from_rigid_transform(const py::object &RT) {
    // RT.as_matrix() returns a 4x4 numpy array (float64, since scipy works in double)
    Eigen::Matrix4d T = RT.attr("as_matrix")().cast<Eigen::Matrix4d>();

    Eigen::Matrix3f R = T.block<3,3>(0,0).cast<float>();
    Eigen::Vector3f t = T.block<3,1>(0,3).cast<float>();

    return Sophus::SE3f(R, t);
}

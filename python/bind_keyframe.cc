// bind_keyframe.cpp
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "KeyFrame.h"

namespace py = pybind11;
using namespace ORB_SLAM3;

void bind_keyframe(py::module &m) {
    py::class_<KeyFrame>(m, "KeyFrame")
        .def_readonly("id", &KeyFrame::mnId)
        .def_readonly("frame_id", &KeyFrame::mnFrameId)
        .def_readonly("timestamp", &KeyFrame::mTimeStamp)
        .def("get_pose", [](KeyFrame &kf) {
            // Sophus::SE3f -> 4x4 homogeneous matrix, pybind11/eigen.h
            // handles the Eigen::Matrix4f return automatically
            return kf.GetPose().matrix();
        })
        .def("get_pose_inverse", [](KeyFrame &kf) {
            return kf.GetPoseInverse().matrix();
        })
        .def("get_camera_center", [](KeyFrame &kf) {
            return kf.GetCameraCenter(); // Eigen::Vector3f, native conversion
        })
        .def("get_imu_position", [](KeyFrame &kf) {
            return kf.GetImuPosition();
        })
        .def("get_imu_rotation", [](KeyFrame &kf) {
            return kf.GetImuRotation(); // Eigen::Matrix3f
        })
        .def("get_map_point_matches", [](KeyFrame &kf) {
            return kf.GetMapPointMatches(); // vector<MapPoint*>, nulls -> None
        }, py::return_value_policy::reference)
        .def("get_connected_keyframes", &KeyFrame::GetConnectedKeyFrames,
             py::return_value_policy::reference)
        .def("get_covisibles_by_weight", &KeyFrame::GetCovisiblesByWeight,
             py::return_value_policy::reference)
        .def("get_weight", &KeyFrame::GetWeight)
        .def("get_parent", &KeyFrame::GetParent,
             py::return_value_policy::reference)
        .def("is_bad", &KeyFrame::isBad)
        .def("num_keypoints", [](KeyFrame &kf) { return kf.N; })
        .def("get_keypoints_undistorted", [](KeyFrame &kf) {
            // vector<cv::KeyPoint> -> pull (x,y) pairs out for the Python side
            // rather than binding cv::KeyPoint itself
            std::vector<std::pair<float,float>> pts;
            pts.reserve(kf.mvKeysUn.size());
            for (auto &kp : kf.mvKeysUn) pts.emplace_back(kp.pt.x, kp.pt.y);
            return pts;
        });
}

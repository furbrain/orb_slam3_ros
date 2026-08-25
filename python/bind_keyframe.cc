 // bind_keyframe.cpp
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "KeyFrame.h"
#include <opencv2/core/eigen.hpp>
#include "pose_utils.h"

#include "DBoW2/DBoW2/BowVector.h"   // DBoW2

namespace pybind11 { namespace detail {

template <>
struct type_caster<DBoW2::BowVector>
    : map_caster<DBoW2::BowVector, DBoW2::WordId, DBoW2::WordValue> {};

}} // namespace pybind11::detail

namespace py = pybind11;
using namespace ORB_SLAM3;

void bind_keyframe(py::module &m) {
    py::class_<KeyFrame>(m, "KeyFrame")
        .def_readonly("id", &KeyFrame::mnId)
        .def_readonly("frame_id", &KeyFrame::mnFrameId)
        .def_readonly("timestamp", &KeyFrame::mTimeStamp)
        .def_readonly("baseline", &KeyFrame::mb)
        .def_readonly("bowVec", &KeyFrame::mBowVec)
        .def_readonly("featVec", &KeyFrame::mFeatVec)
        .def("get_pose", [](KeyFrame &kf) {
            // Sophus::SE3f -> 4x4 homogeneous matrix, pybind11/eigen.h
            // handles the Eigen::Matrix4f return automatically
            return to_rigid_transform(kf.GetPose());
        })
        .def("get_pose_inverse", [](KeyFrame &kf) {
            return to_rigid_transform(kf.GetPoseInverse());
        })
        .def("set_pose", [](KeyFrame &kf, const py::object &pose) {
            kf.SetPose(from_rigid_transform(pose));
        }, py::arg("pose"))
        .def("get_camera_center", [](KeyFrame &kf) {
            return kf.GetCameraCenter(); // Eigen::Vector3f, native conversion
        })
        .def("get_imu_estimate", [](KeyFrame &kf) {
            return to_rigid_transform(kf.GetPoseFromEstimate());
        })
        .def("get_map_point_matches", [](KeyFrame &kf) {
            return kf.GetMapPointMatches(); // vector<MapPoint*>, nulls -> None
        }, py::return_value_policy::reference)
        .def("add_map_point", &KeyFrame::AddMapPoint, py::arg("mp"), py::arg("idx"))
        .def("erase_map_point",static_cast<void (KeyFrame::*)(MapPoint*)>(&KeyFrame::EraseMapPointMatch), py::arg("mp"))
        .def("replace_map_point", &KeyFrame::ReplaceMapPointMatch, py::arg("idx"), py::arg("mp"))
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
        })
        .def("get_descriptors", [](KeyFrame &kf) {
            // cv::Mat -> Eigen::Matrix<uint8_t, Dynamic, Dynamic>
            Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> desc;
            cv::cv2eigen(kf.mDescriptors, desc);
            return desc;
        })
        .def("get_u_right", [](KeyFrame &kf) {
            return kf.mvuRight; // vector<float>
        })
        .def("get_depth", [](KeyFrame &kf) {
            return kf.mvDepth; // vector<float>
        })
        .def("tlr", [](KeyFrame &kf) {
            return kf.GetRelativePoseTlr().matrix();
        })
        .def("K", [](KeyFrame &kf) {
            Eigen::Matrix3f K;
            K << kf.fx, 0,     kf.cx,
                 0,     kf.fy, kf.cy,
                 0,     0,     1;
            return K; // Eigen::Matrix3f
        })
        .def("dist_coeffs", [](KeyFrame &kf) {
            Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> desc;
            cv::cv2eigen(kf.mDistCoef, desc);
            return desc;
        })
        .def_readonly("aruco_observations", &KeyFrame::mvArucoObservations)
        .def("add_aruco_observation", &KeyFrame::AddArucoObservation, py::arg("obs"))
        .def("clear_aruco_observations", &KeyFrame::ClearArucoObservations)
        .def("update_connections", &KeyFrame::UpdateConnections)
        .def("__repr__", [](KeyFrame &kf) {
            return "<KeyFrame id=" + std::to_string(kf.mnId) + 
                   " timestamp=" + std::to_string(kf.mTimeStamp) + 
                   " num_keypoints=" + std::to_string(kf.N) + 
                   " num_map_points=" + std::to_string(kf.GetNumberMPs()) + ">";
        });
}

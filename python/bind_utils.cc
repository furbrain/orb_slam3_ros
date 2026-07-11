#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "utils.h"
#include "pose_utils.h"
#include <Eigen/Dense>

namespace py = pybind11;
using namespace ORB_SLAM3;

void bind_utils(py::module &m) {
    m.def("load_atlas", &prepare_atlas,
          py::arg("url"), py::arg("strVocFile") = "",
          py::return_value_policy::take_ownership,
          "Load an Atlas and rewire vocabulary/KeyFrameDatabase if strVocFile is given.");

    m.def("merge_atlas", &merge_atlas,
          py::arg("atlas"), py::arg("url"),
          py::return_value_policy::reference,
          "Merge the atlas at url into the given Atlas. Returns the resulting Maps.");

    m.def("get_biggest_map", &get_biggest_map,
          py::arg("atlas"),
          py::return_value_policy::reference,
          "Return the Map with the most KeyFrames in the given Atlas.");

    m.def("align_atlas", &alignAtlas, py::arg("atlas"));
    m.def("align_map", &alignMap, py::arg("map"));
    m.def("ransac_horn_alignment", [](const Eigen::MatrixX3f& src, const Eigen::MatrixX3f&dst, float inlierThreshold, int maxIterations, int minInliersToAccept) {
        RansacResult result = RansacHornAlignmentPy(src, dst, inlierThreshold, maxIterations, minInliersToAccept);
        return py::make_tuple(to_rigid_transform(result.T), result.inlierIndices, result.numInliers);
    }, py::arg("src"), py::arg("dst"), py::arg("inlierThreshold") = 0.05f, py::arg("maxIterations") = 2000, py::arg("minInliersToAccept") = 10, "Perform RANSAC-based Horn alignment between two sets of points. Returns a RansacResult struct.");
}

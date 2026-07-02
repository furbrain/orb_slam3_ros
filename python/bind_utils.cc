#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "utils.h"

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
}

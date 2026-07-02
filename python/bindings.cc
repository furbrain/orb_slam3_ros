// bindings.cpp
#include <pybind11/pybind11.h>
namespace py = pybind11;

void bind_mappoint(py::module &m);
void bind_keyframe(py::module &m);
void bind_map(py::module &m);
void bind_atlas(py::module &m);
void bind_utils(py::module &m);

PYBIND11_MODULE(orb_slam3_py, m) {
    m.doc() = "Python bindings for ORB-SLAM3 Atlas/Map/KeyFrame/MapPoint";
    bind_mappoint(m);
    bind_keyframe(m);
    bind_map(m);
    bind_atlas(m);
    bind_utils(m);
}

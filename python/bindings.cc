// bindings.cpp
#include <pybind11/pybind11.h>
namespace py = pybind11;

void bind_mappoint(py::module &m);
void bind_keyframe(py::module &m);
void bind_map(py::module &m);
void bind_atlas(py::module &m);
void bind_utils(py::module &m);
void bind_kfdb(py::module &m);
void bind_aruco(py::module &m);
void bind_station(py::module &m);
void bind_leg(py::module &m);

PYBIND11_MODULE(orb_slam3_py, m) {
    m.doc() = "Python bindings for ORB-SLAM3 Atlas/Map/KeyFrame/MapPoint";
    bind_mappoint(m);
    bind_keyframe(m);
    bind_map(m);
    bind_atlas(m);
    bind_utils(m);
    bind_kfdb(m);
    bind_aruco(m);
    bind_station(m);
    bind_leg(m);
}

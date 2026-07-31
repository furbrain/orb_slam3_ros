// bind_atlas.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "ArucoObservation.h"

namespace py = pybind11;
using namespace ORB_SLAM3;

void bind_aruco(py::module &m) {
    py::class_<ArucoObservation>(m, "ArucoObservation")
        .def(py::init<>())
        .def_readwrite("id", &ArucoObservation::id)
        .def_readwrite("xLeft", &ArucoObservation::xLeft)
        .def_readwrite("yLeft", &ArucoObservation::yLeft)
        .def_readwrite("xRight", &ArucoObservation::xRight);
}
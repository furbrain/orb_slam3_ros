#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "Surveys.h"

namespace py = pybind11;
using namespace ORB_SLAM3;

void bind_station(py::module &m) {
    py::class_<Station, std::shared_ptr<Station>>(m, "Station")
        .def(py::init<>())
        .def(py::init<const std::string&, const Eigen::Vector3f&>(), 
            "Station constructor",
            py::arg("id"), 
            py::arg("position"))
        .def_readwrite("id", &Station::id)
        .def_readwrite("position", &Station::position)
        .def("__repr__", [](const Station &station) {
            return "<Station id='" + station.id + 
                   "' position=[" + std::to_string(station.position.x()) + 
                   ", " + std::to_string(station.position.y()) + 
                   ", " + std::to_string(station.position.z()) + "]>";
        });
}

void bind_leg(py::module &m) {
    py::class_<Leg>(m, "Leg")
        .def(py::init<std::string, std::string, float, float, float, float, float, float>(), 
            "Leg constructor, uses default values for noise parameters (+/-5mm distance, +/-1 degree azimuth, +/-0.5 degree inclination)",
            py::arg("from_station_id"), 
            py::arg("to_station_id"), 
            py::arg("distance"), 
            py::arg("azimuth"), 
            py::arg("inclination"), 
            py::arg("distance_noise") = 0.0025f, //+-5mm 
            py::arg("azimuth_noise") = 0.00873f, //+-1 degree 
            py::arg("inclination_noise") = 0.00436f)  //+-0.5degrees
        .def_readwrite("from_station_id", &Leg::from_station_id)
        .def_readwrite("to_station_id", &Leg::to_station_id)
        .def_readwrite("distance", &Leg::distance)
        .def_readwrite("azimuth", &Leg::azimuth)
        .def_readwrite("inclination", &Leg::inclination)
        .def_readwrite("distance_noise", &Leg::distance_noise)
        .def_readwrite("azimuth_noise", &Leg::azimuth_noise)
        .def_readwrite("inclination_noise", &Leg::inclination_noise)
        .def_readwrite("from_station", &Leg::from_station)
        .def_readwrite("to_station", &Leg::to_station)
        .def("__repr__", [](const Leg &leg) {
            return "<Leg from_station_id='" + leg.from_station_id + 
                   "' to_station_id='" + leg.to_station_id + 
                   "' distance=" + std::to_string(leg.distance) + 
                   " azimuth=" + std::to_string(leg.azimuth) + 
                   " inclination=" + std::to_string(leg.inclination) + ">";
        });
}
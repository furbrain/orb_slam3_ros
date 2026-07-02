// bind_atlas.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "Atlas.h"
#include "Map.h"

namespace py = pybind11;
using namespace ORB_SLAM3;

void bind_atlas(py::module &m) {
    py::class_<Atlas>(m, "Atlas")
        .def("get_current_map", &Atlas::GetCurrentMap,
             py::return_value_policy::reference)
        .def("get_all_maps", &Atlas::GetAllMaps,
             py::return_value_policy::reference)
        .def("count_maps", &Atlas::CountMaps)
        .def("is_inertial", &Atlas::isInertial)
        .def("get_last_init_kf_id", &Atlas::GetLastInitKFid)
        .def("get_all_keyframes", [](Atlas &atlas) {
            // aggregates across every map, not just the active one --
            // useful post-merge when you want the full survey, not
            // just whichever map happens to be "current"
            std::vector<KeyFrame*> all;
            for (Map* map : atlas.GetAllMaps()) {
                auto kfs = map->GetAllKeyFrames();
                all.insert(all.end(), kfs.begin(), kfs.end());
            }
            return all;
        }, py::return_value_policy::reference)
        .def("get_all_map_points", [](Atlas &atlas) {
            std::vector<MapPoint*> all;
            for (Map* map : atlas.GetAllMaps()) {
                auto mps = map->GetAllMapPoints();
                all.insert(all.end(), mps.begin(), mps.end());
            }
            return all;
        }, py::return_value_policy::reference);
}

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "KeyFrameDatabase.h"

namespace py = pybind11;
using namespace ORB_SLAM3;

void bind_kfdb(py::module &m) {
    py::class_<KeyFrameDatabase>(m, "KeyFrameDatabase")
        .def(py::init<const ORBVocabulary&>())
        .def("add", &KeyFrameDatabase::add)
        .def("erase", &KeyFrameDatabase::erase)
        .def("clear", &KeyFrameDatabase::clear)
        .def("clear_map", &KeyFrameDatabase::clearMap)
        .def("detect_loop_candidates", &KeyFrameDatabase::DetectLoopCandidates)
        .def("detect_candidates", &KeyFrameDatabase::DetectCandidates)
        .def("detect_best_candidates", &KeyFrameDatabase::DetectBestCandidates)
        .def("detect_n_best_candidates",
          [](ORB_SLAM3::KeyFrameDatabase &db, ORB_SLAM3::KeyFrame *pKF, int nNumCandidates)
            -> std::tuple<std::vector<ORB_SLAM3::KeyFrame*>, std::vector<ORB_SLAM3::KeyFrame*>>
            {
                std::vector<ORB_SLAM3::KeyFrame*> vpLoopCand;
                std::vector<ORB_SLAM3::KeyFrame*> vpMergeCand;
                db.DetectNBestCandidates(pKF, vpLoopCand, vpMergeCand, nNumCandidates);
                return std::make_tuple(vpLoopCand, vpMergeCand);
            },
            py::arg("pKF"),
            py::arg("num_candidates"),
            py::return_value_policy::reference,
            "Returns (loop_candidates, merge_candidates), each sorted by "
            "descending accumulated BoW score."
    );
}
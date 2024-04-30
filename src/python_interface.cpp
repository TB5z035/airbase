#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "airbase.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace py = pybind11;

PYBIND11_MODULE(airbase_py, m) {
  m.attr("__version__") = "1.0.0";
  py::class_<AirBase, std::unique_ptr<AirBase>>(m, "AirBase")
     .def("print_pose", &AirBase::printPose)
     .def("get_base_lock_state", &AirBase::getBaseLockState)
     .def("set_base_lock_state", &AirBase::setBaseLockState)
     .def("save_data_to_json", &AirBase::saveDataToJson)
     .def("load_data_from_json", &AirBase::loadDataFromJson)
     .def("move_to_origin", &AirBase::moveToOrigin)
     .def("build_stcm_map", &AirBase::buildStcmMap)
     .def("load_stcm_map", &AirBase::loadStcmMap)
     .def("teach", &AirBase::teach)
     .def("replay", &AirBase::replay);

};

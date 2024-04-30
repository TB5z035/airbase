#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "airbase.hpp"

namespace py = pybind11;

AirBase create(const py::list& args) {
  int argc = args.size();
  const char* argv[argc];
  for (int i = 0; i < argc; ++i) {
    argv[i] = py::str(args[i]).cast<std::string>().c_str();
  }
  return AirBase(argc, argv);
}

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
      .def("replay", &AirBase::replay)
      .def_readwrite("platform", &AirBase::platform);
  
  py::enum_<rpos::core::ACTION_DIRECTION>(m, "ACTION_DIRECTION")
      .value("FORWARD", rpos::core::ACTION_DIRECTION::FORWARD)
      .value("BACKWARD", rpos::core::ACTION_DIRECTION::BACKWARD)
      .value("TURNRIGHT", rpos::core::ACTION_DIRECTION::TURNRIGHT)
      .value("TURNLEFT", rpos::core::ACTION_DIRECTION::TURNLEFT)
      .value("INVALIDDIRECTION", rpos::core::ACTION_DIRECTION::INVALIDDIRECTION)
      .export_values();

  py::class_<Direction>(m, "Direction")
      .def(py::init<ACTION_DIRECTION>(),
           py::arg("direction") = ACTION_DIRECTION::FORWARD)
      .def("direction",
           (ACTION_DIRECTION(Direction::*)() const) & Direction::direction);

  py::class_<Location>(m, "Location")
      .def(py::init<>())
      .def(py::init<double, double, double>(), py::arg("x") = 0,
           py::arg("y") = 0, py::arg("z") = 0)
      .def(py::init<const Location&>())
      .def("__eq__", &Location::operator==)
      .def("x", (double(Location::*)() const) & Location::x)
      .def("x", (double& (Location::*)()) & Location::x)
      .def("y", (double(Location::*)() const) & Location::y)
      .def("y", (double& (Location::*)()) & Location::y)
      .def("z", (double(Location::*)() const) & Location::z)
      .def("z", (double& (Location::*)()) & Location::z)
      .def("distance_to", &Location::distanceTo);

  py::class_<Rotation>(m, "Rotation")
      .def(py::init<>())
      .def(py::init<double, double, double>(), py::arg("yaw") = 0,
           py::arg("pitch") = 0, py::arg("roll") = 0)
      .def(py::init<const Rotation&>())
      .def(py::init<const Eigen::Matrix3d&>())
      .def("__eq__", &Rotation::operator==)
      .def("__float__", &Rotation::operator Eigen::Matrix3d)
      .def("yaw", (double(Rotation::*)() const) & Rotation::yaw)
      .def("yaw", (double& (Rotation::*)()) & Rotation::yaw)
      .def("pitch", (double(Rotation::*)() const) & Rotation::pitch)
      .def("pitch", (double& (Rotation::*)()) & Rotation::pitch)
      .def("roll", (double(Rotation::*)() const) & Rotation::roll)
      .def("roll", (double& (Rotation::*)()) & Rotation::roll);

  py::class_<Pose>(m, "Pose")
      .def(py::init<>())
      .def(py::init<Location, Rotation>(), py::arg("location") = Location(),
           py::arg("rotation") = Rotation())
      .def(py::init<const Pose&>())
      .def("__eq__", &Pose::operator==);

  py::class_<Point>(m, "Point")
      .def(py::init<>())
      .def(py::init<float, float>(), py::arg("x") = 0.0f, py::arg("y") = 0.0f)
      .def(py::init<const Point&>())
      .def("x", (float(Point::*)() const) & Point::x)
      .def("x", (float& (Point::*)()) & Point::x)
      .def("y", (float(Point::*)() const) & Point::y)
      .def("y", (float& (Point::*)()) & Point::y);

  py::class_<SlamwareCorePlatform, std::unique_ptr<SlamwareCorePlatform>>(
      m, "SlamwareCorePlatform")
      .def("set_system_parameter", &SlamwareCorePlatform::setSystemParameter);
  
  m.def("create", &create);
};

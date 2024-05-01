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

PYBIND11_MODULE(airbase_py, m) {
  m.attr("__version__") = "1.0.0";
  py::class_<AirBase, std::unique_ptr<AirBase>>(m, "AirBase")
      .def(py::init<std::string>(), py::arg("ip"))
      .def("print_pose", &AirBase::print_pose)
      .def("get_base_lock_state", &AirBase::get_baselock_state)
      .def("set_base_lock_state", &AirBase::set_baselock_state)
      .def("save_data_to_json", &AirBase::save_data_to_json)
      .def("load_data_from_json", &AirBase::load_data_from_json)
      .def("move_to_origin", &AirBase::move_to_origin)
      .def("build_stcm_map", &AirBase::build_stcm_map)
      .def("load_stcm_map", &AirBase::load_stcm_map)
      .def("teach", &AirBase::record_trajectory)
      .def("replay", &AirBase::replay_trajectory)
      .def_readwrite("platform", &AirBase::platform);

  py::enum_<rpos::core::ACTION_DIRECTION>(m, "ACTION_DIRECTION")
      .value("FORWARD", rpos::core::ACTION_DIRECTION::FORWARD)
      .value("BACKWARD", rpos::core::ACTION_DIRECTION::BACKWARD)
      .value("TURNRIGHT", rpos::core::ACTION_DIRECTION::TURNRIGHT)
      .value("TURNLEFT", rpos::core::ACTION_DIRECTION::TURNLEFT)
      .value("INVALIDDIRECTION", rpos::core::ACTION_DIRECTION::INVALIDDIRECTION)
      .export_values();

  py::class_<Direction>(m, "Direction")
      .def(py::init<ACTION_DIRECTION>(), py::arg("direction") = ACTION_DIRECTION::FORWARD)
      .def("direction", (ACTION_DIRECTION(Direction::*)() const) & Direction::direction);

  py::class_<Location>(m, "Location")
      .def(py::init<>())
      .def(py::init<double, double, double>(), py::arg("x") = 0, py::arg("y") = 0, py::arg("z") = 0)
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
      .def(py::init<double, double, double>(), py::arg("yaw") = 0, py::arg("pitch") = 0, py::arg("roll") = 0)
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
      .def(py::init<Location, Rotation>(), py::arg("location") = Location(), py::arg("rotation") = Rotation())
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

  py::class_<MoveAction>(m, "MoveAction")
      .def(py::init<boost::shared_ptr<MoveAction::impl_t>>())
      .def("getRemainingPath", &MoveAction::getRemainingPath)
      .def("getRemainingMilestones", &MoveAction::getRemainingMilestones)
      .def("getCurrentSpeed", &MoveAction::getCurrentSpeed)
      .def("getRemainingTime", &MoveAction::getRemainingTime);

  py::class_<SlamwareCorePlatform, std::unique_ptr<SlamwareCorePlatform>>(m, "SlamwareCorePlatform")
      .def("set_system_parameter", &SlamwareCorePlatform::setSystemParameter)
      .def("move_by", py::overload_cast<const Direction&>(&SlamwareCorePlatform::moveBy));
};

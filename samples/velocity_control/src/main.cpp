/**
 * Slamtec Robot Go Action and Get Path Demo
 *
 * Created By Jacky Li @ 2014-8-8
 * Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
 */
#include <rpos/features/location_provider/map.h>
#include <rpos/features/motion_planner/velocity_control_move_action.h>
#include <rpos/robot_platforms/slamware_core_platform.h>

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <regex>

using namespace rpos::robot_platforms;
using namespace rpos::features;
using namespace rpos::features::location_provider;

std::string ipaddress = "";
const char* ipReg = "\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}";

void showHelp(std::string appName) {
  std::cout << "SLAMWARE console demo." << std::endl
            << "Usage: " << appName << " <slamware_address>" << std::endl;
}

bool parseCommandLine(int argc, const char* argv[]) {
  bool opt_show_help = false;

  for (int pos = 1; pos < argc; ++pos) {
    const char* current = argv[pos];
    if (strcmp(current, "-h") == 0) {
      opt_show_help = true;
    } else {
      ipaddress = current;
    }
  }

  std::regex reg(ipReg);
  if (!opt_show_help && !std::regex_match(ipaddress, reg)) {
    opt_show_help = true;
  }

  if (opt_show_help) {
    showHelp("velocity_control");
    return false;
  }

  return true;
}

void velocityActionInitCheck(
    SlamwareCorePlatform& sdp,
    rpos::actions::VelocityControlMoveAction& velocityControllAction,
    const bool& no_limit = false) {
  if (velocityControllAction.isEmpty()) {
    try {
      if (no_limit) {
        velocityControllAction =
            sdp.velocityControl(rpos::features::motion_planner::NotMonitored);
      } else
        velocityControllAction = sdp.velocityControl();
    } catch (const std::exception& e) {
      std::cout << "Can't construct velocity controll action:" << e.what()
                << std::endl;
      return;
    }
  }
}

void velocityControl(
    SlamwareCorePlatform sdp,
    rpos::actions::VelocityControlMoveAction velocityControllAction,
    std::array<double, 3> velocity, int_least64_t duration,
    bool no_limit = false) {
  try {
    velocityControllAction.setVelocity(velocity[0], velocity[1], velocity[2]);
  } catch (const std::exception& e) {
    std::cout << "Can't set velocity:" << e.what() << std::endl;
    velocityControllAction = rpos::actions::VelocityControlMoveAction();
    return;
  }
  boost::this_thread::sleep_for(boost::chrono::milliseconds(duration));
}

void velocitiesControl(
    SlamwareCorePlatform sdp,
    rpos::actions::VelocityControlMoveAction velocityControllAction,
    std::vector<std::array<double, 3>> velocities,
    std::vector<int_least64_t> durations, bool no_limit = false) {
  velocityActionInitCheck(sdp, velocityControllAction, no_limit);
  int period = 5;
  for (std::size_t i = 0; i < velocities.size(); i++) {
    std::array<double, 3> velocity = velocities[i];
    for (int j = 0; j < int(durations[i]/period); j++) {
      auto velocityControllAction = sdp.velocityControl(rpos::features::motion_planner::NotMonitored);
      velocityControllAction.setVelocity(velocity[0], velocity[1], velocity[2]);
      boost::this_thread::sleep_for(boost::chrono::milliseconds(period));
    }
  }
}

std::vector<std::array<double, 3>> generateVelocityCommands(double linear_velocity, double angular_velocity, double speed_factor, int duration_seconds, int control_frequency) {
    // 计算总步数
    int total_steps = duration_seconds * control_frequency;
    
    // 二维数组存储速度命令，每行代表一个周期
    std::vector<std::array<double,3>> velocity_commands;
    std::cout << "test2" << std::endl;
    // 生成速度命令
    for (int i = 0; i < total_steps; i++) {
        // 计算线速度和角速度
        double vx = linear_velocity * speed_factor;
        double vy = 0.0; // 假设平面双轮小车只能直行，无侧向速度
        double omega = angular_velocity * speed_factor;
        // 存储速度命令到二维数组
        velocity_commands.push_back({vx,vy,omega});
        linear_velocity *= 0.999; // 线速度衰减
        angular_velocity *= 0.777; // 角速度衰减
    }
    return velocity_commands;
}

int main(int argc, const char* argv[]) {
  if (!parseCommandLine(argc, argv)) {
    return 1;
  }

  std::vector<std::array<double, 3>> velocities;
  std::vector<int_least64_t> durations = {2000, 2000, 2000, 2000};
  for (size_t i=0;i<1000;i++)
    velocities.push_back({0.0003*i, 0.0, -0.00025*i});

  // velocities = generateVelocityCommands(0.5, 0.5, 1.0, 5, 10);

  SlamwareCorePlatform sdp;

  try {
    sdp = SlamwareCorePlatform::connect(argv[1], 1445);
    std::cout << "SDK Version: " << sdp.getSDKVersion() << std::endl;
    std::cout << "SDP Version: " << sdp.getSDPVersion() << std::endl;
  } catch (ConnectionTimeOutException& e) {
    std::cout << e.what() << std::endl;
    return 1;
  } catch (ConnectionFailException& e) {
    std::cout << e.what() << std::endl;
    return 1;
  }
  std::cout << "Connection Successfully" << std::endl;

  int i = 0;

  rpos::actions::VelocityControlMoveAction velocityControllAction;
  // velocitiesControl(sdp, velocityControllAction, velocities, durations);
  for (auto velocity : velocities) {
    velocityControllAction = sdp.velocityControl(rpos::features::motion_planner::NotMonitored);
    velocityControl(sdp, velocityControllAction, velocity, 5);
  }

  // 信息打印
  while (true) {
    try {
      // 信息打印
      rpos::actions::MoveAction move_action = sdp.getCurrentAction();
      auto motion_strategy = sdp.getCurrentMotionStrategy();
      std::cout << "Motion Strategy: " << motion_strategy << std::endl;

      while (move_action) {
        std::cout << (move_action.isEmpty() ? "Empty" : "Non-Empty")
                  << std::endl;
        std::cout << "Action ID: " << move_action.getActionId() << std::endl;
        std::cout << "Action Name: " << move_action.getActionName()
                  << std::endl;

        rpos::core::Location location = sdp.getLocation();
        std::cout << "Robot Location: " << std::endl;
        std::cout << "x: " << location.x() << ", ";
        std::cout << "y: " << location.y() << std::endl;

        rpos::core::Pose pose = sdp.getPose();
        std::cout << "Robot Pose: " << std::endl;
        std::cout << "x: " << pose.x() << ", ";
        std::cout << "y: " << pose.y() << ", ";
        std::cout << "yaw: " << pose.yaw() << std::endl;

        int battPercentage = sdp.getBatteryPercentage();
        std::cout << "Battery: " << battPercentage << "%" << std::endl;

        boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));
      }
    } catch (ConnectionFailException& e) {
      std::cout << e.what() << std::endl;
      break;
    } catch (RequestTimeOutException& e) {
      std::cout << e.what() << std::endl;
    }
    i++;
  }
  return 0;
}
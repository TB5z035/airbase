/**
 * Slamtec Robot Go Action and Get Path Demo
 *
 * Created By Jacky Li @ 2014-8-8
 * Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
 */
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>

// #include <ncurses.h>
#include <regex>
#include <rpos/features/location_provider/map.h>
#include <rpos/features/motion_planner/feature.h>
#include <rpos/robot_platforms/objects/composite_map.h>
#include <rpos/robot_platforms/objects/composite_map_reader.h>
#include <rpos/robot_platforms/objects/composite_map_writer.h>
#include <rpos/robot_platforms/slamware_core_platform.h>

using json = nlohmann::json;
using namespace rpos::core;
using namespace rpos::actions;
using namespace rpos::robot_platforms;
using namespace rpos::robot_platforms::objects;
using namespace rpos::features;
using namespace rpos::features::artifact_provider;
using namespace rpos::features::location_provider;
using namespace rpos::features::motion_planner;

std::string redmsg = "\033[31m";
std::string resetmsg = "\033[0m";

std::string ipaddress = "";
const char *ipReg = "\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}";

void showHelp(std::string appName) {
  std::cout << "SLAMWARE console demo." << std::endl
            << "Usage: " << appName << " <slamware_address>" << std::endl;
}

bool parseCommandLine(int argc, const char *argv[]) {
  bool opt_show_help = false;

  for (int pos = 1; pos < argc; ++pos) {
    const char *current = argv[pos];
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
    showHelp("mobileAloha");
    return false;
  }

  return true;
}

bool StcmMapWriter(const std::string file_name, SlamwareCorePlatform platform) {
  CompositeMap composite_map = platform.getCompositeMap();
  CompositeMapWriter composite_map_writer;
  std::string error_message;
  bool result =
      composite_map_writer.saveFile(error_message, file_name, composite_map);
  return result;
}

bool StcmMapReader(const std::string file_path, rpos::core::Pose pose,
                   SlamwareCorePlatform platform) {
  CompositeMapReader composite_map_reader;
  std::string error_message;
  boost::shared_ptr<CompositeMap> composite_map(
      composite_map_reader.loadFile(error_message, file_path));
  if (composite_map) {
    platform.setCompositeMap((*composite_map), pose);
    return true;
  }
  return false;
}

int64_t getCurrentTime() {
  auto currentTime = std::chrono::duration_cast<std::chrono::microseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
  return currentTime;
}

void printLocationAndPose(SlamwareCorePlatform platform) {
  Location location = platform.getLocation();
  std::cout << "\nRobot Location: " << std::endl;
  std::cout << "x: " << location.x() << ", ";
  std::cout << "y: " << location.y() << std::endl;
  Pose pose = platform.getPose();
  std::cout << "Robot Pose: " << std::endl;
  std::cout << "x: " << pose.x() << ", ";
  std::cout << "y: " << pose.y() << ", ";
  std::cout << "yaw: " << pose.yaw() << "\n" << std::endl;
}

void saveDataToJson(const std::string &filename,
                    const std::vector<Pose> &poseVec,
                    const std::vector<int64_t> &timestampVec) {
  json jsonData;

  for (size_t i = 0; i < poseVec.size(); ++i) {
    json poseData;
    poseData["x"] = poseVec[i].x();
    poseData["y"] = poseVec[i].y();
    poseData["yaw"] = poseVec[i].yaw();
    poseData["timestamp"] = timestampVec[i];
    jsonData["poses"].push_back(poseData);
  }

  std::ofstream outputFile(filename);
  if (!outputFile.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }

  outputFile << jsonData.dump(2);
  outputFile.close();
  std::cout << "JSON data saved to file: " << filename << std::endl;
}

void loadDataFromJson(const std::string &filename, std::vector<Pose> &poseVec,
                      std::vector<int64_t> &timestampVec) {

  std::ifstream inputFile(filename);
  if (!inputFile.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }

  json jsonData;
  try {
    inputFile >> jsonData;
  } catch (json::parse_error &e) {
    std::cerr << "Failed to parse JSON in file: " << filename << " - "
              << e.what() << std::endl;
    inputFile.close();
    return;
  }

  inputFile.close();

  if (jsonData.find("poses") == jsonData.end() ||
      !jsonData["poses"].is_array()) {
    std::cerr << "JSON data in file: " << filename
              << " does not contain 'poses' array." << std::endl;
    return;
  }

  for (const auto &poseData : jsonData["poses"]) {
    if (!poseData.is_object()) {
      std::cerr << "Invalid data format in JSON file: " << filename
                << std::endl;
      return;
    }

    if (!poseData.contains("x") || !poseData.contains("y") ||
        !poseData.contains("yaw") || !poseData.contains("timestamp")) {
      std::cerr << "Missing required fields in pose data in JSON file: "
                << filename << std::endl;
      return;
    }

    try {
      double x = poseData["x"];
      double y = poseData["y"];
      double yaw = poseData["yaw"];
      int64_t timestamp = poseData["timestamp"];

      Pose pose(Location(x, y, 0), Rotation(yaw, 0, 0));
      poseVec.push_back(pose);
      timestampVec.push_back(timestamp);
    } catch (json::type_error &e) {
      std::cerr << "Error reading pose data from JSON file: " << filename
                << " - " << e.what() << std::endl;
      return;
    }
  }

  std::cout << "JSON data loaded from file: " << filename << std::endl;
}

void moveToByTrack(SlamwareCorePlatform platform, Point targetPoint) {
  rpos::actions::MoveAction moveAction = platform.getCurrentAction();
  Pose currentPose;
  Line pathLine;
  MoveOptions options;
  currentPose = platform.getPose();
  pathLine.startP() = Point(currentPose.x(), currentPose.y());
  pathLine.endP() = Point(targetPoint.x(), targetPoint.y());
  platform.addLine(ArtifactUsageVirtualTrack, pathLine);
  options.flag =
      MoveOptionFlag(MoveOptionFlagKeyPoints | MoveOptionFlagPrecise);
  moveAction =
      platform.moveTo(Location(targetPoint.x(), targetPoint.y(), 0), options);
  moveAction.waitUntilDone();
}

void moveToOrigin(SlamwareCorePlatform platform) {
  platform.clearLines(ArtifactUsageVirtualTrack);
  MoveAction action = platform.getCurrentAction();
  std::cout << "\nMoving to origin" << std::endl;
  moveToByTrack(platform, Point(0, 0));
  action = platform.rotateTo(Rotation(0, 0, 0));
  action.waitUntilDone();
}

void teachBase(SlamwareCorePlatform platform, bool &teaching) {
  std::vector<Pose> poseVec;
  std::vector<int64_t> timestampVec;
  while (teaching) {
    Pose _pose = platform.getPose();
    int64_t currentTime = getCurrentTime();
    poseVec.emplace_back(_pose);
    timestampVec.emplace_back(currentTime);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
  }
  saveDataToJson("poses_data.json", poseVec, timestampVec);
}

float distanceTo(Pose pose1, Pose pose2) {
  return std::sqrt(((pose1.x() - pose2.x()) * (pose1.x() - pose2.x()) +
                    (pose1.y() - pose2.y()) * (pose1.y() - pose2.y())));
}

void replayBase(SlamwareCorePlatform platform) {
  MoveAction action = platform.getCurrentAction();
  auto beginTime = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  std::vector<Pose> poseVec;
  std::vector<int64_t> timestampVec;

  loadDataFromJson("poses_data.json", poseVec, timestampVec);
  rpos::actions::MoveAction moveAction = platform.getCurrentAction();
  bool breakGoal = false;

  std::vector<Line> lines;
  for (size_t i = 0; i < poseVec.size() - 1; ++i) {
    lines.emplace_back(Line(Point(poseVec[i].x(), poseVec[i].y()),
                            Point(poseVec[i + 1].x(), poseVec[i + 1].y())));
  }
  platform.clearLines(ArtifactUsageVirtualTrack);

  platform.addLines(ArtifactUsageVirtualTrack, lines);

  MoveOptions options;

  options.flag =
      MoveOptionFlag(MoveOptionFlagKeyPoints | MoveOptionFlagPrecise);

  for (size_t i = 0; i < poseVec.size();) {
    auto currentTime = getCurrentTime();
    while ((currentTime - beginTime) > (timestampVec[i] - timestampVec[0])) {
      i++;
    }
    if (breakGoal) {
      break;
    }
    if (i >= poseVec.size() - 1) {
      i = poseVec.size() - 1;
      breakGoal = true;
    }

    std::cout << "\n[" << i << "/" << poseVec.size() - 1
              << "] Moving to Pose: " << std::endl;
    std::cout << "x: " << poseVec[i].x() << ", ";
    std::cout << "y: " << poseVec[i].y() << ", ";
    std::cout << "yaw: " << poseVec[i].yaw() << std::endl;

    if (distanceTo(platform.getPose(), poseVec[i]) > 0.1) {
      moveAction =
          platform.moveTo(Location(poseVec[i].x(), poseVec[i].y(), 0), options);
      moveAction.waitUntilDone();
    } else {
      moveAction = platform.rotateTo(Rotation(poseVec[i].yaw(), 0, 0));
      moveAction.waitUntilDone();
    }
    if (i == poseVec.size() - 1) {
      platform.setSystemParameter(SYSPARAM_ROBOT_SPEED, SYSVAL_ROBOT_SPEED_LOW);
      platform.setSystemParameter(SYSPARAM_ROBOT_ANGULAR_SPEED,
                                  SYSVAL_ROBOT_SPEED_LOW);
      moveAction =
          platform.moveTo(Location(poseVec[i].x(), poseVec[i].y(), 0), options);
      moveAction.waitUntilDone();
      if (std::fabs(platform.getPose().yaw() - poseVec[i].yaw()) > 0.02) {
        moveAction = platform.rotateTo(Rotation(poseVec[i].yaw(), 0, 0));
        moveAction.waitUntilDone();
      }
    }
  }
}

int main(int argc, const char *argv[]) {
  if (!parseCommandLine(argc, argv)) {
    return 1;
  }

  // Connect to the robot
  SlamwareCorePlatform sdp;

  try {
    sdp = SlamwareCorePlatform::connect(argv[1], 1445);
    std::cout << "SDK Version: " << sdp.getSDKVersion() << std::endl;
    std::cout << "SDP Version: " << sdp.getSDPVersion() << std::endl;
  } catch (ConnectionTimeOutException &e) {
    std::cout << e.what() << std::endl;
    return 1;
  } catch (ConnectionFailException &e) {
    std::cout << e.what() << std::endl;
    return 1;
  }
  std::cout << "Connection Successfully" << std::endl;
  printLocationAndPose(sdp);
  int battPercentage = sdp.getBatteryPercentage();
  std::cout << "Battery: " << battPercentage << "%" << std::endl;

  sdp.setSystemParameter(SYSPARAM_ROBOT_SPEED, SYSVAL_ROBOT_SPEED_HIGH);
  sdp.setSystemParameter(SYSPARAM_ROBOT_ANGULAR_SPEED, SYSVAL_ROBOT_SPEED_HIGH);

  // Direction forward(ACTION_DIRECTION::FORWARD);
  // rpos::actions::MoveAction moveforward = sdp.moveBy(forward);
  // while(true){
  //   sdp.moveBy(forward);
  // };
  // return 0;

  // set the map
  char ch;
  std::string tmp;
  while (true) {
    std::cout << "Do you want to rebuild map? [y/n]" << std::endl;
    std::cin >> ch;
    if (ch == 'y') {
      std::cout << "Rebuild map\n" << std::endl;
      std::cout << "--------Begin build map-------------\n" << std::endl;
      sdp.setSystemParameter(SYSPARAM_BRAKE_RELEASE, SYSVAL_BRAKE_RELEASE_ON);
      std::cin.ignore();
      std::cout << "\nPlease move the robot base to the origin and press Enter"
                << std::endl;
      getline(std::cin, tmp);
      sdp.setPose(Pose(Location(0, 0), Rotation(0, 0, 0)));
      sdp.clearMap();
      std::cout << "Origin setted. Look RoboStudio for checking map status"
                << std::endl;
      std::cout << "\nPlease move the robot base to build map, and press 's' "
                   "to finish build and save map, press 'o' to reset orgin"
                << std::endl;
      while (true) {
        std::cin >> ch;
        if (ch == 's') {
          std::cout << "Confirm build finish? [y/n]" << std::endl;
          std::cin >> ch;
          if (ch == 'y') {
            StcmMapWriter("newmap.stcm", sdp);
            std::cout << "Build map finish!\n" << std::endl;
            break;
          } else if (ch == 'n') {
            std::cout
                << "Please move the robot base to build map, and press 's' "
                   "to finish build and save map"
                << std::endl;
          } else {
            continue;
          }
        } else if (ch == 'o') {
          std::cout << "Origin setted, please move the robot base to build "
                       "map, and press 's' to finish build and save map, , "
                       "press 'o' to reset orgin"
                    << std::endl;
          std::cout << "Look RoboStudio for checking map status" << std::endl;
          sdp.setPose(Pose(Location(0, 0), Rotation(0, 0, 0)));
          sdp.clearMap();
        }
      }
      std::cout << "Newmap saved" << std::endl;

      printLocationAndPose(sdp);

      std::cout << "--------End build map-------------\n" << std::endl;

      std::cin.ignore();
      sdp.setSystemParameter(SYSPARAM_BRAKE_RELEASE, SYSVAL_BRAKE_RELEASE_OFF);

      printLocationAndPose(sdp);

      break;
    } else if (ch == 'n') {
      std::cout << "Load map\n" << std::endl;
      std::cout << "--------Begin load map-------------" << std::endl;
      StcmMapReader("newmap.stcm", Pose(Location(0, 0), Rotation(0, 0, 0)),
                    sdp);
      std::cout << "\nMap loaded!\n" << std::endl;

      sdp.setSystemParameter(SYSPARAM_BRAKE_RELEASE, SYSVAL_BRAKE_RELEASE_OFF);

      std::cout << "Begin localization\n" << std::endl;
      MoveAction action = sdp.getCurrentAction();
      if (action)
        action.cancel();
      action = sdp.recoverLocalization(rpos::core::RectangleF(-5, -5, 5, 5));
      action.waitUntilDone();

      std::cout << "Localization finished\n" << std::endl;

      printLocationAndPose(sdp);

      moveToOrigin(sdp);

      printLocationAndPose(sdp);

      std::cout << "--------End load map-------------" << std::endl;
      break;
    } else {
      std::cout << "Please enter y or n" << std::endl;
      continue;
    }
  }

  while (true) {
    // teach and replay
    std::cout << "\n---------------------------------" << std::endl;
    std::cout << "Please select option:" << std::endl;
    std::cout << "1. robot move to the origin of the map" << std::endl;
    std::cout << "2. teach the base action" << std::endl;
    std::cout << "3. replay the base action" << std::endl;
    std::cout << "4. unlock the base" << std::endl;
    std::cout << "5. lock the base" << std::endl;
    std::cin >> ch;
    switch (ch) {
    case '1': {
      moveToOrigin(sdp);
    } break;
    case '2': {
      std::cout << "Do you want reteach? [y/n]" << std::endl;
      while (true) {
        std::cin >> ch;
        if (ch == 'y') {
          bool teaching = true;

          sdp.setSystemParameter(SYSPARAM_BRAKE_RELEASE,
                                 SYSVAL_BRAKE_RELEASE_ON);
          std::cout << "------------ teach start ------------\n " << std::endl;
          std::thread teach(teachBase, sdp, std::ref(teaching));

          std::cout << "Teaching ... , input 's' to stop teach:" << std::endl;
          while (true) {
            std::cin >> ch;
            if (ch == 's')
              break;
          }
          teaching = false;
          teach.join();
          sdp.setSystemParameter(SYSPARAM_BRAKE_RELEASE,
                                 SYSVAL_BRAKE_RELEASE_OFF);
          std::cout << "------------ teach finish ------------\n " << std::endl;

          break;
        } else {
          break;
        }
      }
    } break;
    case '3': {
      sdp.setSystemParameter(SYSPARAM_BRAKE_RELEASE, SYSVAL_BRAKE_RELEASE_OFF);
      std::cout << "------------ replay start ------------\n " << std::endl;
      replayBase(sdp);
      std::cout << "------------ replay finish ------------\n " << std::endl;
    } break;
    case '4': {
      sdp.setSystemParameter(SYSPARAM_BRAKE_RELEASE, SYSVAL_BRAKE_RELEASE_ON);
    } break;
    case '5': {
      sdp.setSystemParameter(SYSPARAM_BRAKE_RELEASE, SYSVAL_BRAKE_RELEASE_OFF);
    } break;
    default:
      continue;
      break;
    }
  }
  return 0;
}
// std::cout << "Do you want to save map? [y/n]" << std::endl;
// std::cin >> ch;
// initscr();  // 初始化 ncurses
// timeout(0); // 设置非阻塞输入
// int ch;

// Location origin_location = sdp.getLocation();
// Location pointsToGo;
// rpos::core::Pose origin(Location(0, 0), Rotation(0, 0, 0));
// Direction forward(ACTION_DIRECTION::FORWARD);
// rpos::actions::MoveAction moveforward = sdp.moveBy(forward);
// Direction backward(ACTION_DIRECTION::BACKWARD);
// rpos::actions::MoveAction movebackward = sdp.moveBy(backward);
// Rotation rotation(M_PI / 180 * 10, 0, 0);
// Rotation _rotation(-M_PI / 180 * 10, 0, 0);
// sdp.setSystemParameter(SYSPARAM_ROBOT_SPEED, SYSVAL_ROBOT_SPEED_HIGH);
// while (true) {
//   ch = getch();
//   if (ch != ERR) {
//     if (ch == '\n' || ch == '\r')
//       break;
//     switch (ch) {
//     case 'w':
//       sdp.moveBy(forward);
//       break;
//     case 's':
//       sdp.moveBy(backward);
//       break;
//     case 'a':
//       sdp.rotate(rotation);
//       break;
//     case 'd':
//       sdp.rotate(_rotation);
//       break;
//     case 'o':
//       sdp.setPose(origin);
//       break;
//     case 'c':
//       sdp.clearMap();
//       break;
//     default:
//       break;
//     }
//   }
//   // pointsToGo = Location(origin_location.x() + distance_x,
//   //                               origin_location.y() + distance_y);

//   // try {
//   //   rpos::actions::MoveAction moveAction = sdp.getCurrentAction();

//   //   if (moveAction) {
//   // boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
//   //     if (moveAction.getStatus() == rpos::core::ActionStatusFinished)
//   {
//   //       moveAction = sdp.moveTo(pointsToGo, false);
//   //     }
//   //   } else {
//   // boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
//   //     moveAction = sdp.moveTo(pointsToGo, false);
//   //   }

//   // } catch (ConnectionFailException &e) {
//   //   std::cout << e.what() << std::endl;
//   //   break;
//   // } catch (RequestTimeOutException &e) {
//   //   std::cout << e.what() << std::endl;
//   // }
//   flushinp();
//   clear();
// }
// endwin(); // 结束 ncurses
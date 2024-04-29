/**
 * Slamtec Robot Go Action and Get Path Demo
 *
 * Created By Jacky Li @ 2014-8-8
 * Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
 */
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>

#include <ncurses.h>
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

void showHelp(std::string appName);
bool parseCommandLine(int argc, const char *argv[]);
int64_t getCurrentTime();

class AirBase {
private:
  //
  bool running = true;
  bool release_brake = false;
  int save_delay = 100;
  float angle_factor = 2.7;
  float angle_threshold = 10 * save_delay / 1000;
  float distance_threshold = 0.1 * save_delay / 1000;
  enum Behavior { stoping, moving, rotating, backwarding };
  //
  std::vector<Pose> &poseVec;
  std::vector<int64_t> &timestampVec;
  std::vector<int> &behaviorVec;
  int vecSize;
  //
  Pose currentPose;
  Pose lastPose;

protected:
  bool StcmMapWriter(const std::string file_name);
  bool StcmMapReader(const std::string file_path);

public:
  SlamwareCorePlatform platform;
  //
  AirBase();
  ~AirBase();
  //
  void printPose();
  void saveDataToJson(const std::string &filename);
  void loadDataFromJson(const std::string &filename);
  //
  float distanceBetween(Pose pose1, Pose pose2);
  float angleBetween(Pose pose1, Pose pose2);
  bool ifBackward(Pose last_pose, Pose current_pose);
  int getBehavior(Pose last_pose, Pose current_pose);
  //
  void moveToByTrack(Point targetPoint);
  void moveToOrigin();
  //
  void teach(std::string data_savepath);
  void replay(std::string data_path);
  //
  void buildMap(std::string map_savepath);
  void loadMap(std::string map_path);
};
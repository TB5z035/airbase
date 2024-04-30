#pragma once

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

extern const char *ipReg;
extern std::string ipaddress;

void showHelp(std::string appName);
bool parseCommandLine(int argc, const char *argv[]);
int64_t getCurrentTime();

class AirBase {
private:
  //
  bool running = true;
  bool base_lock = true;
  int save_delay = 100;
  float angle_factor = 2.7;
  float angle_threshold = 10 * save_delay / 1000;
  float distance_threshold = 0.1 * save_delay / 1000;
  enum Behavior { stoping, moving, rotating, backwarding };
  //
  std::vector<Pose> poseVec;
  std::vector<int64_t> timestampVec;
  std::vector<int> behaviorVec;
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
  AirBase(int argc, const char *argv[]);
  ~AirBase();
  //
  void printPose();
  bool getBaseLockState();
  void setBaseLockState(bool lockState);
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
  void buildStcmMap(std::string map_savepath);
  void loadStcmMap(std::string map_path);
  //
  void teach(std::string data_savepath);
  void replay(std::string data_path);
};
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

class AirBase {
private:
  float angle_factor = 3;
  float angle_threshold;
  float distance_threshold;
  enum Behavior {
    stopping,
    forwarding,
    leftturning,
    rightturning,
    backwarding
  };
  //
  std::vector<Pose> poseVec;
  std::vector<int64_t> timestampVec;
  std::vector<int> behaviorVec;
  int vecSize;
  //
  Pose currentPose = Pose(Location(0, 0, 0), Rotation(0, 0, 0));
  Pose lastPose = Pose(Location(0, 0, 0), Rotation(0, 0, 0));
  bool recording = false;
  bool baseLock_ = true;
  int currentBehavior;
  time_t lastTimestamp = 0;
  time_t currentTimestamp = 0;

protected:
  bool StcmMapWriter(const std::string file_name);
  bool StcmMapReader(const std::string file_path);

public:
  SlamwareCorePlatform platform;
  //
  AirBase(std::string ip);
  ~AirBase();
  //
  void printPose();
  bool getBaseLockState();
  void setBaseLockState(bool lockState);
  void saveDataToJson(const std::string &filename);
  void loadDataFromJson(const std::string &filename);
  //
  double distanceBetween(Pose pose1, Pose pose2);
  double angleBetween(Pose pose1, Pose pose2);
  bool ifBackward(Pose last_pose, Pose current_pose);
  int getBehavior(Pose last_pose, Pose current_pose);
  //
  void moveToByTrack(Point targetPoint);
  void moveToOrigin();
  //
  void buildStcmMap(std::string map_savepath);
  void loadStcmMap(std::string map_path);
  //
  void record(std::string task_name, int max_time_steps, int frequency,
              int start_episode);
  void replay(std::string task_name);
};
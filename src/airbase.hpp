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
  float angle_factor_ = 3;
  float angle_threshold_;
  float distance_threshold_;
  enum BEHAVIOR {
    stopping,
    forwarding,
    leftturning,
    rightturning,
    backwarding
  };
  //
  std::vector<Pose> poseVec_;
  std::vector<int64_t> timestampVec_;
  std::vector<int> behaviorVec_;
  int vecSize_;
  //
  time_t base_time_;
  //
  std::vector<Pose> poseTogo_;
  std::vector<int64_t> timestampTogo_;
  std::vector<int> behaviorTogo_;
  size_t traj_lenth_;
  Pose lastPose_ = Pose(Location(0, 0, 0), Rotation(0, 0, 0));
  bool recording_ =  false;
  bool baseLock_ = true;
  int currentBehavior_;
  time_t lastTimestamp_ = 0;
  time_t currentTimestamp_ = 0;

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
  // sub functions
  void get_key_traj(const std::vector<Pose> &pose_in,
                    const std::vector<int64_t> &timestamp_in,
                    const std::vector<int> &behavior_in,
                    std::vector<Pose> &pose_out,
                    std::vector<int64_t> &timestamp_out,
                    std::vector<int> &behavior_out);
  void init_lines(std::vector<Pose> poseTogo);
  void init_traj(const std::vector<Pose> &poses,
                 const std::vector<int64_t> &timestamp,
                 const std::vector<int> &behavior,
                 const bool &use_key_points = false);
  void init_traj(const std::vector<std::vector<double>> &poses,
                 const std::vector<int64_t> &timestamp,
                 const std::vector<int> &behavior, const bool &use_key_points);
  void step(std::vector<double> action, int behavior, bool wait);
  void next_step(const bool &wait = false);
  //
  void record(std::string task_name, int max_time_steps, int frequency,
              int start_episode);
  void replay(std::string task_name);
};
#pragma once

/**
 * AirBase class
 *
 * Created By Stars & OpenGHz @ 2024/05/01
 * Copyright (c) 2024 WuXi Discover Co., Ltd.
 */
<<<<<<< HEAD
=======
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>

>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7
#include <ncurses.h>
#include <rpos/features/location_provider/map.h>
#include <rpos/features/motion_planner/feature.h>
#include <rpos/robot_platforms/objects/composite_map.h>
#include <rpos/robot_platforms/objects/composite_map_reader.h>
#include <rpos/robot_platforms/objects/composite_map_writer.h>
#include <rpos/robot_platforms/slamware_core_platform.h>

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <regex>
#include <thread>
#include <mutex>

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
<<<<<<< HEAD
 private:
  float angle_factor = 3;
  float angle_threshold;
  float distance_threshold;
  enum Behavior { STOP, FORWARD, TURNLEFT, TURNRIGHT, BACKWARD };
=======
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
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7
  //
  std::vector<Pose> pose_vec_;
  std::vector<int64_t> timestamp_vec_;
  std::vector<int> behavior_vec_;
  int vec_size_;
  int episode_ = 0;
  //
<<<<<<< HEAD
  bool base_lock_ = true;
=======
  Pose currentPose = Pose(Location(0, 0, 0), Rotation(0, 0, 0));
  Pose lastPose = Pose(Location(0, 0, 0), Rotation(0, 0, 0));
  bool recording = false;
  bool baseLock_ = true;
  int currentBehavior;
  time_t lastTimestamp = 0;
  time_t currentTimestamp = 0;
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7

 protected:
  bool _stcm_map_writer(const std::string file_name);
  bool _stcm_map_reader(const std::string file_path);

 public:
  SlamwareCorePlatform platform;
  //
  AirBase(std::string ip);
  ~AirBase();
  //
  void print_pose();
  bool get_baselock_state();
  int get_current_episode();
  void set_baselock_state(bool lockState);
  void save_data_to_json(const std::string &filename);
  void load_data_from_json(const std::string &filename);
  //
<<<<<<< HEAD
  double distance_between(Pose pose1, Pose pose2);
  double angle_between(Pose pose1, Pose pose2);
  bool if_backward(Pose last_pose, Pose current_pose);
  int get_current_behavior(Pose last_pose, Pose current_pose);
=======
  double distanceBetween(Pose pose1, Pose pose2);
  double angleBetween(Pose pose1, Pose pose2);
  bool ifBackward(Pose last_pose, Pose current_pose);
  int getBehavior(Pose last_pose, Pose current_pose);
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7
  //
  void moveTo_by_line(Point targetPoint);
  void move_to_origin();
  //
  void build_stcm_map(std::string map_savepath);
  void load_stcm_map(std::string map_path);
  //
<<<<<<< HEAD
  void record_trajectory(std::string task_name, int max_time_steps, int frequency, int start_episode);
  void replay_trajectory(std::string task_name);
=======
  void record(std::string task_name, int max_time_steps, int frequency,
              int start_episode);
  void replay(std::string task_name);
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7
};
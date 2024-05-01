#include "airbase.hpp"

inline time_t get_current_time() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

AirBase::AirBase(std::string ip) {
  try {
    platform = SlamwareCorePlatform::connect(ip, 1445);
    std::cout << "SDK Version: " << platform.getSDKVersion() << std::endl;
    std::cout << "SDP Version: " << platform.getSDPVersion() << std::endl;
  } catch (ConnectionTimeOutException &e) {
    std::cerr << e.what() << std::endl;
    return;
  } catch (ConnectionFailException &e) {
    std::cerr << e.what() << std::endl;
    return;
  }
  std::cout << "Connection Successfully!" << std::endl;
  int battPercentage = platform.getBatteryPercentage();
  std::cout << "Battery: " << battPercentage << "%" << std::endl;
  std::cout << "\n ------------- robot base inited ------------- \n" << std::endl;
  set_baselock_state(true);
}

AirBase::~AirBase() { set_baselock_state(true); }

bool AirBase::_stcm_map_writer(const std::string file_name) {
  CompositeMap composite_map = platform.getCompositeMap();
  CompositeMapWriter composite_map_writer;
  std::string error_message;
  bool result = composite_map_writer.saveFile(error_message, file_name, composite_map);
  return result;
}

bool AirBase::_stcm_map_reader(const std::string file_path) {
  CompositeMapReader composite_map_reader;
  std::string error_message;
  boost::shared_ptr<CompositeMap> composite_map(composite_map_reader.loadFile(error_message, file_path));
  if (composite_map) {
    platform.setCompositeMap((*composite_map), Pose(Location(0, 0), Rotation(0, 0, 0)));
    return true;
  }
  return false;
}

void AirBase::print_pose() {
  Pose pose = platform.getPose();
  std::cout << "Robot Pose: " << std::endl;
  std::cout << "x: " << pose.x() << ", ";
  std::cout << "y: " << pose.y() << ", ";
  std::cout << "yaw: " << pose.yaw() << std::endl;
}

bool AirBase::get_baselock_state() { return base_lock_; }
void AirBase::set_baselock_state(bool lockState) {
  base_lock_ = lockState;
  if (base_lock_) {
    platform.setSystemParameter(SYSPARAM_BRAKE_RELEASE, SYSVAL_BRAKE_RELEASE_OFF);
  } else {
    platform.setSystemParameter(SYSPARAM_BRAKE_RELEASE, SYSVAL_BRAKE_RELEASE_ON);
  }
};

int AirBase::get_current_episode() { return episode_; }

void AirBase::save_data_to_json(const std::string &filename) {
  json jsonData;

  // data clean
  int behaviorCnt = 0;  // for fixing noise behaviors
  int last_valid_behavior = 0;
  for (int i = 0; i < pose_vec_.size() - 1; i++) {
    if (behavior_vec_[i] == behavior_vec_[i + 1]) {
      behaviorCnt++;
      continue;
    }
    if (behaviorCnt <= 3) {
      for (int j = i; j >= i - 3; j--) {
        behavior_vec_[j] = behavior_vec_[i - 3];
      }
    } else {
      last_valid_behavior = behavior_vec_[i];
    }
    behaviorCnt = 0;
  }

  for (size_t i = 0; i < pose_vec_.size(); ++i) {
    jsonData["x"].push_back(pose_vec_[i].x());
    jsonData["y"].push_back(pose_vec_[i].y());
    jsonData["yaw"].push_back(pose_vec_[i].yaw());
    jsonData["timestamp"].push_back(timestamp_vec_[i]);
    jsonData["behavior"].push_back(behavior_vec_[i]);
  }
  vec_size_ = pose_vec_.size();
  jsonData["vec_size_"] = vec_size_;
  std::ofstream outputFile(filename);

  outputFile << jsonData.dump(2);
  outputFile.close();

  pose_vec_.clear();
  timestamp_vec_.clear();
  behavior_vec_.clear();
}

void AirBase::load_data_from_json(const std::string &filename) {
  std::ifstream inputFile(filename);

  json jsonData;
  try {
    inputFile >> jsonData;
  } catch (json::parse_error &e) {
    inputFile.close();
    return;
  }

  inputFile.close();

  vec_size_ = jsonData["vec_size_"];

  for (int i = 0; i < vec_size_; i++) {
    try {
      double x = jsonData["x"][i];
      double y = jsonData["y"][i];
      double yaw = jsonData["yaw"][i];
      int64_t timestamp = jsonData["timestamp"][i];
      int behavior = jsonData["behavior"][i];
      pose_vec_.emplace_back(Pose(Location(x, y), Rotation(yaw, 0, 0)));
      timestamp_vec_.emplace_back(timestamp);
      behavior_vec_.emplace_back(behavior);
    } catch (json::type_error &e) {
      return;
    }
  }
}

double AirBase::distance_between(Pose pose1, Pose pose2) {
  return std::sqrt((std::pow(pose1.x() - pose2.x(), 2) + std::pow(pose1.y() - pose2.y(), 2)));
}

double AirBase::angle_between(Pose pose1, Pose pose2) { return std::fabs(pose1.yaw() - pose2.yaw()) / M_PI * 180; }

bool AirBase::if_backward(Pose last_pose, Pose current_pose) {
  double dx = current_pose.x() - last_pose.x();
  double dy = current_pose.y() - last_pose.y();
  double distance = sqrt(dx * dx + dy * dy);
  double angle = atan2(dy, dx);
  double dyaw = std::fabs(angle - last_pose.yaw());
  return ((dyaw > M_PI * 0.6) && (dyaw < M_PI * 1.4)) && (distance > distance_threshold) ? true : false;
}

int AirBase::get_current_behavior(Pose last_pose, Pose current_pose) {
  int current_behavior;
  auto distance = distance_between(last_pose, current_pose);
  auto angle = angle_between(last_pose, current_pose);
  if ((angle > angle_threshold) && (distance < angle_factor * distance_threshold)) {
    if (last_pose.yaw() < current_pose.yaw()) {
      if (current_pose.yaw() == 0) {
        current_behavior = TURNRIGHT;
      } else {
        current_behavior = TURNLEFT;
      }
    } else {
      if (current_pose.yaw() == 0) {
        current_behavior = TURNLEFT;
      } else {
        current_behavior = TURNRIGHT;
      }
    }
  } else {
    if (distance < distance_threshold) {
      current_behavior = STOP;
    } else {
      if (if_backward(last_pose, current_pose)) {
        current_behavior = BACKWARD;
      } else {
        current_behavior = FORWARD;
      }
    }
  }
  switch (current_behavior) {
    case FORWARD:
      std::cout << "car is FORWARD   | ^^^^^^^^^^^^^^^^^^^^^^^^^^^ | distance: " << distance << std::endl;
      break;
    case BACKWARD:
      std::cout << "car is BACKWARD  | vvvvvvvvvvvvvvvvvvvvvvvvvvv | distance: " << distance << std::endl;
      break;
    case TURNLEFT:
      std::cout << "car is TURNLEFT  | <<<<<<<<<<<<<<<<<<<<<<<<<<< |    angle: " << angle << std::endl;
      break;
    case TURNRIGHT:
      std::cout << "car is TURNRIGHT | >>>>>>>>>>>>>>>>>>>>>>>>>>> |    angle: " << angle << std::endl;
      break;
    case STOP:
      std::cout << "car is STOP      | ........................... | distance: " << distance << std::endl;
      break;
  }
  return current_behavior;
}

void AirBase::moveTo_by_line(Point targetPoint) {
  rpos::actions::MoveAction moveAction = platform.getCurrentAction();
  Pose currentPose;
  Line pathLine;
  MoveOptions options;
  currentPose = platform.getPose();
  pathLine.startP() = Point(currentPose.x(), currentPose.y());
  pathLine.endP() = Point(targetPoint.x(), targetPoint.y());
  platform.addLine(ArtifactUsageVirtualTrack, pathLine);
  options.flag = MoveOptionFlag(MoveOptionFlagKeyPoints | MoveOptionFlagPrecise);
  moveAction = platform.moveTo(Location(targetPoint.x(), targetPoint.y(), 0), options);
  moveAction.waitUntilDone();
  platform.clearLines(ArtifactUsageVirtualTrack);
}

void AirBase::move_to_origin() {
  set_baselock_state(true);
  set_baselock_state(true);
  platform.clearLines(ArtifactUsageVirtualTrack);
  MoveAction action = platform.getCurrentAction();
  std::cout << "\nMoving to origin" << std::endl;
  moveTo_by_line(Point(0, 0));
  action = platform.rotateTo(Rotation(0, 0, 0));
  action.waitUntilDone();
  std::cout << std::endl;
}

void AirBase::build_stcm_map(std::string map_savepath) {
  char ch;
  std::string tmp;
  std::cout << "Rebuild map\n" << std::endl;
  std::cout << "--------Begin build map-------------\n" << std::endl;
  set_baselock_state(false);
  std::cout << "\nPlease move the robot base to the origin and press Enter" << std::endl;
  getline(std::cin, tmp);
  platform.setPose(Pose(Location(0, 0), Rotation(0, 0, 0)));
  platform.clearMap();
  std::cout << "Origin setted. Look RoboStudio for checking map status" << std::endl;
  std::cout << "\nPlease move the robot base to build map, and press 's' "
               "to finish build and save map, press 'o' to reset orgin"
            << std::endl;
  while (true) {
    std::cin >> ch;
    if (ch == 's') {
      _stcm_map_writer(map_savepath);
      std::cout << "Build map finish!\n" << std::endl;
      break;
    } else if (ch == 'o') {
      std::cout << "Origin setted, please move the robot base to build "
                   "map, and press 's' to finish build and save map, , "
                   "press 'o' to reset orgin"
                << std::endl;
      std::cout << "Look RoboStudio for checking map status" << std::endl;
      platform.setPose(Pose(Location(0, 0), Rotation(0, 0, 0)));
      platform.clearMap();
    }
  }
  std::cout << "Newmap saved\n" << std::endl;

  print_pose();

  std::cout << "\n--------End build map-------------\n" << std::endl;
  set_baselock_state(true);

  std::cout << "input 'o' move to origin, else to skip" << std::endl;
  std::cin >> ch;
  std::cin.ignore();
  if (ch == 'o') move_to_origin();

  print_pose();
}

void AirBase::load_stcm_map(std::string map_path) {
  set_baselock_state(true);
  set_baselock_state(true);
  std::cout << "Load map\n" << std::endl;
  std::cout << "--------Begin load map-------------" << std::endl;
  _stcm_map_reader(map_path);
  std::cout << "\nMap loaded!\n" << std::endl;

  std::cout << "Begin localization ...\n" << std::endl;
  MoveAction action = platform.getCurrentAction();
  if (action) action.cancel();
  action = platform.recoverLocalization(rpos::core::RectangleF(0, 0, 10, 10));
  action.waitUntilDone();

  std::cout << "Localization finished\n" << std::endl;

  print_pose();

  move_to_origin();

  print_pose();

  std::cout << "--------End load map-------------" << std::endl;
}

void AirBase::record_trajectory(std::string task_name, int max_time_steps, int frequency, int start_episode) {
  episode_ = (episode_ == 0 && episode_ != start_episode) ? start_episode : episode_;
  angle_threshold = 10.0 / frequency;
  distance_threshold = 0.05 / frequency;
  set_baselock_state(false);
  std::cout << "get in" << std::endl;
  int current_time_step = 0;
  int ch;
  bool teaching = true;
  std::thread teach([&]() {
    Pose last_pose = platform.getPose();
    time_t last_timestamp = get_current_time();
    while (teaching && (current_time_step < max_time_steps)) {
      std::cout << "[" << current_time_step + 1 << "/" << max_time_steps << "] ";
      Pose current_pose = platform.getPose();
      time_t current_time = get_current_time();
      pose_vec_.emplace_back(current_pose);
      timestamp_vec_.emplace_back(current_time);
      auto behavior = get_current_behavior(last_pose, current_pose);
      behavior_vec_.emplace_back(behavior);
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / frequency));
      // if (get_current_time() - last_timestamp < 1000.0 / frequency) {
      //   boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
      // }
      // last_timestamp = current_time;
      last_pose = current_pose;
      current_time_step++;
      if (current_time_step >= max_time_steps) {
        std::cout << "\033[36m Data Collected!\n\033[0m" << std::endl;
        std::cout << "\nPress Enter to continue ..." << std::endl;
      }
    }
    teaching = false;
  });

  std::cin >> ch;

  if (current_time_step >= max_time_steps) {
    std::cout << "\nInput 's' to save the collected data, any other key to drop" << std::endl;
    std::cin >> ch;
    if (ch == 's') {
      std::string dataname = "base_data/raw/" + task_name + "/" + std::to_string(episode_++) + ".json";
      save_data_to_json(dataname);
    } else {
      pose_vec_.clear();
      timestamp_vec_.clear();
      behavior_vec_.clear();
      vec_size_ = 0;
      current_time_step = 0;
      std::cout << "\033[31m Data Droped!\n\033[0m" << std::endl;
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1500));
    }
  } else {
    pose_vec_.clear();
    timestamp_vec_.clear();
    behavior_vec_.clear();
    vec_size_ = 0;
    current_time_step = 0;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
    std::cout << "\033[31m \nData collect cancled! \033[0m" << std::endl;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(1500));
  }

  set_baselock_state(true);
  teach.join();
  return;
}

void AirBase::replay_trajectory(std::string data_path) {
  set_baselock_state(true);
  pose_vec_.clear();
  timestamp_vec_.clear();
  behavior_vec_.clear();
  load_data_from_json(data_path);
  MoveAction action = platform.getCurrentAction();

  // select key points
  std::vector<Pose> poseTogo;
  std::vector<int64_t> timestampTogo;
  std::vector<int> behaviorTogo;
  for (int i = 0; i < vec_size_ - 1; i++) {
    if (behavior_vec_[i] == behavior_vec_[i + 1]) {
      continue;
    }
    poseTogo.emplace_back(pose_vec_[i]);
    timestampTogo.emplace_back(timestamp_vec_[i]);
    behaviorTogo.emplace_back(behavior_vec_[i]);
  }

  std::vector<Line> lines;

  // // fix path
  //
  //
  //
  //

  // build path
  for (int i = 0; i < vec_size_ - 1; ++i) {
    lines.emplace_back(
        Line(Point(pose_vec_[i].x(), pose_vec_[i].y()), Point(pose_vec_[i + 1].x(), pose_vec_[i + 1].y())));
  }
  platform.clearLines(ArtifactUsageVirtualTrack);
  platform.addLines(ArtifactUsageVirtualTrack, lines);

  // excute replay_trajectory
  auto beginTime = get_current_time();
  for (size_t i = 0; i < poseTogo.size(); i++) {
    rpos::actions::MoveAction moveAction = platform.getCurrentAction();
    MoveOptions options;
    options.mode = NavigationMode(NavigationModeStrictVirtualTrack);
    options.flag = MoveOptionFlag(MoveOptionFlagKeyPoints | MoveOptionFlagPrecise);

    switch (behaviorTogo[i]) {
      case FORWARD:
      case BACKWARD:
      case STOP: {
        std::cout << "\n[" << i << "/" << poseTogo.size() - 1 << "] Moving to: ";
        std::cout << "x: " << poseTogo[i].x() << ", ";
        std::cout << "y: " << poseTogo[i].y() << std::endl;
        moveAction = platform.moveTo(Location(poseTogo[i].x(), poseTogo[i].y(), 0), options);
        moveAction.waitUntilDone();
      } break;
      case TURNLEFT:
      case TURNRIGHT: {
        std::cout << "\n[" << i << "/" << poseTogo.size() - 1 << "] Rotating to: ";
        std::cout << "yaw: " << poseTogo[i].yaw() << std::endl;
        moveAction = platform.rotateTo(Rotation(poseTogo[i].yaw(), 0, 0));
        moveAction.waitUntilDone();
      } break;
      // case BACKWARD: {
      //   // options.mode =
      //   //     NavigationMode(NavigationModeStrictVirtualTrackReverseWalk);
      //   moveAction = platform.moveTo(
      //       Location(poseTogo[i].x(), poseTogo[i].y(), 0), options);
      //   moveAction.waitUntilDone();
      // }
      default:
        break;
    }

    while ((get_current_time() - beginTime) < (timestampTogo[i] - timestamp_vec_[0])) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
    }
  }
}

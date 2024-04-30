#include "airbase.hpp"

inline time_t getCurrentTime() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
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
  std::cout << "\n ------------- robot base inited ------------- \n"
            << std::endl;
  platform.setSystemParameter(SYSPARAM_BRAKE_RELEASE, SYSVAL_BRAKE_RELEASE_OFF);
}

AirBase::~AirBase() {
  platform.setSystemParameter(SYSPARAM_BRAKE_RELEASE, SYSVAL_BRAKE_RELEASE_OFF);
}

bool AirBase::StcmMapWriter(const std::string file_name) {
  CompositeMap composite_map = platform.getCompositeMap();
  CompositeMapWriter composite_map_writer;
  std::string error_message;
  bool result =
      composite_map_writer.saveFile(error_message, file_name, composite_map);
  return result;
}

bool AirBase::StcmMapReader(const std::string file_path) {
  CompositeMapReader composite_map_reader;
  std::string error_message;
  boost::shared_ptr<CompositeMap> composite_map(
      composite_map_reader.loadFile(error_message, file_path));
  if (composite_map) {
    platform.setCompositeMap((*composite_map),
                             Pose(Location(0, 0), Rotation(0, 0, 0)));
    return true;
  }
  return false;
}

void AirBase::printPose() {
  Pose pose = platform.getPose();
  std::cout << "Robot Pose: " << std::endl;
  std::cout << "x: " << pose.x() << ", ";
  std::cout << "y: " << pose.y() << ", ";
  std::cout << "yaw: " << pose.yaw() << "\n" << std::endl;
}

bool AirBase::getBaseLockState() { return baseLock_; }
void AirBase::setBaseLockState(bool lockState) {
  baseLock_ = lockState;
  if (baseLock_) {
    platform.setSystemParameter(SYSPARAM_BRAKE_RELEASE,
                                SYSVAL_BRAKE_RELEASE_OFF);
  } else {
    platform.setSystemParameter(SYSPARAM_BRAKE_RELEASE,
                                SYSVAL_BRAKE_RELEASE_ON);
  }
};

void AirBase::saveDataToJson(const std::string &filename) {
  json jsonData;

  for (size_t i = 0; i < poseVec.size(); ++i) {
    jsonData["x"].push_back(poseVec[i].x());
    jsonData["y"].push_back(poseVec[i].y());
    jsonData["yaw"].push_back(poseVec[i].yaw());
    jsonData["timestamp"].push_back(timestampVec[i]);
    jsonData["behavior"].push_back(behaviorVec[i]);
  }
  vecSize = poseVec.size();
  jsonData["vecSize"] = vecSize;
  std::ofstream outputFile(filename);

  outputFile << jsonData.dump(2);
  outputFile.close();
}

void AirBase::loadDataFromJson(const std::string &filename) {
  std::ifstream inputFile(filename);

  json jsonData;
  try {
    inputFile >> jsonData;
  } catch (json::parse_error &e) {
    inputFile.close();
    return;
  }

  inputFile.close();

  vecSize = jsonData["vecSize"];

  for (int i = 0; i < vecSize; i++) {
    try {
      double x = jsonData["x"][i];
      double y = jsonData["y"][i];
      double yaw = jsonData["yaw"][i];
      int64_t timestamp = jsonData["timestamp"][i];
      int behavior = jsonData["behavior"][i];
      poseVec.emplace_back(Pose(Location(x, y), Rotation(yaw, 0, 0)));
      timestampVec.emplace_back(timestamp);
      behaviorVec.emplace_back(behavior);
    } catch (json::type_error &e) {
      return;
    }
  }
}

double AirBase::distanceBetween(Pose pose1, Pose pose2) {
  return std::sqrt((std::pow(pose1.x() - pose2.x(), 2) +
                    std::pow(pose1.y() - pose2.y(), 2)));
}

double AirBase::angleBetween(Pose pose1, Pose pose2) {
  return std::fabs(pose1.yaw() - pose2.yaw()) / M_PI * 180;
}

bool AirBase::ifBackward(Pose last_pose, Pose current_pose) {
  double dx = current_pose.x() - last_pose.x();
  double dy = current_pose.y() - last_pose.y();
  double distance = sqrt(dx * dx + dy * dy);
  double angle = atan2(dy, dx);
  double dyaw = std::fabs(angle - last_pose.yaw());
  return ((dyaw > M_PI * 0.6) && (dyaw < M_PI * 1.4)) &&
                 (distance > distance_threshold)
             ? true
             : false;
}

int AirBase::getBehavior(Pose last_pose, Pose current_pose) {
  auto distance = distanceBetween(last_pose, current_pose);
  printw("distancd: %.8f\n\r", distance);
  refresh();
  auto angle = angleBetween(last_pose, current_pose);
  printw("angle: %.8f\n\r", angle);
  refresh();

  printw("distance_threshold: %.5f\n", distance_threshold);
  if ((angle > angle_threshold) &&
      (distance < angle_factor * distance_threshold)) {
    if (last_pose.yaw() < current_pose.yaw()) {
      return leftturning;
    } else {
      return rightturning;
    }
  }
  if (distance < distance_threshold) {
    return stopping;
  } else {
    if (ifBackward(last_pose, current_pose)) {
      return backwarding;
    } else {
      return forwarding;
    }
  }
}

void AirBase::moveToByTrack(Point targetPoint) {
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
  // MoveOptionFlag(MoveOptionFlagKeyPoints);
  moveAction =
      platform.moveTo(Location(targetPoint.x(), targetPoint.y(), 0), options);
  moveAction.waitUntilDone();
  platform.clearLines(ArtifactUsageVirtualTrack);
}

void AirBase::moveToOrigin() {
  setBaseLockState(true);
  setBaseLockState(true);
  platform.clearLines(ArtifactUsageVirtualTrack);
  MoveAction action = platform.getCurrentAction();
  std::cout << "\nMoving to origin" << std::endl;
  moveToByTrack(Point(0, 0));
  action = platform.rotateTo(Rotation(0, 0, 0));
  action.waitUntilDone();
  std::cout << std::endl;
}

void AirBase::buildStcmMap(std::string map_savepath) {
  char ch;
  std::string tmp;
  std::cout << "Rebuild map\n" << std::endl;
  std::cout << "--------Begin build map-------------\n" << std::endl;
  setBaseLockState(false);
  std::cout << "\nPlease move the robot base to the origin and press Enter"
            << std::endl;
  getline(std::cin, tmp);
  platform.setPose(Pose(Location(0, 0), Rotation(0, 0, 0)));
  platform.clearMap();
  std::cout << "Origin setted. Look RoboStudio for checking map status"
            << std::endl;
  std::cout << "\nPlease move the robot base to build map, and press 's' "
               "to finish build and save map, press 'o' to reset orgin"
            << std::endl;
  while (true) {
    std::cin >> ch;
    if (ch == 's') {
      StcmMapWriter(map_savepath);
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
  std::cout << "Newmap saved" << std::endl;

  printPose();

  std::cout << "--------End build map-------------\n" << std::endl;
  setBaseLockState(true);

  std::cout << "input 'o' move to origin, else to skip" << std::endl;
  std::cin >> ch;
  std::cin.ignore();
  if (ch == 'o')
    moveToOrigin();

  printPose();
}

void AirBase::loadStcmMap(std::string map_path) {
  setBaseLockState(true);
  setBaseLockState(true);
  std::cout << "Load map\n" << std::endl;
  std::cout << "--------Begin load map-------------" << std::endl;
  StcmMapReader(map_path);
  std::cout << "\nMap loaded!\n" << std::endl;

  std::cout << "Begin localization ...\n" << std::endl;
  MoveAction action = platform.getCurrentAction();
  if (action)
    action.cancel();
  action = platform.recoverLocalization(rpos::core::RectangleF(0, 0, 10, 10));
  action.waitUntilDone();

  std::cout << "Localization finished\n" << std::endl;

  printPose();

  moveToOrigin();

  printPose();

  std::cout << "--------End load map-------------" << std::endl;
}

void AirBase::record(std::string task_name, int max_time_steps, int frequency,
                     int start_episode) {
  static int episode = start_episode;
  angle_threshold = 10.0 / frequency;
  distance_threshold = 0.01 / frequency;
  setBaseLockState(false);
  initscr();
  noecho();
  cbreak();
  nodelay(stdscr, TRUE);
  start_color();
  init_pair(1, COLOR_CYAN, COLOR_BLACK);
  init_pair(2, COLOR_RED, COLOR_BLACK);
  init_pair(3, COLOR_GREEN, COLOR_BLACK);

  int ch;
  int current_time_steps = 0;

  while ((ch = getch()) != 'q' || (ch = getch()) != 'Q') {
    clear();
    printw("Data Collect Mode [episode: %d]\n\r\n\r"
           "Press 'Space' to start/stop collect data\n\r"
           "Press 'd' to drop collected data\n\r"
           "Press 's' to save collected data\n\r"
           "Press 'r' to remember a position\n\r"
           "Press 'o' to move the RobotBase to the remembered position\n\r"
           "Press 'q' to quit data collect mode \n\r\n\r",
           episode);

    if (ch == ' ') {
      recording = !recording;
    }

    if (recording) {
      if (current_time_steps == 0) {
        lastTimestamp = getCurrentTime();
        lastPose = platform.getPose();
      }
      if (current_time_steps <= max_time_steps) {
        currentTimestamp = getCurrentTime();
        timestampVec.emplace_back(currentTimestamp);
        currentPose = platform.getPose();
        poseVec.emplace_back(currentPose);
        currentBehavior = getBehavior(lastPose, currentPose);
        behaviorVec.emplace_back(currentBehavior);
        if (getCurrentTime() - lastTimestamp < 1000.0 / frequency) {
          boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
        }
        lastTimestamp = currentTimestamp;
        lastPose = currentPose;
        attron(COLOR_PAIR(1));
        printw("Recording\n\r");
        printw("Collecting %d/%d steps\n\r", current_time_steps,
               max_time_steps);
        attroff(COLOR_PAIR(1));
        switch (currentBehavior) {
        case forwarding:
          printw("\n\rcar is forwarding   |^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n\r");
          refresh();
          break;
        case backwarding:
          printw("\n\rcar is backwarding  |vvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n\r");
          refresh();
          break;
        case leftturning:
          printw("\n\rcar is leftturning  |<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\r");
          refresh();
          break;
        case rightturning:
          printw("\n\rcar is rightturning |>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n\r");
          refresh();
          break;
        case stopping:
          printw("\n\rcar is stoping      |.............................\n\r");
          refresh();
          break;
        }
        current_time_steps++;
      } else {
        attron(COLOR_PAIR(3));
        printw("\n\rdata collected.\n\r");
        attroff(COLOR_PAIR(3));
        if (ch == 's') {
          std::string dataname = "base_data/raw/" + task_name + "/" +
                                 std::to_string(episode++) + ".json";
          saveDataToJson(dataname);
          poseVec.clear();
          timestampVec.clear();
          behaviorVec.clear();
          vecSize = 0;
          current_time_steps = 0;
          recording = false;
          //
          attron(COLOR_PAIR(3));
          printw(("\ncollected data saved to " + dataname + "\n\r").c_str());
          attroff(COLOR_PAIR(3));
          refresh();
          boost::this_thread::sleep_for(boost::chrono::milliseconds(1500));
          break;
        }
        refresh();
      }
    } else {
      attron(COLOR_PAIR(2));
      printw("not recording \n\r");
      attroff(COLOR_PAIR(2));
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }
    if (ch == 'd') {
      poseVec.clear();
      timestampVec.clear();
      behaviorVec.clear();
      vecSize = 0;
      current_time_steps = 0;
      recording = false;
      //
      attron(COLOR_PAIR(2));
      printw("data dropped !\n\r");
      attroff(COLOR_PAIR(2));
      refresh();
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1500));
    }
  }
  endwin();
  return;
}

void AirBase::replay(std::string data_path) {
  setBaseLockState(true);
  setBaseLockState(true);
  std::cout << "------------ replay start ------------\n " << std::endl;
  MoveAction action = platform.getCurrentAction();
  loadDataFromJson("base_data/raw/test/0.json");

  std::vector<Line> lines;
  for (size_t i = 0; i < vecSize - 1; ++i) {
    lines.emplace_back(Line(Point(poseVec[i].x(), poseVec[i].y()),
                            Point(poseVec[i + 1].x(), poseVec[i + 1].y())));
  }
  platform.clearLines(ArtifactUsageVirtualTrack);
  platform.addLines(ArtifactUsageVirtualTrack, lines);

  std::vector<Pose> poseTogo;
  std::vector<int64_t> timestampTogo;
  std::vector<int> behaviorTogo;
  for (int i = 0; i < vecSize - 1; i++) {
    if (behaviorVec[i] == behaviorVec[i + 1]) {
      continue;
    }
    poseTogo.emplace_back(poseVec[i]);
    timestampTogo.emplace_back(timestampVec[i]);
    behaviorTogo.emplace_back(behaviorVec[i]);
  }

  auto beginTime = getCurrentTime();
  for (size_t i = 0; i < poseTogo.size(); i++) {

    rpos::actions::MoveAction moveAction = platform.getCurrentAction();
    MoveOptions options;
    options.flag =
        MoveOptionFlag(MoveOptionFlagKeyPoints | MoveOptionFlagPrecise);
    // MoveOptionFlag(MoveOptionFlagKeyPoints);

    switch (behaviorTogo[i]) {
    case forwarding:
    case backwarding:
    case stopping: {
      std::cout << "\n[" << i << "/" << poseTogo.size() - 1 << "] Moving to: ";
      std::cout << "x: " << poseTogo[i].x() << ", ";
      std::cout << "y: " << poseTogo[i].y() << std::endl;
      moveAction = platform.moveTo(
          Location(poseTogo[i].x(), poseTogo[i].y(), 0), options);
      moveAction.waitUntilDone();
    } break;
    case leftturning:
    case rightturning: {
      std::cout << "\n[" << i << "/" << poseTogo.size() - 1
                << "] Rotating to: ";
      std::cout << "yaw: " << poseTogo[i].yaw() << std::endl;
      moveAction = platform.rotateTo(Rotation(poseTogo[i].yaw(), 0, 0));
      moveAction.waitUntilDone();
    } break;
    // case backwarding: {
    //   // options.mode =
    //   //     NavigationMode(NavigationModeStrictVirtualTrackReverseWalk);
    //   moveAction = platform.moveTo(
    //       Location(poseTogo[i].x(), poseTogo[i].y(), 0), options);
    //   moveAction.waitUntilDone();
    // }
    default:
      break;
    }

    auto currentTime = getCurrentTime();
    while ((currentTime - beginTime) < (timestampTogo[i] - timestampVec[0])) {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
      currentTime = getCurrentTime();
    }
  }
  std::cout << "------------ replay finish ------------\n " << std::endl;
}

#include "airbase.hpp"

const char *ipReg = "\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}";
std::string ipaddress = "";

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

inline time_t getCurrentTime() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

AirBase::AirBase(int argc, const char *argv[]) {
  if (!parseCommandLine(argc, argv)) {
    return;
  }
  try {
    platform = SlamwareCorePlatform::connect(argv[1], 1445);
    std::cout << "SDK Version: " << platform.getSDKVersion() << std::endl;
    std::cout << "SDP Version: " << platform.getSDPVersion() << std::endl;
  } catch (ConnectionTimeOutException &e) {
    std::cerr << e.what() << std::endl;
    return;
  } catch (ConnectionFailException &e) {
    std::cerr << e.what() << std::endl;
    return;
  }
  std::cout << "Connection Successfully" << std::endl;
  printPose();
  int battPercentage = platform.getBatteryPercentage();
  std::cout << "Battery: " << battPercentage << "%" << std::endl;
  std::cout << "\n ------------- robot base inited ------------- \n"
            << std::endl;
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

bool AirBase::getBaseLockState() { return base_lock; }
void AirBase::setBaseLockState(bool lockState) {
  base_lock = lockState;
  if (base_lock) {
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

float AirBase::distanceBetween(Pose pose1, Pose pose2) {
  return std::sqrt((std::pow(pose1.x() - pose2.x(), 2) +
                    std::pow(pose1.y() - pose2.y(), 2)));
}

float AirBase::angleBetween(Pose pose1, Pose pose2) {
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
  auto angle = angleBetween(last_pose, current_pose);
  if ((angle > angle_threshold) &&
      (distance < angle_factor * distance_threshold)) {
    std::cout << "angle: " << std::fixed << std::setprecision(5) << angle
              << "\n\rcar is rotating     |⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳⟳\n\r";
    return rotating;
  }
  if (distance < distance_threshold) {

    std::cout << "distance: " << std::fixed << std::setprecision(5) << distance
              << "\n\rcar is stoping      |...............................\n\r";
    return stoping;

  } else {
    std::cout << "distance: " << std::fixed << std::setprecision(5) << distance;
    if (ifBackward(last_pose, current_pose)) {
      std::cout << "\n\rcar is backwarding  |▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼\n\r";
      return backwarding;
    } else {
      std::cout << "\n\rcar is moving       |▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲\n\r";
      return moving;
    }
  }
  refresh();
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

void AirBase::teach(std::string data_savepath) {
  setBaseLockState(false);
  initscr();
  noecho();
  cbreak();
  nodelay(stdscr, TRUE);
  printw("Press SPACE to start/stop. Press 'q' to drop data. Press 's' to save "
         "data and return.\n");
  refresh();
  start_color(); // Enable color
  init_pair(1, COLOR_CYAN, COLOR_BLACK);
  init_pair(2, COLOR_RED, COLOR_BLACK);
  init_pair(3, COLOR_GREEN, COLOR_BLACK);

  bool running = false;
  int ch;

  while ((ch = getch()) != 'q') {
    clear();
    printw("Press SPACE to start/stop collect data\n\r"
           "Press 'd' to drop data\n\r"
           "Press 's' to save data\n\r"
           "Press 'q' to quit\n\r\n\r");

    if (ch == ' ') {
      running = !running;
    }

    if (running) {
      attron(COLOR_PAIR(1)); // Turn on blue color attribute
      printw("Recording\n\r");
      attroff(COLOR_PAIR(1)); // Turn off blue color attribute
    } else {
      attron(COLOR_PAIR(2)); // Turn on red color attribute
      printw("Not recording.\n\r");
      attroff(COLOR_PAIR(2)); // Turn off red color attribute
    }

    if(vecSize != 0){
      attron(COLOR_PAIR(3)); // Turn on green color attribute
      printw("\n\rdata collected.\n\r");
      attroff(COLOR_PAIR(3)); // Turn off green color attribute
    }

    if (running) {
      currentPose = platform.getPose();
      int64_t current_time = getCurrentTime();
      poseVec.emplace_back(currentPose);
      timestampVec.emplace_back(current_time);
      behaviorVec.emplace_back(getBehavior(lastPose, currentPose));
      lastPose = currentPose;
      boost::this_thread::sleep_for(boost::chrono::milliseconds(save_delay));
    } else if (ch == 's') {
      saveDataToJson(data_savepath);
      break;
    } else if (ch == 'd') {
      poseVec.clear();
      timestampVec.clear();
      behaviorVec.clear();
      vecSize = 0;
      printw("Data dropped !!!");
      refresh();
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1500));
    } else {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
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
  loadDataFromJson(data_path);

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
    case moving:
    case stoping: {
      std::cout << "\n[" << i << "/" << poseTogo.size() - 1 << "] Moving to: ";
      std::cout << "x: " << poseTogo[i].x() << ", ";
      std::cout << "y: " << poseTogo[i].y() << std::endl;
      moveAction = platform.moveTo(
          Location(poseTogo[i].x(), poseTogo[i].y(), 0), options);
      moveAction.waitUntilDone();
    } break;
    case rotating: {
      std::cout << "\n[" << i << "/" << poseTogo.size() - 1
                << "] Rotating to: ";
      std::cout << "yaw: " << poseTogo[i].yaw() << std::endl;
      moveAction = platform.rotateTo(Rotation(poseTogo[i].yaw(), 0, 0));
      moveAction.waitUntilDone();
    } break;
    case backwarding: {
      // options.mode =
      //     NavigationMode(NavigationModeStrictVirtualTrackReverseWalk);
      moveAction = platform.moveTo(
          Location(poseTogo[i].x(), poseTogo[i].y(), 0), options);
      moveAction.waitUntilDone();
    }
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

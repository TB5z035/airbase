#include "airbase.hpp"
#include "argparse/argparse.hpp"
#include <filesystem>

namespace fs = std::filesystem;

int main(int argc, const char *argv[]) {
  argparse::ArgumentParser program("airbase");
  program.add_argument("-tn", "--task_name")
      .help("the baudrate for communication with Robotbase")
      .required();
  program.add_argument("-mn", "--map_name")
      .help("the map name to build/load")
      .required();
  program.add_argument("-mts", "--max_time_steps")
      .scan<'i', int>()
      .help("the max time steps for each episode")
      .required();
  program.add_argument("-f", "--data_collection_frequency")
      .scan<'i', int>()
      .help("the frequency of data collection")
      .default_value(10);
  program.add_argument("-se", "--starting_episode")
      .scan<'i', int>()
      .help("the starting episode")
      .default_value(0);
  program.add_argument("-sp", "--speed")
      .help("the speed of the RobotBase")
      .default_value("high")
      .choices("low", "medium", "high");
  program.add_argument("-ip", "--ip_address")
      .help("the ip address of Robotbase")
      .default_value("192.168.11.1");
  try {
    program.parse_args(argc, const_cast<char **>(argv));
  } catch (const std::runtime_error &err) {
    std::cout << err.what() << std::endl;
    std::cout << program.help().str();
    exit(0);
  }

  std::string task_name = program.get("-tn");
  std::string map_name = program.get("-mn");
  int max_time_steps = program.get<int>("-mts");
  int frequency = program.get<int>("-f");
  int start_episode = program.get<int>("-se");
  std::string speed = program.get("-sp");
  std::string ip = program.get("-ip");

  std::string data_path = "base_data/raw/" + task_name;
  std::string map_path = "base_data/maps";

  if (!fs::exists(data_path)) {
    try {
      fs::create_directories(data_path);
      std::cout << "Folder '" << data_path << "' created successfully.\n";
    } catch (const std::exception &e) {
      std::cerr << "Error creating folder: " << e.what() << std::endl;
      return 1;
    }
  } else {
    std::cout << "Folder '" << data_path << "' already exists.\n";
  }

  if (!fs::exists(map_path)) {
    try {
      fs::create_directories(map_path);
      std::cout << "Folder '" << map_path << "' created successfully.\n";
    } catch (const std::exception &e) {
      std::cerr << "Error creating folder: " << e.what() << std::endl;
      return 1;
    }
  } else {
    std::cout << "Folder '" << map_path << "' already exists.\n";
  }

  AirBase airbase(ip);

  airbase.platform.setSystemParameter(SYSPARAM_ROBOT_SPEED, speed);
  airbase.platform.setSystemParameter(SYSPARAM_ROBOT_ANGULAR_SPEED, speed);

  // set the map
  char ch;
  bool baseLock_ = true;
  while (true) {
    std::cout << "\n---------------------------------" << std::endl;
    if (airbase.getBaseLockState() == true) {
      std::cout << "\033[32m[ locked ] \033[0m" << std::endl;
    } else {
      std::cout << "\033[31m[ unlocked ]\033[0m" << std::endl;
    }
    std::cout << "Please select option:" << std::endl;
    std::cout << "0. unlock/lock the Robotbase" << std::endl;
    std::cout << "1. rebuild the map" << std::endl;
    std::cout << "2. load the map from local file" << std::endl;
    std::cin >> ch;
    std::cin.ignore();
    if (ch == '0') {
      baseLock_ = !baseLock_;
    } else if (ch == '1') {
      airbase.buildStcmMap(map_path + "/" + map_name + ".stcm");
      break;
    } else if (ch == '2') {
      if (airbase.getBaseLockState() == false) {
        std::cout << "\nPlease lock the Robotbase first." << std::endl;
        continue;
      }
      airbase.loadStcmMap(map_path + "/" + map_name + ".stcm");
      break;
    }
    airbase.setBaseLockState(baseLock_);
  }

  while (true) {
    int current_episode = airbase.getCurrentEpisode();
    // teach and replay
    std::cout << "\n---------------------------------" << std::endl;
    if (airbase.getBaseLockState() == true) {
      std::cout << "\033[32m[ locked ] \033[0m" << std::endl;
    } else {
      std::cout << "\033[31m[ unlocked ]\033[0m" << std::endl;
    }
    std::cout << "Please select option:" << std::endl;
    std::cout << "0. unlock/lock the Robotbase" << std::endl;
    std::cout << "1. move the Robotbase to origin" << std::endl;
    std::cout << "2. record the Robotbase action \033[36m[ episode " +
                     std::to_string(current_episode) + " ]\033[0m"
              << std::endl;
    std::cout << "3. replay the Robotbase action" << std::endl;
    std::cin >> ch;
    std::cin.ignore();
    if (ch == '0') {
      baseLock_ = !baseLock_;
    } else if (ch == '1') {
      airbase.moveToOrigin();
    } else if (ch == '2') {
      airbase.record(task_name, max_time_steps, frequency, start_episode);
    } else if (ch == '3') {
      if (airbase.getBaseLockState() == false) {
        std::cout << "\nPlease lock the Robotbase first." << std::endl;
        continue;
      }
      std::cout << "\nInput the episode num to replay:\n";
      try {
        for (const auto &entry : fs::directory_iterator(data_path)) {
          if (entry.is_regular_file()) {
            std::cout << entry.path().filename().string() << std::endl;
          }
        }
      } catch (const std::exception &e) {
        std::cerr << "error: " << e.what() << std::endl;
      }
      int index;
      std::cin >> index;
      airbase.replay(data_path + "/" + std::to_string(index) + ".json");
    }
    airbase.setBaseLockState(baseLock_);
  }
  return 0;
}

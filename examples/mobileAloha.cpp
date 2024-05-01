#include <filesystem>

#include "airbase.hpp"
#include "argparse/argparse.hpp"

namespace fs = std::filesystem;

void create_dir(const std::string &dir);
bool check_path(const std::string &path);

int main(int argc, const char *argv[]) {
  argparse::ArgumentParser program("airbase");
  program.add_argument("-tn", "--task_name").help("the task name that will be used to in saving/loading directory").required();
  program.add_argument("-mn", "--map_name").help("the map name to build/load").required();
  program.add_argument("-mts", "--max_time_steps")
      .scan<'i', int>()
      .help("the max time steps for each episode")
      .required();
  program.add_argument("-f", "--data_collection_frequency")
      .scan<'i', int>()
      .help("the frequency of data collection")
      .default_value(10);
  program.add_argument("-se", "--starting_episode").scan<'i', int>().help("the starting episode").default_value(0);
  program.add_argument("-sp", "--speed")
      .help("the speed of the RobotBase")
      .default_value("high")
      .choices("low", "medium", "high");
  program.add_argument("-ip", "--ip_address").help("the ip address of RobotBase").default_value("192.168.11.1");
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

  std::string data_dir = "base_data/raw/" + task_name;
  std::string maps_dir = "base_data/maps";

  create_dir(data_dir);
  create_dir(maps_dir);

  AirBase airbase(ip);
  // set speed
  airbase.platform.setSystemParameter(SYSPARAM_ROBOT_SPEED, speed);
  airbase.platform.setSystemParameter(SYSPARAM_ROBOT_ANGULAR_SPEED, speed);

  // set the map
  char ch;
  bool baseLock_ = true;
  while (true) {
    if (airbase.is_emergency_stop()) {
      std::cout << "\n\033[31mEmergency stop triggered, please release the emergency stop.\033[0m" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }
    std::cout << "\n---------------------------------" << std::endl;
    if (airbase.get_baselock_state()) {
      std::cout << "\033[32m[ locked ] \033[0m" << std::endl;
    } else {
      std::cout << "\033[31m[ unlocked ]\033[0m" << std::endl;
    }
    std::cout << "Please select option:" << std::endl;
    std::cout << "0. unlock/lock the RobotBase" << std::endl;
    std::cout << "1. rebuild the map" << std::endl;
    std::cout << "2. load the map from local file" << std::endl;
    std::cout << "You can press 'Ctrl + C' to exit the program." << std::endl;
    std::cin >> ch;
    std::cin.ignore();
    if (ch == '0') {
      baseLock_ = !baseLock_;
    } else if (ch == '1') {
      airbase.build_stcm_map(maps_dir + "/" + map_name + ".stcm");
      break;
    } else if (ch == '2') {
      airbase.set_baselock_state(false);
      std::cout << "Please manually move the car near the origin and press any key and Enter to continue." << std::endl;
      std::cin >> ch;
      std::cin.ignore();
      airbase.load_stcm_map(maps_dir + "/" + map_name + ".stcm");
      break;
    }
    airbase.set_baselock_state(baseLock_);
  }

  // teach and replay
  airbase.set_start_episode(start_episode);
  while (true) {
    if (airbase.is_emergency_stop()) {
      std::cout << "\n\033[31mEmergency stop triggered, please release the emergency stop.\033[0m" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }
    int current_episode = airbase.get_current_episode();
    std::cout << "\n---------------------------------" << std::endl;
    if (airbase.get_baselock_state() == true) {
      std::cout << "\033[32m[ locked ] \033[0m" << std::endl;
    } else {
      std::cout << "\033[31m[ unlocked ]\033[0m" << std::endl;
    }
    std::cout << "Please select option:" << std::endl;
    std::cout << "0. unlock/lock the RobotBase" << std::endl;
    std::cout << "1. move the RobotBase to origin" << std::endl;
    std::cout << "2. record the RobotBase action \033[36m[ episode " + std::to_string(current_episode) + " ]\033[0m"
              << std::endl;
    std::cout << "3. replay the RobotBase action" << std::endl;
    std::cout << "You can press 'Ctrl + C' to exit the program." << std::endl;
    std::cin >> ch;
    std::cin.ignore();
    if (ch == '0') {
      baseLock_ = !baseLock_;
    } else if (ch == '1') {
      airbase.move_to_origin();
    } else if (ch == '2') {
      airbase.record_trajectory(data_dir, max_time_steps, frequency);
    } else if (ch == '3') {
      if (airbase.get_baselock_state() == false) {
        std::cout << "\nPlease lock the RobotBase first." << std::endl;
        continue;
      }
      std::cout << "\nInput the episode num to replay or any other key to choose another option:\n";
      try {
        for (const auto &entry : fs::directory_iterator(data_dir)) {
          if (entry.is_regular_file()) {
            std::cout << entry.path().filename().string() << std::endl;
          }
        }
      } catch (const std::exception &e) {
        std::cerr << "error: " << e.what() << std::endl;
      }
      char index;
      std::cin >> index;
      std::cin.ignore();
      std::string path = data_dir + "/" + index + ".json";
      if (!check_path(path)) {
        std::cout << "Invalid episode num:" << index << std::endl;
        continue;
      }
      std::cout << "Replaying: " << path << std::endl;
      airbase.replay_trajectory(path);
    }
    airbase.set_baselock_state(baseLock_);
  }
  return 0;
}

bool check_path(const std::string &path) {
  return fs::exists(path);
}

void create_dir(const std::string &dir) {
  if (!fs::exists(dir)) {
    try {
      fs::create_directories(dir);
      std::cout << "Folder '" << dir << "' created successfully.\n";
    } catch (const std::exception &e) {
      std::cerr << "Error creating folder: " << e.what() << std::endl;
    }
  } else {
    std::cout << "Folder '" << dir << "' already exists.\n";
  }
}
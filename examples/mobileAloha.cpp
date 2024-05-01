#include <filesystem>

#include "airbase.hpp"
#include "argparse/argparse.hpp"
<<<<<<< HEAD
=======
#include <filesystem>
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7

namespace fs = std::filesystem;

int main(int argc, const char *argv[]) {
  argparse::ArgumentParser program("airbase");
<<<<<<< HEAD
  program.add_argument("-tn", "--task_name").help("the baudrate for communication with RobotBase").required();
  program.add_argument("-mn", "--map_name").help("the map name to build/load").required();
=======
  program.add_argument("-tn", "--task_name")
      .help("the baudrate for communication with Robotbase")
      .required();
  program.add_argument("-mn", "--map_name")
      .help("the map name to build/load")
      .required();
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7
  program.add_argument("-mts", "--max_time_steps")
      .scan<'i', int>()
      .help("the max time steps for each episode")
      .required();
  program.add_argument("-f", "--data_collection_frequency")
      .scan<'i', int>()
      .help("the frequency of data collection")
      .default_value(10);
<<<<<<< HEAD
  program.add_argument("-se", "--starting_episode").scan<'i', int>().help("the starting episode").default_value(0);
=======
  program.add_argument("-se", "--starting_episode")
      .scan<'i', int>()
      .help("the starting episode")
      .default_value(0);
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7
  program.add_argument("-sp", "--speed")
      .help("the speed of the RobotBase")
      .default_value("high")
      .choices("low", "medium", "high");
<<<<<<< HEAD
  program.add_argument("-ip", "--ip_address").help("the ip address of RobotBase").default_value("192.168.11.1");
=======
  program.add_argument("-ip", "--ip_address")
      .help("the ip address of Robotbase")
      .default_value("192.168.11.1");
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7
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
<<<<<<< HEAD
  std::string map_path = "base_data/maps";
=======
  std::string map_path = "base_data/maps/" + map_name;
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7

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
    if (airbase.get_baselock_state() == true) {
      std::cout << "\033[32m[ locked ] \033[0m" << std::endl;
    } else {
      std::cout << "\033[31m[ unlocked ]\033[0m" << std::endl;
    }
    std::cout << "Please select option:" << std::endl;
<<<<<<< HEAD
    std::cout << "0. unlock/lock the RobotBase" << std::endl;
=======
    std::cout << "0. unlock/lock the Robotbase" << std::endl;
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7
    std::cout << "1. rebuild the map" << std::endl;
    std::cout << "2. load the map from local file" << std::endl;
    std::cin >> ch;
    std::cin.ignore();
    if (ch == '0') {
      baseLock_ = !baseLock_;
    } else if (ch == '1') {
<<<<<<< HEAD
      airbase.build_stcm_map(map_path + "/" + map_name + ".stcm");
      break;
    } else if (ch == '2') {
      if (airbase.get_baselock_state() == false) {
        std::cout << "\nPlease lock the RobotBase first." << std::endl;
        continue;
      }
      airbase.load_stcm_map(map_path + "/" + map_name + ".stcm");
      break;
    }
    airbase.set_baselock_state(baseLock_);
=======
      airbase.buildStcmMap("maps/" + map_name + ".stcm");
      break;
    } else if (ch == '2') {
      airbase.loadStcmMap("maps/" + map_name + ".stcm");
      break;
    }
    airbase.setBaseLockState(baseLock_);
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7
  }
  int current_episode = airbase.get_current_episode();
  while (true) {
    // teach and replay
    std::cout << "\n---------------------------------" << std::endl;
    if (airbase.get_baselock_state() == true) {
      std::cout << "\033[32m[ locked ] \033[0m" << std::endl;
    } else {
      std::cout << "\033[31m[ unlocked ]\033[0m" << std::endl;
    }
    std::cout << "Please select option:" << std::endl;
<<<<<<< HEAD
    std::cout << "0. unlock/lock the RobotBase" << std::endl;
    std::cout << "1. move the RobotBase to origin" << std::endl;
    std::cout << "2. record the RobotBase action \033[36m[ episode " + std::to_string(current_episode) + " ]\033[0m"
              << std::endl;
    std::cout << "3. replay the RobotBase action" << std::endl;
=======
    std::cout << "0. unlock/lock the Robotbase" << std::endl;
    std::cout << "1. move the Robotbase to origin" << std::endl;
    std::cout << "2. record the Robotbase action" << std::endl;
    std::cout << "3. replay the Robotbase action" << std::endl;
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7
    std::cin >> ch;
    if (ch == '0') {
      baseLock_ = !baseLock_;
    } else if (ch == '1') {
      airbase.move_to_origin();
    } else if (ch == '2') {
<<<<<<< HEAD
      airbase.record_trajectory(task_name, max_time_steps, frequency, start_episode);
    } else if (ch == '3') {
      if (airbase.get_baselock_state() == false) {
        std::cout << "\nPlease lock the RobotBase first." << std::endl;
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
      airbase.replay_trajectory(data_path + "/" + std::to_string(index) + ".json");
    }
    airbase.set_baselock_state(baseLock_);
=======
      airbase.record(task_name, max_time_steps, frequency, start_episode);
    } else if (ch == '3') {
      airbase.replay(task_name);
    }
    airbase.setBaseLockState(baseLock_);
>>>>>>> 4ca2b4989a82a45e29262089055b3a83f758efe7
  }
  return 0;
}

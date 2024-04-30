#include "airbase.hpp"

int main(int argc, const char *argv[]) {

  AirBase airbase(argc, argv);

  airbase.platform.setSystemParameter(SYSPARAM_ROBOT_SPEED,
                                      SYSVAL_ROBOT_SPEED_HIGH);
  airbase.platform.setSystemParameter(SYSPARAM_ROBOT_ANGULAR_SPEED,
                                      SYSVAL_ROBOT_SPEED_HIGH);

  // set the map
  char ch;
  bool base_lock = true;
  while (true) {
    std::cout << "Please select option:" << std::endl;
    std::cout << "0. unlock/lock the base" << std::endl;
    std::cout << "1. rebuild the map" << std::endl;
    std::cout << "2. load the map from local file(please move car close to the "
                 "origin)"
              << std::endl;
    std::cin >> ch;
    std::cin.ignore();
    if (ch == '0') {
      base_lock = !base_lock;
    } else if (ch == '1') {
      airbase.buildStcmMap("map.stcm");
      break;
    } else if (ch == '2') {
      airbase.loadStcmMap("map.stcm");
      break;
    }
    airbase.setBaseLockState(base_lock);
  }

  while (true) {
    // teach and replay
    std::cout << "\n---------------------------------" << std::endl;
    std::cout << "Please select option:" << std::endl;
    std::cout << "0. unlock/lock the base" << std::endl;
    std::cout << "1. base move to the origin of the map" << std::endl;
    std::cout << "2. teach the base action" << std::endl;
    std::cout << "3. replay the base action" << std::endl;
    std::cin >> ch;
    std::cin.ignore();
    if (ch == '0') {
      base_lock = !base_lock;
    } else if (ch == '1') {
      airbase.moveToOrigin();
    } else if (ch == '2') {
      airbase.teach("movedata.json");
    } else if (ch == '3') {
      airbase.replay("movedata.json");
    }
    airbase.setBaseLockState(base_lock);
  }
  return 0;
}

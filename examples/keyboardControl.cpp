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
  airbase.setBaseLockState(base_lock);
  while (true) {
    std::cout << "Please select option:" << std::endl;
    std::cout << "0. unlock the base" << std::endl;
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

  // begin keyboardControl mode
  initscr();
  noecho();
  cbreak();
  nodelay(stdscr, TRUE);

  bool running = false;

  Direction forward(ACTION_DIRECTION::FORWARD);
  Direction backward(ACTION_DIRECTION::BACKWARD);
  Direction turnleft(ACTION_DIRECTION::TURNLEFT);
  Direction turnright(ACTION_DIRECTION::TURNRIGHT);

  while ((ch = getch()) != 'q') {
    clear();
    printw("Keyboard Control Mode, Press 'q' to quit.\n");
    printw("Press 'w' to move forward\n"
           "Press 's' to move backward\n"
           "Press 'a' to turn left\n"
           "Press 'd' to turn right\n");
    switch (ch) {
    case 'w':
      airbase.platform.moveBy(forward);
      break;
    case 's':
      airbase.platform.moveBy(backward);
      break;
    case 'a':
      airbase.platform.moveBy(turnleft);
      break;
    case 'd':
      airbase.platform.moveBy(turnright);
      break;
    default:
      break;
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
  }
  endwin();
  
  return 0;
}
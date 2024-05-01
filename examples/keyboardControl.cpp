#include "airbase.hpp"
#include "argparse/argparse.hpp"

int main(int argc, const char *argv[]) {
  argparse::ArgumentParser program("airbase");
  program.add_argument("-mp", "--map_path").help("the .stcm map path to build/load").default_value("map.stcm");
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

  std::string map_path = program.get("-mp");
  std::string speed = program.get("-sp");
  std::string ip = program.get("-ip");

  AirBase airbase(ip);

  airbase.platform.setSystemParameter(SYSPARAM_ROBOT_SPEED, speed);
  airbase.platform.setSystemParameter(SYSPARAM_ROBOT_ANGULAR_SPEED, speed);

  // set the map
  char ch;
  bool base_lock = true;
  airbase.set_baselock_state(base_lock);
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
      airbase.build_stcm_map(map_path);
      break;
    } else if (ch == '2') {
      airbase.load_stcm_map(map_path);
      break;
    }
    airbase.set_baselock_state(base_lock);
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
    printw(
        "Press 'w' to move forward\n"
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
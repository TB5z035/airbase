
// int AirBase::get_current_behavior(Pose last_pose, Pose current_pose) {
//   auto distance = distance_between(last_pose, current_pose);
//   printw("distancd: %.8f\n\r", distance);
//   refresh();
//   auto angle = angle_between(last_pose, current_pose);
//   printw("angle: %.8f\n\r", angle);
//   refresh();

//   printw("distance_threshold: %.5f\n", distance_threshold);
//   if ((angle > angle_threshold) &&
//       (distance < angle_factor * distance_threshold)) {
//     if (last_pose.yaw() < current_pose.yaw()) {
//       return TURNLEFT;
//     } else {
//       return TURNRIGHT;
//     }
//   }
//   if (distance < distance_threshold) {
//     return STOP;
//   } else {
//     if (if_backward(last_pose, current_pose)) {
//       return BACKWARD;
//     } else {
//       return FORWARD;
//     }
//   }
// }

// void AirBase::record_trajectory(std::string task_name, int max_time_steps, int
// frequency,
//                      int start_episode) {
//   static int episode = start_episode;
//   angle_threshold = 10.0 / frequency;
//   distance_threshold = 0.01 / frequency;
//   set_baselock_state(false);
//   initscr();
//   noecho();
//   cbreak();
//   nodelay(stdscr, TRUE);
//   start_color();
//   init_pair(1, COLOR_CYAN, COLOR_BLACK);
//   init_pair(2, COLOR_RED, COLOR_BLACK);
//   init_pair(3, COLOR_GREEN, COLOR_BLACK);

//   int ch;
//   int current_time_steps = 0;

//   while ((ch = getch()) != 'q' || (ch = getch()) != 'Q') {
//     clear();
//     printw("Data Collect Mode [episode: %d]\n\r\n\r"
//            "Press 'Space' to start/stop collect data\n\r"
//            "Press 'd' to drop collected data\n\r"
//            "Press 's' to save collected data\n\r"
//            "Press 'r' to remember a position\n\r"
//            "Press 'o' to move the RobotBase to the remembered position\n\r"
//            "Press 'q' to quit data collect mode \n\r\n\r",
//            episode);

//     if (ch == ' ') {
//       recording = !recording;
//     }

//     if (recording) {
//       if (current_time_steps == 0) {
//         lastTimestamp = getCurrentTime();
//         lastPose = platform.getPose();
//       }
//       if (current_time_steps <= max_time_steps) {
//         currentTimestamp = getCurrentTime();
//         timestamp_vec_.emplace_back(currentTimestamp);
//         currentPose = platform.getPose();
//         pose_vec_.emplace_back(currentPose);
//         currentBehavior = get_current_behavior(lastPose, currentPose);
//         behavior_vec_.emplace_back(currentBehavior);
//         if (getCurrentTime() - lastTimestamp < 1000.0 / frequency) {
//           boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
//         }
//         lastTimestamp = currentTimestamp;
//         lastPose = currentPose;
//         attron(COLOR_PAIR(1));
//         printw("Recording\n\r");
//         printw("Collecting %d/%d steps\n\r", current_time_steps,
//                max_time_steps);
//         attroff(COLOR_PAIR(1));
//         switch (currentBehavior) {
//         case FORWARD:
//           printw("\n\rcar is FORWARD |^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n\r");
//           refresh();
//           break;
//         case BACKWARD:
//           printw("\n\rcar is BACKWARD
//           |vvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n\r"); refresh(); break;
//         case TURNLEFT:
//           printw("\n\rcar is TURNLEFT
//           |<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\r"); refresh(); break;
//         case TURNRIGHT:
//           printw("\n\rcar is TURNRIGHT
//           |>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n\r"); refresh(); break;
//         case STOP:
//           printw("\n\rcar is stoping |.............................\n\r");
//           refresh();
//           break;
//         }
//         current_time_steps++;
//       } else {
//         attron(COLOR_PAIR(3));
//         printw("\n\rdata collected.\n\r");
//         attroff(COLOR_PAIR(3));
//         if (ch == 's') {
//           std::string dataname = "base_data/raw/" + task_name + "/" +
//                                  std::to_string(episode++) + ".json";
//           save_data_to_json(dataname);
//           pose_vec_.clear();
//           timestamp_vec_.clear();
//           behavior_vec_.clear();
//           vec_size_ = 0;
//           current_time_steps = 0;
//           recording = false;
//           //
//           attron(COLOR_PAIR(3));
//           printw(("\ncollected data saved to " + dataname + "\n\r").c_str());
//           attroff(COLOR_PAIR(3));
//           refresh();
//           boost::this_thread::sleep_for(boost::chrono::milliseconds(1500));
//           break;
//         }
//         refresh();
//       }
//     } else {
//       attron(COLOR_PAIR(2));
//       printw("not recording \n\r");
//       attroff(COLOR_PAIR(2));
//       boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
//     }
//     if (ch == 'd') {
//       pose_vec_.clear();
//       timestamp_vec_.clear();
//       behavior_vec_.clear();
//       vec_size_ = 0;
//       current_time_steps = 0;
//       recording = false;
//       //
//       attron(COLOR_PAIR(2));
//       printw("data dropped !\n\r");
//       attroff(COLOR_PAIR(2));
//       refresh();
//       boost::this_thread::sleep_for(boost::chrono::milliseconds(1500));
//     }
//   }
//   endwin();
//   return;
// }
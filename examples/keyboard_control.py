import airbase_py
import sys

params = {
    "SYSPARAM_ROBOT_SPEED": "base.max_moving_speed",
    "SYSPARAM_ROBOT_ANGULAR_SPEED": "base.max_angular_speed",
    "SYSVAL_ROBOT_SPEED_HIGH": "high",
    "SYSVAL_ROBOT_SPEED_MEDIUM": "medium",
    "SYSVAL_ROBOT_SPEED_LOW": "low",
    "SYSPARAM_POWEROFF_WAIT_TIME": "base.power_off_wait_time",
    "SYSPARAM_EMERGENCY_STOP": "base.emergency_stop",
    "SYSVAL_EMERGENCY_STOP_ON": "on",
    "SYSVAL_EMERGENCY_STOP_OFF": "off",
    "SYSPARAM_BRAKE_RELEASE": "base.brake_release",
    "SYSVAL_BRAKE_RELEASE_ON": "on",
    "SYSVAL_BRAKE_RELEASE_OFF": "off",
    "SYSPARAM_DOCKED_REGISTER_STRATEGY": "docking.docked_register_strategy",
    "SYSVAL_DOCKED_REGISTER_STRATEGY_ALWAYS": "always",
    "SYSVAL_DOCKED_REGISTER_STRATEGY_WHEN_NOT_EXISTS": "when_not_exists",
    "SYSPARAM_ACTIONS_WAIT_FOR_ACCEPTABLE_PATH_TIME": "actions.wait_for_acceptable_path_time",
    "SYSPARAM_VIRTUAL_TRACKS_ACCESSIBLE_AREA": "virtual_track.enable_limited_accessible_area",
    "SYSVAL_ACCESSIBLE_AREA_ON": "on",
    "SYSVAL_ACCESSIBLE_AREA_OFF": "off",
}

if __name__ == "__main__":

    sys.argv.append("192.168.11.1")
    airbase = airbase_py.create(sys.argv)
    airbase.platform.set_system_parameter(
        params["SYSPARAM_ROBOT_SPEED"], params["SYSVAL_ROBOT_SPEED_HIGH"]
    )
    airbase.platform.set_system_parameter(
        params["SYSPARAM_ROBOT_ANGULAR_SPEED"], params["SYSVAL_ROBOT_SPEED_HIGH"]
    )

    # set the map
    base_lock = True
    airbase.set_base_lock_state(base_lock)
    while True:
        print("Please select option:")
        print("0. unlock the base")
        print("1. rebuild the map")
        print("2. load the map from local file(please move car close to the origin)")
        key = input("Please input the option and press Enter:")
        if key == "0":
            # 求反
            base_lock = not base_lock
        elif key == "1":
            airbase.build_stcm_map("map.stcm")
            break
        elif key == "2":
            airbase.load_stcm_map("map.stcm")
            break
        airbase.set_base_lock_state(base_lock)

    forward = airbase_py.Direction(airbase_py.ACTION_DIRECTION.FORWARD)
    backward = airbase_py.Direction(airbase_py.ACTION_DIRECTION.BACKWARD)
    turn_left = airbase_py.Direction(airbase_py.ACTION_DIRECTION.TURNLEFT)
    turn_right = airbase_py.Direction(airbase_py.ACTION_DIRECTION.TURNRIGHT)

    while True:
        key = input(
            "Press 'w' 's' 'a' 'd' to move (press Enter to submit), 'q' to quit:"
        )
        if key == "w":
            airbase.move(forward)
        elif key == "s":
            airbase.move(backward)
        elif key == "a":
            airbase.move(turn_left)
        elif key == "d":
            airbase.move(turn_right)
        elif key == "q":
            break

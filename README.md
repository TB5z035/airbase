# SLAMTEC Athena Demonstration & Replay

## Introduction

This project contains the codes about demonstration of SLAMTEC Athena (a  RobotBase) for raw RobotBase 2D pose data collecting and data replaying. Please refer to [Slamtec Cpp Reference](https://developer.slamtec.com/docs/slamware/cpp-sdk/2.8.2_rtm/) and [Slamtec Sdk Examples](https://wiki.slamtec.com/pages/viewpage.action?pageId=13959292) for detailed instructions of the RobotBase itself.

## Environment Setup

```bash
git clone -b main https://github.com/RoboticsChen/airbase.git
git submodule update --init --recursive
mkdir build && cd build
cmake .. && make -j8
pip install . -i https://pypi.mirrors.ustc.edu.cn/simple/
```

## Data Collection && Replay


### 1. Press the Power Button until the RobotBase light is bright.
### 2. Computer connect to the WIFI named "SLAMWARE-XXXXXX"
**Notes:**
>[Environment Setup](#environment-setup) needs to be done first.

> There should be no obstacles and wires around the RobotBase body, otherwise the Lidar will be affected and the RobotBase may not move.


### 3. Starting Data Collection

```bash
mobileAloha -tn <task_name> -mn <map_name.stcm> -mts <max_time_steps> [-f <data_collection_frequency>] [-sp <speed_level>] [-se <starting_episode>] [-ip <ip_address>]
```

#### Explanation of Parameters
`-tn`: Task name, specified by the specific task.
-  The collected data will be saved in the `base_data/raw/<task_name>` folder in the current directory.

`-mn`: The map name to build/load. 

`-mts`: The maximum time steps.
-  Specifies the max time of frames to be captured. depending on the specific task, The total collection duration is 1/f*mts. 
 -  When reaching the maximum time steps, the program will info you, and stop collect data.

`-f`: Data collection frequency, default is 10Hz.

`-sp`: Specify the speed of the RobotBase, options are `low`, `medium`, `high`, default is `high`.

`-se`: Specify the first episode number of this collection, default is 0.
- Each set of data is saved in the `base_data/raw/<task_name>/` folder, named by `<episode_number>.json`.
- After interruption, you can modify this value to continue collection without overwriting previous data.
- When data is mistakenly saved, specifying this episode number allows for re-collection to overwrite existing data.

`-ip`: Specify the IP address of the RobotBase.

#### Excution Example

##### 3.1 Execute this command in the terminal:
```bash
mobileAloha -tn mobileAloha -mn map.stcm -mts 3000 [-f 10] [-sp high] [-se 0] [-ip 192.168.11.1]
```
This command means that you want to collect data for 3000 frames(5min) with a frequency of 10Hz. The first set of collected data will be named "episode_0" 

>Prameters in [ ] are optional
    
##### 3.2 After executing the command, you can see following log_info in the terminal.
```bash
SDK Version: x.x.x
SDP Version: x.x.x
Connection Successfully!
Battery: xx%
```
##### 3.3 Then there are some options you can choose in the terminal:
   
   0. `unlock/lock the Robotbase`: unlock the wheel so that you can manually move the RobotBase, but will not record data
   1. `rebuild the map`: you should build a map in order to record the pose of the RobotBase when you have no map or want to build a new one
   2. `load the map from local file`: when you already built a map, you can load it without rebuild

**Notes:**
> In order to relocalize, please move the RobotBbase close to the origin of the built map before load the map.

##### 3.4 After the map is ready. You can choose the operation you want to do:
   0. `unlock/lock the Robotbase` 
   1. `move the Robotbase to origin`
   2. `record the Robotbase action`
   3. `replay the Robotbase action`
##### 3.5 Data Collect: Input `2` and press the `Enter` to enter data collect mode. 
    Data Collect Mode

    Press `Spacebar` to start/stop collect data
    Press `d` to drop collected data
    Press `s` to save collected data
    Press `q` to quit data collect mode
    Press `r` to remember a position
    Press `o` to move the RobotBase to the remembered position

During the data collection process, you can perform the corresponding operation by pressing the button.

**Notes:**
> The RobotBase movement speed should not be too fast; otherwise, the collected information will be sparse, and the speed of the RobotBase may not be kept in the collection data during the replay or reasoning, resulting in the failure of the reasoning.

##### 3.6 Data Replay: Input `3` and press the `Enter` to enter data replay mode.

> Data replay can be used to verify if there are issues with collected data.

    Data Replay Mode[The selected task_name has total xx collected data]

    Press `r` to choose the data to replay
    Press `s` to stop the replaying data 
    Press `q` to quit data replay mode

During the data replay process, you can perform the corresponding operation by pressing the button.



# SLAMTEC Athena2.0 Demonstration & Replay

## Introduction

This project contains the codes about demonstration of SLAMTEC Athena2.0 (a  RobotBase) for raw RobotBase 2D pose data collecting and data replaying. Please refer to [Slamtec Cpp Reference](https://developer.slamtec.com/docs/slamware/cpp-sdk/2.8.2_rtm/) and [Slamtec Sdk Examples](https://wiki.slamtec.com/pages/viewpage.action?pageId=13959292) for detailed instructions of the RobotBase itself.

## Environment Setup

```bash
git clone --recursive https://github.com/RoboticsChen/airbase.git -b develop_ghz airbase
mkdir build && cd build
cmake .. && make -j8
sudo make install
pip install . -i https://pypi.mirrors.ustc.edu.cn/simple/
```

## Data Collection && Replay

### Starting The Robot Base

> - [Environment Setup](#environment-setup) needs to be done first.
>
> - There should be no obstacles and wires around the RobotBase body, otherwise the Lidar will be affected and the RobotBase may not move.

1.Press the Power Button until the RobotBase light is bright.

2.Computer connects to the WIFI named "SLAMWARE-XXXXXX" (wait for a while to let the WIFI appear).

### Starting Data Collection

The command is as follows:

```bash
airbase_demonstrate -tn <task_name> -mn <map_name.stcm> -mts <max_time_steps> [-f <data_collection_frequency>] [-sp <speed_level>] [-se <starting_episode>] [-ip <ip_address>]
```

#### Explanation of Parameters

> Prameters in `[ ]` are optional

`-tn`: Task name, specified by the specific task.
-  The collected data will be saved in the `base_data/raw/<task_name>` folder in the current directory.

`-mn`: The map name to build/load. 

`-mts`: The maximum time steps.
-  Specifies the max time of frames to be captured. depending on the specific task, The total collection duration is `mts/f` seconds. 
-  When reaching the maximum time steps, the program will info you, and stop collect data.

`-f`: Data collection frequency, default is 10Hz.

`-sp`: Specify the speed of the RobotBase, options are `low`, `medium`, `high`, default is `high`.

`-se`: Specify the first episode number of this collection, default is 0.
- Each set of data is saved in the `base_data/raw/<task_name>/` folder, named by `<episode_number>.json`.
- After interruption, you can modify this value to continue collection without overwriting previous data.
- When data is mistakenly saved, specifying this episode number allows for re-collection to overwrite existing data.

`-ip`: Specify the IP address of the RobotBase WIFI.

#### Excution Example

##### Execute the following command in a terminal:
```bash
airbase_demonstrate -tn mobileAloha -mn map.stcm -mts 3000 -f 15
```
This command means that you want to collect data for 3000 frames with a frequency of 15Hz. The first collected data will be saved to "0.json".

##### After executing the command, you can see following log_info in the terminal:

> Folder 'base_data/raw/mobileAloha' created successfully.
> 
> Folder 'base_data/maps' created successfully.
> 
> SDK Version: x.x.x
> 
> SDP Version: x.x.x
> 
> Connection Successfully!
> 
> Battery: xx%

##### Then there are some options you can choose in the terminal:
   
0.`unlock/lock the Robotbase`: unlock the wheel so that you can manually move the RobotBase, but will not record data

1.`rebuild the map`: you should build a map in order to record the pose of the RobotBase when you have no map or want to build a new one

2.`load the map from local file`: when you already built a map, you can load it without rebuild

> **Note:** In order to relocalize, please move the RobotBbase close to the origin of the built map before load the map.

##### After the map is ready. You can choose the operation you want to do:
0.`unlock/lock the Robotbase` 

1.`move the Robotbase to origin`

2.`record the Robotbase action`

3.`replay the Robotbase action`

##### Data Collection: Input `2` and press `Enter` to enter the data collect mode
> Data Collect Mode
> 
> > Press `Spacebar` to start/stop collect data
> > 
> > Press `d` to drop collected data
> > 
> > Press `s` to save collected data
> > 
> > Press `q` to quit data collect mode
> > 
> > Press `r` to remember a position
> > 
> > Press `o` to move the RobotBase to the remembered position

During the data replay process, you can perform the corresponding operation by pressing the button.

> **Note:** The RobotBase movement speed should not be too fast; otherwise, the collected information will be sparse, and the speed of the RobotBase may not remain the same as demonstration during the replay or inference.

##### Data Replay: Input `3` and press the `Enter` to enter data replay mode.

> You should collect the data first.

> Data Replay Mode[The selected task_name has total xx collected data]
> > Press `r` to choose the data to replay
> 
> > Press `s` to stop the replaying data 
> 
> > Press `q` to quit data replay mode

During the data replay process, you can perform the corresponding operation by pressing the button.

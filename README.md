# POMCP ROS

This repository contains the integration of [POMCP](https://papers.nips.cc/paper/2010/hash/edfbe1afcf9246bb0d40eb4d8027d90f-Abstract.html) within ROS, targeting mobile robots. The architecture has been tested in simulation with Gazebo and TurtleBot3 with the benchmark problem Rocksample.

Overview:

- [Dependencies](#Installation)

- [Configuration](#Configuration)

- [How to run](#How-to-run)

- [References](#References)




https://github.com/kriato/pomcp_mrf_ros/assets/26410045/511585dc-4849-4979-9dd4-7ff76c9cd185




## Installation
Follow this [link](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) to install the missing dependencies to run the benchmark problem with Turtlebot3.

Install other dependencies with these two commands.
```s
sudo apt-get install ros-melodic-dwa-local-planner
rosdep install --from-paths src --ignore-src
```

This package is needed for I/O with .yaml files.
```s
sudo apt install libyaml-cpp-dev
```

Then, proceed to create a catkin workspace, clone this repository and build the workspace. If you already have a workspace, just go to your src folder, clone and then build.

```s
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/kriato/pomcp_ros_mrf.git
cd ..
catkin build
# Always remember to source your workspace :)
source devel/setup.bash 
```

## Configuration

To tweak the behaviour of the ROS Navigation Stack, edit the following files in the _params_ folder:

- base_local_planner_params.yaml
- costmap_common_params.yaml
- dwa_local_planner_params.yaml
- global_costmap_params.yaml
- local_costmap_params.yaml

To change the **number of the grid squares**, the **number of rocks**, or the **size of the grid squares** (in meters), change _shared_params.yml_ file in the _params_ folder.

To change the parameters of the agent, environment or the planner, refer to the corresponding launch files. Each field is commented. Please keep in mind that most of the settings are related to POMCP implementation, and out of scope from this work.

To feed the system with a set of real configurations, please create a file formatted as a sequence of integers separated by a TAB '\t' character and a newline '\n' at the end of each episode. Insert the path in the files mentioned above.

For example the following will be a set of 3 episodes, with 8 binary states (in the case of Rocksample 8 rocks with value [1] or without value[0]):
```
0   1   0   1   0   1   1   1
1   1   1   0   0   1   0   1
0   1   0   0   1   1   0   1
```

## How to run

Prepare four terminals. The use of [Terminator](https://terminator-gtk3.readthedocs.io/en/latest/) is strongly suggested.

__Terminal #1__: run the ROS master.

```s
roscore
```

__Terminal #2__: run the _environment_ node.

```s
roslaunch pomcp_ros env_bringup.launch
```

__Terminal #3__: run the _agent_ node.

```s
roslaunch pomcp_ros agent_bringup.launch
```

__Terminal #4__: run the _planner_ node (or change it to _*testing.launch_ to use a MRF).

```s
roslaunch pomcp_ros rocksample_single_batch_learning.launch
```

## References
Zuccotto, M., Piccinelli, M., Castellini, A., Marchesini, E. and Farinelli, A., 2022. Learning State-Variable Relationships in POMCP: A Framework for Mobile Robots. Frontiers in Robotics and AI, 9

Department of Computer Science, University of Verona


## Contacts
For any information/bugfix/issue contact Marco Piccinelli at picci.19@gmail.com

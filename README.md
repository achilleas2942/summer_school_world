# AEROTRAIN SUMMER SCHOOL - Human-Robot Interaction Day: Gazebo World
This work is a part of the AERO-TRAIN Summer School 2024. It was presented during Day 3 - Human-Robot Interaction Day, and it is part of [Tutorial 1](https://github.com/AERO-TRAIN/exercises_summer_school_hri_day/tree/main).

## Repository Description
- This repository is designed to run within a [docker container](https://github.com/achilleas2942/summer_school_world/tree/main/docker) as a part of the [Tutorial 1](https://github.com/AERO-TRAIN/exercises_summer_school_hri_day/tree/main), howerver it can also run locally.
- The repository includes:
  - The Gazebo world (figure).
  - The low level controller (velocity controller).
  - UDP tunnels to transmit the ROS messages (_odometry_ and _command_ messages) to any machine without communication constrains. In order for the UDP tunnels to work correctly, you should specify:
    - The correct destination IP and port in the [odometry_client.py](https://github.com/achilleas2942/summer_school_world/blob/main/src/scripts/odometry_client.py) file
    - The correct reading IP and port in the [command_server.py](https://github.com/achilleas2942/summer_school_world/blob/main/src/scripts/command_server.py) file
  - A downlink (_command_ messages) delay calculator, that publish the delay in the _/downlink_delay_ topic.

### Dependencies
- [gazebo_plugins](https://wiki.ros.org/gazebo_plugins)
  ```
  sudo apt-get install -y ros-noetic-gazebo-plugins
  ```
- [gazebo_ros_pkgs](https://wiki.ros.org/gazebo_ros_pkgs)
  ```
  sudo apt-get install -y ros-noetic-gazebo-ros-pkgs
  ```
- [xarco](https://wiki.ros.org/xacro)
  ```
  sudo apt-get install -y ros-noetic-xacro
  ```
- [octomap](https://wiki.ros.org/octomap)
  ```
  sudo apt-get install -y ros-noetic-octomap ros-noetic-octomap-msgs ros-noetic-octomap-ros
  ```
- [rqt_gui](https://wiki.ros.org/rqt_gui)
  ```
  sudo apt-get install -y rros-noetic-rqt-gui ros-noetic-rqt-gui-py
  ```
- [mavros](https://github.com/mavlink/mavros)
  ```
  sudo apt-get install -y ros-noetic-mavros ros-noetic-mavros-msgs
  ```
- [mav_comm](https://github.com/ethz-asl/mav_comm)
  ```
  git clone https://github.com/ethz-asl/mav_comm.git
  ```
- [rotors_simulator](https://github.com/ethz-asl/rotors_simulator)
  ```
  git clone https://github.com/ethz-asl/rotors_simulator.git
  ```
  
### Clone the repository
  ```
  git clone https://github.com/achilleas2942/summer_school_world.git
  ```
    
### Run the Gazebo world locally
- After building, e.g.
  ```
  catkin build summer_school_world
  ```
  or
  ```
  catkin_make summer_school_world
  ```
- And sourcing your workspace, e.g.
  ```
  source <your_workspace_name>/devel/setup.bash
  ```
- Execute the launch file
  ```
  roslaunch summer_school_world world_map.launch
  ```

## Troubleshooting
Consider opening an Issue if you have [troubles](https://github.com/achilleas2942/summer_school_world/issues) with the exercises of the repo.\
Feel free to use the [Discussions](https://github.com/achilleas2942/summer_school_world/discussions) tab to exchange ideas and ask questions.

# Harvester Arm Robot
This package is written for the harvesting arm robot in my lab. It is written for Ubuntu 18.04 with ROS Melodic.[For Mom!]


## Quickstart (Run if the whole system has been setup before)
Bring Up robot
```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.1.102 [reverse_port:=REVERSE_PORT] kinematics_config:=$(rospack find ur_calibration)/etc/ex-ur5_calibration.yaml

```
#### For Moveit
```
roslaunch ur5_moveit_config moveit_planning_execution.launch 

roslaunch ur5_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5_moveit_config)/launch/moveit.rviz
```

## Installation
### Connection Setup


### UR5
Clone UR driver and ROS driver
```
mkdir -p ws_arm/src
cd ws_arm/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
git clone -b melodic-devel-staging https://github.com/ros-industrial/universal_robot.git
cd ..
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y
catkin_make
source devel/setup.bash
```

#### First Time Only for Downloading calibration file from Robot
Create Folder
```
cd ~/ws_arm/src/Universal_Robots_ROS_Driver/ur_calibration
mkdir etc
```
Download by calling
```
roslaunch ur_calibration calibration_correction.launch robot_ip:={qqq.qqq.qqq.qqq} target_filename:="$(rospack find ur_calibration)/etc/ex-ur5_calibration.yaml"

```


### onrobot SG Gripper
Clone Gripper ROS Driver
```
cd ws_arm/src
git clone https://github.com/Phayuth/onrobot_sg.git
cd ..
catkin_make
```


### IntelRealsense Camera
Follow: https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy


## References
- UR5 Setup
	- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
	- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_robot_driver/doc
	- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md
	- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/301
- onrobot SG
	- https://github.com/Phayuth/onrobot_sg
- Irl task :
	- https://github.com/thinclab/ur3e_irl_project
- UR5 control :
	- https://github.com/Osaka-University-Harada-Laboratory/ur5e_tutorials
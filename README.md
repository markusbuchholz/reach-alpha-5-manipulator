# Reach Alpha 5 Robot Arm

## Introduction

This repository contains files to launch Reach Alpha 5 robot arm in RViz and Gazebo 11. 
Additionally, it contains C++ file to run simple motion in ROS 2 on the path computed by Moveit.

![image](https://github.com/markusbuchholz/reach-alpha-5-manipulator/assets/30973337/aa14d79c-4e1a-420b-8c82-96f5178b78fa)



The repository uses robot setup proposed [here](https://github.com/evan-palmer/alpha).
Additional useful references:
* [Reach Alpha 5 manipulator](https://reachrobotics.com/products/manipulators/reach-alpha/)
* [ROS 2 Humble documentation](https://docs.ros.org/en/humble/index.html)
* [Moveit 2 documentation](https://moveit.picknik.ai/main/index.html)


## Build and Run

#### Lauch robot in RViz

```bash
git clone https://github.com/markusbuchholz/reach-alpha-5-manipulator.git
cd reach-alpha-5-manipulator
colcon build
source install/setup.bash
ros2 launch alpha_bringup planning_alpha5.launch.py
```

#### Run simple motion

In other terminal
```bash
source install/setup.bash
ros2 run alpha_moveit joint_moveit
```

#### You can run also ```joint_state_publisher-gui```

First, install.

```bash
sudo apt install ros-humble-joint-state-publisher-gui
```
and run,
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

#### Lauch robot in Gazebo
```bash
ros2 launch alpha_gazebo gazebo_alpha.launch.py
```

![image](https://github.com/markusbuchholz/reach-alpha-5-manipulator/assets/30973337/44fd0225-a2d1-405f-9ff6-7f7557e0b14c)

In other terminal you can run simulation,
```bash
source install/setup.bash
ros2 run alpha_moveit joint_moveit
```





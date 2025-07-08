# AGV_Pro
ROS2 packages for AGV_Pro

> Software environment for Jetson Orin Nano

```
ubuntu 22.04
ros2 humble
gazebo 11
```

# Installation

Create workspace and clone the repository.

```
git clone https://github.com/elephantrobotics/agv_pro_ros2.git agv_pro_ros2/src
```

Install dependencies

```
cd ~/agv_pro_ros2

rosdep install --from-paths src --ignore-src -r -y
```

Build workspace

```
cd ~/agv_pro_ros2

colcon build
```

Setup the workspace

```
source ~/agv_pro_ros2/install/local_setup.bash
```

```
apt install ros-$ROS_DISTRO-gazebo-ros-pkgs

sudo apt install ros-$ROS_DISTRO-ros2-controllers

sudo apt install ros-humble-teleop-twist-keyboard
```

# Update to new version

```
cd ~/myagv_ros2/src

git pull

cd ..

colcon build
```

# Start

```
ros2 launch agv_pro_gazebo agv_pro_gazebo.launch.py
```

# keyboard Control

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# Synchronous motion

```
ros2 launch agv_pro_bringup agv_pro_bringup.launch.py
ros2 launch agv_pro_gazebo agv_pro_gazebo.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

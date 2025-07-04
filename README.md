# AGV_Pro
ROS2 packages for AGV_Pro

![agvpro_1](https://github.com/user-attachments/assets/a406ae13-b685-48a5-bcb8-6b654809eb43)

![agvpro_2](https://github.com/user-attachments/assets/dd0ed80f-9d7d-4e10-a951-e84a0b311daa)

> Software environment for Jetson Orin Nano

```
ubuntu 22.04
ros2 humble
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

# Update to new version

```
cd ~/agv_pro_ros2/src

git pull

cd ..

colcon build
```
# User manual

[myagvPro_docs](https://github.com/elephantrobotics/myagvPro_docs/tree/main/MYAGV_PRO_EN/6-SDKDevelopment/6.2-ApplicationBaseROS2)

# AGV AutoCharge ROS2 Package

这是一个为ROS2 Humble设计的AGV自动充电系统软件包。

## 功能特性

- 监听充电桩位置更新
- 持续发布可视化标记
- 键盘交互启动导航功能
- 导航成功后启动串口控制
- 支持充电状态检测和控制

## 安装依赖

确保您的系统已安装以下依赖：

```bash
# ROS2 Humble基础包
sudo apt install ros-humble-rclpy
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-std-msgs
sudo apt install ros-humble-nav-msgs
sudo apt install ros-humble-visualization-msgs
sudo apt install ros-humble-nav2-simple-commander

# Python依赖
pip3 install pyserial
```

## 编译安装

```bash
# 进入ROS2工作空间
cd /path/to/your/ros2_ws/src

# 复制软件包到工作空间
cp -r agv_autocharge_ros2 .

# 编译软件包
cd ..
colcon build --packages-select agv_autocharge_ros2

# 加载环境变量
source install/setup.bash
```

## 使用方法

### 启动节点

```bash
# 启动自动充电控制器节点
ros2 run agv_autocharge_ros2 combined_auto_recharger
```

### 节点功能

- 监听 `/charger_position_update` 话题，接收充电桩位置更新
- 发布 `/goal_marker` 话题，在RViz中显示充电桩位置标记
- 发布 `/cmd_vel` 话题，控制机器人运动
- 按键 `q` 启动导航到充电桩
- 导航成功后自动启动串口控制

### 配置文件

充电桩位置配置文件位于：
```
config/charger_position.json
```

文件格式：
```json
{
  "p_x": 1.329015495451769,
  "p_y": 0.31961151635354823,
  "orien_z": 0.4981823289472456,
  "orien_w": 0.8670722963655905
}
```

### 话题接口

#### 订阅话题
- `/charger_position_update` (geometry_msgs/PoseStamped): 充电桩位置更新

#### 发布话题
- `/goal_marker` (visualization_msgs/MarkerArray): 充电桩位置可视化标记
- `/cmd_vel` (geometry_msgs/Twist): 机器人运动控制
- `/chassis_security` (std_msgs/Int8): 底盘安全控制

### 串口配置

默认串口配置：
- 端口: `/dev/ttyCH341USB0`
- 波特率: 9600
- 超时: 1秒

可以根据需要修改代码中的串口参数。

## 操作说明

1. 启动节点后，系统会自动加载充电桩位置配置
2. 系统会定期发布充电桩标记到RViz进行可视化
3. 按下键盘上的 `q` 键启动导航到充电桩
4. 导航成功后，系统会自动启动串口控制功能
5. 串口控制会根据接收到的数据控制机器人运动
6. 按 `Ctrl+C` 退出程序

## 故障排除

### 常见问题

1. **串口无法打开**
   - 检查串口设备是否连接
   - 确认串口权限设置
   - 验证串口设备名称

2. **导航失败**
   - 确认Nav2导航系统正常运行
   - 检查充电桩位置配置是否正确
   - 验证地图和定位系统状态

3. **RViz中看不到标记**
   - 确认RViz已订阅 `/goal_marker` 话题
   - 检查MarkerArray显示设置
   - 验证坐标系设置是否为'map'

## 许可证

MIT License

## 维护者

请联系维护者获取技术支持。

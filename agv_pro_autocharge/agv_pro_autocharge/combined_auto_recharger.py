#!/usr/bin/env python3 
# coding=utf-8

"""
合并的自动充电控制器 - 结合位置管理、可视化和导航功能
- 监听充电桩位置更新
- 持续发布可视化标记
- 按键'q'启动导航功能
- 导航成功后启动串口控制
"""

# 引用ros库
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

# 用到的变量定义
from std_msgs.msg import Bool 
from std_msgs.msg import Int8 
from std_msgs.msg import UInt8
from std_msgs.msg import Float32

# 用于记录充电桩位置、发布导航点
from geometry_msgs.msg import PoseStamped, Twist

# rviz可视化相关
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# 里程计话题相关
from nav_msgs.msg import Odometry

# 键盘控制相关
import sys
import select
import termios
import tty

# 延迟相关
import time
import threading

# 读写充电桩位置文件
import json
import math
import yaml
import os

# 导入串口解析模块
from .serial_can_parser import SerialCANParser

# 存放充电桩位置的文件位置 - 参考原始auto_recharger.py的路径设置方式
def find_config_files():
    """查找配置文件路径"""
    # 首先尝试几个可能的位置
    possible_paths = [
        # 开发环境路径
        '/home/elephant/agv_pro_ros2/src/agv_pro_autocharge/config',
        # 你的工作空间路径
        '/home/elephant/agv_pro_ros2/src/agv_pro_autocharge/config',
        # 当前包的相对路径
        os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config'),
        # 安装路径
        '/home/elephant/agv_pro_ros2/install/agv_pro_autocharge/share/agv_pro_autocharge/config'
    ]
    
    for config_dir in possible_paths:
        yaml_path = os.path.join(config_dir, 'nav_goal_params.yaml')
        json_path = os.path.join(config_dir, 'charger_position.json')
        
        print(f"Checking config directory: {config_dir}")
        if os.path.exists(yaml_path) and os.path.exists(json_path):
            print(f"Found config files in: {config_dir}")
            return yaml_path, json_path
    
    # 如果都找不到，直接报错
    print("ERROR: Could not find config files in any of the following locations:")
    for path in possible_paths:
        print(f"  - {path}")
    print("Please ensure the config files exist in one of these directories.")
    
    # 返回第一个路径作为默认值，但文件可能不存在
    return os.path.join(possible_paths[0], 'nav_goal_params.yaml'), os.path.join(possible_paths[0], 'charger_position.json')

# 获取配置文件路径
yaml_file, json_file = find_config_files()

# print_and_fixRetract相关，用于打印带颜色的信息
RESET = '\033[0m'
RED   = '\033[1;31m'
GREEN = '\033[1;32m'
YELLOW= '\033[1;33m'
BLUE  = '\033[1;34m'
PURPLE= '\033[1;35m'
CYAN  = '\033[1;36m'

# 圆周率
PI = 3.1415926535897

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty
	
settings = None
if os.name != 'nt' and sys.stdin.isatty():
    settings = list(termios.tcgetattr(sys.stdin))

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    else:
        if sys.stdin.isatty():
            tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        if sys.stdin.isatty() and settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def print_and_fixRetract(str):
    global settings
    '''键盘控制会导致回调函数内使用print()出现自动缩进的问题，此函数可以解决该现象'''
    if sys.stdin.isatty() and settings:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(str)

class CombinedAutoRecharger(Node):
    def __init__(self):
        
        # 创建节点
        super().__init__("combined_auto_recharger")

        print_and_fixRetract('Combined Auto Recharger Node Started!')

        # 导航状态标记
        self.navigation_active = False
        
        # 串口控制相关
        self.parser = None
        self.serial_control_active = False
        self.navigation_requested = False  # 添加导航请求标志
        
        # 创建导航器
        self.navigator = BasicNavigator()

        # 加载充电桩位置信息
        self.load_charger_position()
        # 加载导航参数
        self.load_nav_goal_params()

        # 创建发布者
        self.robot_security_off_pub = self.create_publisher(Int8, '/chassis_security', 10) 
        self.Charger_marker_pub = self.create_publisher(MarkerArray, '/goal_marker', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 创建订阅者 - 只订阅充电桩位置更新
        self.Charger_Position_Update_sub = self.create_subscription(
            PoseStamped, "/charger_position_update", 
            self.Position_Update_callback, 10)

        # 创建定时器，定期发布充电桩标记（每2秒发布一次）
        self.marker_timer = self.create_timer(2.0, self.timer_callback)
        
        # 创建导航检查定时器（每0.5秒检查一次导航请求）
        self.navigation_timer = self.create_timer(0.5, self.check_navigation_request)

        # 发布初始充电桩位置标记
        self.update_charger_visualization()

        print_and_fixRetract('Combined auto recharger node initialized successfully!')
        print_and_fixRetract(f'{GREEN}Press "q" to start navigation, Ctrl+C to exit{RESET}')

    def load_nav_goal_params(self):
        """加载导航目标参数（前方距离和角度）"""
        print_and_fixRetract(f"Attempting to load nav goal params from: {yaml_file}")
        print_and_fixRetract(f"File exists? {os.path.exists(yaml_file)}")
        
        try:
            with open(yaml_file, 'r', encoding='utf-8') as f:
                params = yaml.safe_load(f)
            
            print_and_fixRetract(f"Raw params from file: {params}")
            
            self.forward_distance = float(params.get('forward_distance', 1.0))
            self.yaw_offset_deg = float(params.get('yaw_offset_deg', 0.0))
            
            print_and_fixRetract(f"Successfully loaded nav goal params: forward_distance={self.forward_distance}, yaw_offset_deg={self.yaw_offset_deg}")
            
        except FileNotFoundError:
            print_and_fixRetract(f"{RED}Nav goal params file not found: {yaml_file}{RESET}")
            print_and_fixRetract(f"{RED}Please create the configuration file with the required parameters{RESET}")
            # 使用默认值
            self.forward_distance = 1.0
            self.yaw_offset_deg = 0.0
        except Exception as e:
            print_and_fixRetract(f"{RED}Failed to load nav goal params: {e}{RESET}")
            self.forward_distance = 1.0
            self.yaw_offset_deg = 0.0

    def timer_callback(self):
        '''定时器回调函数，定期发布充电桩标记'''
        # 始终发布当前JSON文件中的位置信息
        if hasattr(self, 'json_data') and self.json_data:
            self.Pub_Charger_marker(
                self.json_data['p_x'],
                self.json_data['p_y'], 
                self.json_data['orien_z'],
                self.json_data['orien_w']
            )

    def check_navigation_request(self):
        '''检查是否有导航请求'''
        if self.navigation_requested and not self.navigation_active:
            self.navigation_requested = False
            self.navigation_active = True
            print_and_fixRetract(f"{BLUE}Processing navigation request...{RESET}")
            
            # 在ROS2线程中执行导航
            result = self.execute_navigation_internal()
            
            if result:
                print_and_fixRetract(f"{GREEN}Navigation successful! Starting serial control...{RESET}")
                # 在ROS2线程中启动串口控制
                self.start_serial_control_async()
            else:
                print_and_fixRetract(f"{RED}Navigation failed{RESET}")
                self.navigation_active = False

    def request_navigation(self):
        '''请求开始导航'''
        if not self.navigation_active:
            self.navigation_requested = True
            print_and_fixRetract(f"{BLUE}Navigation request queued...{RESET}")
        else:
            print_and_fixRetract(f"{YELLOW}Navigation already in progress{RESET}")

    def load_charger_position(self):
        '''加载充电桩位置信息'''
        try:
            with open(json_file, 'r', encoding='utf-8') as fp:
                self.json_data = json.load(fp)
            print_and_fixRetract(f"Loaded charger position: x={self.json_data['p_x']:.3f}, y={self.json_data['p_y']:.3f}")
        except FileNotFoundError:
            print_and_fixRetract(f"{RED}Charger position file {json_file} not found{RESET}")
            print_and_fixRetract(f"{RED}Please create the configuration file with default charger position{RESET}")
            # 使用默认位置
            self.json_data = {
                'p_x': 0.0,
                'p_y': 0.0, 
                'orien_z': 0.0,
                'orien_w': 1.0
            }
        except Exception as e:
            print_and_fixRetract(f"Error loading charger position: {e}")
            self.json_data = {
                'p_x': 0.0,
                'p_y': 0.0,
                'orien_z': 0.0, 
                'orien_w': 1.0
            }

    def save_charger_position(self):
        '''保存充电桩位置信息到JSON文件'''
        try:
            with open(json_file, 'w', encoding='utf-8') as fp:
                json.dump(self.json_data, fp, ensure_ascii=False, indent=2)
            print_and_fixRetract(f"{GREEN}Charger position saved to {json_file}{RESET}")
        except Exception as e:
            print_and_fixRetract(f"{RED}Error saving charger position: {e}{RESET}")

    def Pub_Charger_Position(self):
        '''更新充电桩位置信息并保存到JSON文件'''
        # 发布充电桩位置的可视化
        self.Pub_Charger_marker(
            self.json_data['p_x'], 
            self.json_data['p_y'], 
            self.json_data['orien_z'], 
            self.json_data['orien_w'])
        
        # 保存当前充电桩位置到JSON文件
        position_data = {
            'p_x': self.json_data['p_x'],
            'p_y': self.json_data['p_y'],
            'orien_z': self.json_data['orien_z'],
            'orien_w': self.json_data['orien_w']
        }
        
        self.json_data = position_data
        self.save_charger_position()
        print_and_fixRetract(f"Position: x={self.json_data['p_x']:.3f}, y={self.json_data['p_y']:.3f}")

    def Pub_Charger_marker(self, p_x, p_y, o_z, o_w):
        '''发布目标点可视化话题'''
        
        markerArray = MarkerArray()
        
        # 获取当前时间戳
        current_time = self.get_clock().now().to_msg()

        marker_shape = Marker()  # 创建marker对象
        marker_shape.id = 0  # 必须赋值id
        marker_shape.header.frame_id = 'map'  # 以哪一个TF坐标为原点
        marker_shape.header.stamp = current_time  # 添加时间戳
        marker_shape.type = Marker.ARROW  # TEXT_VIEW_FACING #一直面向屏幕的字符格式
        marker_shape.action = Marker.ADD  # 添加marker
        marker_shape.scale.x = 0.5  # marker大小
        marker_shape.scale.y = 0.05  # marker大小
        marker_shape.scale.z = 0.05  # marker大小，对于字符只有z起作用
        marker_shape.pose.position.x = p_x  # 字符位置
        marker_shape.pose.position.y = p_y  # 字符位置
        marker_shape.pose.position.z = 0.1  # msg.position.z #字符位置
        marker_shape.pose.orientation.z = o_z  # 字符位置
        marker_shape.pose.orientation.w = o_w  # 字符位置
        marker_shape.color.r = 1.0  # 字符颜色R(红色)通道
        marker_shape.color.g = 0.0  # 字符颜色G(绿色)通道
        marker_shape.color.b = 0.0  # 字符颜色B(蓝色)通道
        marker_shape.color.a = 1.0  # 字符透明度
        markerArray.markers.append(marker_shape)  # 添加元素进数组
        
        marker_string = Marker()  # 创建marker对象
        marker_string.id = 1  # 必须赋值id
        marker_string.header.frame_id = 'map'  # 以哪一个TF坐标为原点
        marker_string.header.stamp = current_time  # 添加时间戳
        marker_string.type = Marker.TEXT_VIEW_FACING  # 一直面向屏幕的字符格式
        marker_string.action = Marker.ADD  # 添加marker
        marker_string.scale.x = 0.5  # marker大小
        marker_string.scale.y = 0.5  # marker大小
        marker_string.scale.z = 0.5  # marker大小，对于字符只有z起作用
        marker_string.color.a = 1.0  # 字符透明度
        marker_string.color.r = 1.0  # 字符颜色R(红色)通道
        marker_string.color.g = 0.0  # 字符颜色G(绿色)通道
        marker_string.color.b = 0.0  # 字符颜色B(蓝色)通道
        marker_string.pose.position.x = p_x  # 字符位置
        marker_string.pose.position.y = p_y  # 字符位置
        marker_string.pose.position.z = 0.1  # msg.position.z #字符位置
        marker_string.pose.orientation.z = o_z  # 字符位置
        marker_string.pose.orientation.w = o_w  # 字符位置
        marker_string.text = 'Charger'  # 字符内容
        markerArray.markers.append(marker_string)  # 添加元素进数组
        self.Charger_marker_pub.publish(markerArray)  # 发布markerArray，rviz订阅并进行可视化

    def Position_Update_callback(self, topic):
        '''更新json文件中的充电桩位置'''
        position_dic = {'p_x': 0, 'p_y': 0, 'orien_z': 0, 'orien_w': 0}
        position_dic['p_x'] = topic.pose.position.x
        position_dic['p_y'] = topic.pose.position.y
        position_dic['orien_z'] = topic.pose.orientation.z
        position_dic['orien_w'] = topic.pose.orientation.w

        # 保存最新的充电桩位置到json文件
        self.json_data = position_dic
        self.save_charger_position()
        print_and_fixRetract("New charging pile position saved.")
        
        # 位置更新后立即发布一次新的标记，然后继续定时发布
        self.update_charger_visualization()
        print_and_fixRetract(f"{GREEN}Charger position updated and will be published continuously{RESET}")

    def update_charger_visualization(self):
        '''更新充电桩可视化标记'''
        if hasattr(self, 'json_data'):
            self.Pub_Charger_marker(
                self.json_data['p_x'],
                self.json_data['p_y'], 
                self.json_data['orien_z'],
                self.json_data['orien_w']
            )

    def execute_navigation(self):
        """外部调用的导航接口"""
        self.request_navigation()
        return True  # 返回True表示请求已提交

    def execute_navigation_internal(self):
        """内部执行导航任务"""
        print_and_fixRetract(f"{BLUE}Starting navigation...{RESET}")
        # 从JSON文件读取充电桩位置
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                charger_data = json.load(f)
            px = charger_data['p_x']
            py = charger_data['p_y']
            # 充电桩姿态四元数转欧拉角
            orien_z = charger_data['orien_z']
            orien_w = charger_data['orien_w']
            yaw = 2 * math.atan2(orien_z, orien_w)  # 只考虑z/w分量
        except Exception as e:
            print_and_fixRetract(f"{RED}Failed to read charger position file: {e}{RESET}")
            self.navigation_active = False
            return False

        # 计算目标点位置
        x_offset = self.forward_distance * math.cos(yaw)
        y_offset = self.forward_distance * math.sin(yaw)
        goal_x = px + x_offset
        goal_y = py + y_offset

        # 计算目标点姿态（z轴顺时针yaw_offset_deg）
        goal_yaw = yaw - math.radians(self.yaw_offset_deg)
        goal_qz = math.sin(goal_yaw / 2)
        goal_qw = math.cos(goal_yaw / 2)

        print_and_fixRetract(f"Nav goal: x={goal_x:.3f}, y={goal_y:.3f}, yaw={math.degrees(goal_yaw):.1f}°")
        goal_pose = self.create_pose(goal_x, goal_y, goal_qz, goal_qw)

        # 执行导航
        print_and_fixRetract(f"{BLUE}Executing navigation...{RESET}")
        result1 = self.nav_through_pose([goal_pose], verbose=False)
        print_and_fixRetract(f"Navigation result: {result1}")
        self.navigation_active = False
        return result1

    def create_pose(self, x, y, z, w):
        """创建单个目标点的位姿信息"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def nav_through_pose(self, goal_poses, verbose: bool = False) -> bool:
        """执行多点导航任务"""
        # 开始执行多点导航任务
        self.navigator.goThroughPoses(goal_poses)

        # 等待导航任务完成，监控导航状态
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and verbose:
                remaining = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print_and_fixRetract(f"预计到达时间: {remaining:.0f} 秒")

        # 根据导航结果返回相应状态
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print_and_fixRetract(f'{GREEN}Navigation successful!{RESET}')
            return True
        elif result == TaskResult.CANCELED:
            print_and_fixRetract(f'{YELLOW}Navigation canceled!{RESET}')
        elif result == TaskResult.FAILED:
            print_and_fixRetract(f'{RED}Navigation failed!{RESET}')
        else:
            print_and_fixRetract(f'{RED}Invalid navigation result!{RESET}')
        return False

    def start_serial_control_async(self):
        """异步启动串口控制功能"""
        def serial_control_thread():
            self.start_serial_control()
        
        # 在新线程中启动串口控制，避免阻塞ROS2主线程
        serial_thread = threading.Thread(target=serial_control_thread, daemon=True)
        serial_thread.start()

    def start_serial_control(self):
        """启动串口控制功能"""
        print_and_fixRetract(f"{BLUE}Starting serial control...{RESET}")
        try:
            self.parser = SerialCANParser('/dev/agvpro_ec130', 9600, 1)
            self.parser.open_serial()  # 打开usb串口
            
            # 发送AT 命令从透传模式进入AT指令模式
            self.parser.send_at_commands(["AT+CG", "AT+AT"])
            
            self.serial_control_active = True
            print_and_fixRetract(f'{GREEN}Serial control started successfully{RESET}')
            
            while self.serial_control_active:
                # 开始读取数据
                x_speed, z_speed, which_mode, infrared_bits = self.parser.read_serial_data()
                
                if infrared_bits[7] == 0:  # 无障碍物
                    if which_mode == 0x01:  # 正常模式
                        # 直接发布ROS2 Twist消息
                        twist_msg = Twist()
                        twist_msg.linear.x = float(x_speed)
                        twist_msg.linear.y = 0.0
                        twist_msg.angular.z = float(z_speed)
                        self.cmd_vel_pub.publish(twist_msg)
                        print_and_fixRetract(f'Normal mode - Speed: x={x_speed}, z={z_speed}')
                        
                    elif which_mode == 0xBB:  # 测压区
                        # 停止运动
                        stop_msg = Twist()
                        self.cmd_vel_pub.publish(stop_msg)
                        print_and_fixRetract(f'{YELLOW}Pressure zone - Stop movement{RESET}')
                        
                    elif which_mode == 0xAA:  # 充电区
                        # 停止运动
                        stop_msg = Twist()
                        self.cmd_vel_pub.publish(stop_msg)
                        print_and_fixRetract(f'{GREEN}Charging zone - Stop movement{RESET}')
                        break  # 充电完成后退出
                        
                    elif which_mode == 0xCF:  # 急停模式
                        emergency_stop_msg = Twist()  # 所有速度都为0
                        self.cmd_vel_pub.publish(emergency_stop_msg)
                        print_and_fixRetract(f'{RED}Emergency stop mode - Immediate stop{RESET}')
                        break
                        
                else:  # 检测到障碍物
                    obstacle_stop_msg = Twist()  # 所有速度都为0
                    self.cmd_vel_pub.publish(obstacle_stop_msg)
                    print_and_fixRetract(f'{RED}Obstacle detected - Stop movement{RESET}')
                    break
                    
        except KeyboardInterrupt:
            print_and_fixRetract("Serial control interrupted by user")
            # 发布停止消息
            emergency_stop = Twist()
            self.cmd_vel_pub.publish(emergency_stop)
        except Exception as e:
            print_and_fixRetract(f"{RED}Serial control error: {e}{RESET}")
            # 发布停止消息
            emergency_stop = Twist()
            self.cmd_vel_pub.publish(emergency_stop)
        finally:
            if self.parser:
                self.parser.close_serial()
            print_and_fixRetract("Serial control stopped")

    def stop_serial_control(self):
        """停止串口控制功能"""
        self.serial_control_active = False

    def get_charger_info(self):
        '''获取充电桩位置信息'''
        if hasattr(self, 'json_data'):
            return {
                'position': {
                    'x': self.json_data['p_x'],
                    'y': self.json_data['p_y']
                },
                'orientation': {
                    'z': self.json_data['orien_z'],
                    'w': self.json_data['orien_w']
                }
            }
        return None


def main(args=None):
    '''主函数'''
    rclpy.init(args=args)
    
    combined_recharger = None
    try:
        combined_recharger = CombinedAutoRecharger()
        
        print_and_fixRetract("Combined auto recharger node is running...")
        print_and_fixRetract("Node functions:")
        print_and_fixRetract("- Listening for charger position updates on /charger_position_update")
        print_and_fixRetract("- Publishing visualization markers on /goal_marker every 2 seconds")
        print_and_fixRetract("- Press 'q' to start navigation to charger position")
        print_and_fixRetract("- Navigation success will trigger serial control")
        
        # 启动ROS2事件循环线程
        def ros2_spin():
            try:
                rclpy.spin(combined_recharger)
            except Exception as spin_error:
                print_and_fixRetract(f"ROS2 spin error: {spin_error}")

        ros2_thread = threading.Thread(target=ros2_spin, daemon=True)
        ros2_thread.start()
        
        # 键盘监听循环
        print_and_fixRetract(f"{GREEN}Press 'q' to start navigation, Ctrl+C to exit{RESET}")
        print_and_fixRetract("Waiting for keyboard input...")
        
        while True:
            try:
                key = get_key(settings)
                if key:
                    print_and_fixRetract(f"Key pressed: {repr(key)}")  # 调试信息
                    if key.lower() == 'q':
                        print_and_fixRetract(f"{BLUE}Navigation command received!{RESET}")
                        
                        # 请求导航任务（异步执行）
                        combined_recharger.execute_navigation()
                    
                    elif key == '\x03':  # Ctrl+C
                        break
                
                time.sleep(0.1)  # 避免过度占用CPU
            except KeyboardInterrupt:
                break
            except Exception as e:
                print_and_fixRetract(f"Keyboard input error: {e}")
                time.sleep(0.5)
            
    except KeyboardInterrupt:
        print_and_fixRetract("\nShutting down combined auto recharger node...")
    finally:
        if combined_recharger:
            combined_recharger.stop_serial_control()
            combined_recharger.destroy_node()
        rclpy.shutdown()
        print_and_fixRetract("Program exited safely")

if __name__ == '__main__':
    main()


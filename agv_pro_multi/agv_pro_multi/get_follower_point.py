#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import geometry_msgs.msg
from std_msgs.msg import Float32,Bool,Int16,UInt16

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
from agv_pro_msgs.msg import FollowerPoint

# 持续跟随的队形槽位，领航车 base 系下的 (x, y) 系数，乘以 dist 得到偏移；
# Formation slots for live following as (x, y) factors in the leader base frame, scaled by dist.
FORMATION_OFFSETS = {
    'column': {'robot2': (-1.0, 0.0), 'robot3': (-2.0, 0.0)},
    'row': {'robot2': (0.0, -1.0), 'robot3': (0.0, 1.0)},
    'convoy': {'robot2': (-1.0, -1.0), 'robot3': (-1.0, 1.0)},
}

class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.declare_parameter('robot_id', 'robot2')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.declare_parameter('leader', 'robot1')
        self.leader = self.get_parameter('leader').get_parameter_value().string_value

        if self.robot_id == 'robot2':
            self.pub_topic = f"/{self.robot_id}/goal_pose"
        elif self.robot_id == 'robot3':
            self.pub_topic = f"/{self.robot_id}/goal_pose"
        else:
            self.get_logger().error(f"Unsupported robot_id: {self.robot_id}")
            raise ValueError(f"Unsupported robot_id: {self.robot_id}")
        self.pub_robot_pose = self.create_publisher(PoseStamped, self.pub_topic, 1)
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"

        # Initialize previous values to track changes
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0
        self.prev_w = 0.0
        self.last_goal_time = self.get_clock().now()
        from rclpy.qos import QoSProfile, DurabilityPolicy
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Subscribe to current position
        self.odom_sub = self.create_subscription(PoseStamped,f'/{self.leader}/pose',self.odom_callback,latched_qos)
        ## Publish new point
        self.pub_back = self.create_publisher(PoseStamped, self.pub_topic, 1)
        # Subscribe to target point
        self.goal_sub = self.create_subscription(PoseStamped,f'/{self.leader}/goal_pose',self.goal_pose_callback,1)
        # Subscribe to target point
        self.goal_sub2 = self.create_subscription(PoseStamped,f'/{self.robot_id}/pose',self.odom2_callback,latched_qos)
        # Subscribe to queue/dist
        self.queue_subscriber = self.create_subscription(FollowerPoint, '/queue_type', self.queue_callback, latched_qos)

        self.current_pose = None
        self.current_pose2 = None
        self.goal_pose_text = None
        self.back_pose_published = False

        self.queue_flag = False
        self.last_log_time = self.get_clock().now()

        self.timer = self.create_timer(0.65, self.timer_callback)

    def goal_pose_callback(self, msg: PoseStamped):
        self.goal_pose_text = msg
        self.back_pose_published = False
        self.get_logger().info(f"Saved new target point: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}")

    def print_log(self, log_type, text):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < 2000000000:
            return
        self.last_log_time = now
        if log_type == "warn":
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def odom_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose
        if self.goal_pose_text is None:
            return
        self.print_log("info", f"\nLatest target point: x={self.goal_pose_text.pose.position.x:.3f}, y={self.goal_pose_text.pose.position.y:.3f}\n{self.leader} real-time position: x={self.current_pose.position.x:.3f}, y={self.current_pose.position.y:.3f}")

    def odom2_callback(self, msg: PoseStamped):
        self.current_pose2 = msg.pose
        if self.goal_pose_text is None:
            return
        self.print_log("info", f"\n{self.robot_id} real-time position: x={self.current_pose2.position.x:.3f}, y={self.current_pose2.position.y:.3f}")


    def timer_callback(self):
        if self.current_pose is None:
            self.print_log("warn", f"Waiting for /{self.leader}/pose data...")
            return
        if self.goal_pose_text is None:
            self.get_point()
            return

        dx = self.current_pose.position.x - self.goal_pose_text.pose.position.x
        dy = self.current_pose.position.y - self.goal_pose_text.pose.position.y
        dr = math.sqrt(dx**2 + dy**2)
        self.print_log("info", f"\nStraight-line distance between vehicles: dr={dr}")
        if dr <= 0.50  and  not self.back_pose_published:
            self.print_log("warn", f"Publishing new target point for {self.robot_id}........................")
            new_pose = self.compute_back_pose(self.goal_pose_text)
            if new_pose is None:
                return
            self.pub_back.publish(new_pose)
            if self.current_pose2 is None:
                return
            if abs(new_pose.pose.position.x - self.current_pose2.position.x) < 0.2 or abs(new_pose.pose.position.y - self.current_pose2.position.y) < 0.2:
                self.print_log("warn", f"{self.robot_id} reached target point........................")
                self.back_pose_published = True
        else:
            self.get_point()
    def queue_callback(self, msg):
        self.queue = msg.queue
        self.dist = msg.dist
        self.queue_flag = True

    def compute_back_pose(self, pose_msg: PoseStamped):
        """Calculate position x meters behind the target point"""
        if self.queue_flag == False:
            self.print_log("warn", "Waiting for self.dist  self.queue data...")
            return
        q = pose_msg.pose.orientation
        yaw = self.quaternion_to_yaw(q)
        distance  = self.dist

        if self.queue == "column":
            if self.robot_id == 'robot2':
                new_x = pose_msg.pose.position.x - distance * math.cos(yaw)
                new_y = pose_msg.pose.position.y - distance * math.sin(yaw)
            else:
                new_x = pose_msg.pose.position.x - distance * 2 * math.cos(yaw)
                new_y = pose_msg.pose.position.y - distance * 2 * math.sin(yaw)

        elif self.queue == "row":
            # For row formation, calculate side position
            if self.robot_id == 'robot2':
                # robot2: left side (yaw - π/2)
                new_x = pose_msg.pose.position.x + distance * math.cos(yaw - math.pi/2)
                new_y = pose_msg.pose.position.y + distance * math.sin(yaw - math.pi/2)
            else:
                # robot3: right side (yaw + π/2)
                new_x = pose_msg.pose.position.x + distance * math.cos(yaw + math.pi/2)
                new_y = pose_msg.pose.position.y + distance * math.sin(yaw + math.pi/2)
                
        else:
            angle_offset = math.pi / 3
            if self.robot_id == 'robot2':
            # robot2: 后右下角60度位置（yaw + π + angle_offset）
                new_x = pose_msg.pose.position.x + distance *1.5 * math.cos(yaw  +  math.pi + angle_offset)
                new_y = pose_msg.pose.position.y + distance *1.5 * math.sin(yaw  +  math.pi + angle_offset)
            else:
                # robot3: 后左下角60度位置（yaw + math.pi - angle_offset）
                new_x = pose_msg.pose.position.x + distance * math.cos(yaw + math.pi - angle_offset)
                new_y = pose_msg.pose.position.y + distance * math.sin(yaw + math.pi - angle_offset)

        # Create new pose message
        new_pose = PoseStamped()
        new_pose.header = pose_msg.header
        new_pose.header.stamp = rclpy.time.Time().to_msg()
        new_pose.pose.position.x = new_x
        new_pose.pose.position.y = new_y
        new_pose.pose.position.z = pose_msg.pose.position.z
        new_pose.pose.orientation = pose_msg.pose.orientation

        # self.get_logger().info(f"\n{self.robot_id} latest target point: x={new_pose}\n")

        return new_pose

    @staticmethod
    def quaternion_to_yaw(q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


    def compute_live_pose(self, pose_msg):
        """Calculate the formation slot from the leader real-time pose"""
        if self.queue_flag == False:
            self.print_log("warn", "Waiting for self.dist  self.queue data...")
            return None
        table = FORMATION_OFFSETS.get(self.queue, FORMATION_OFFSETS["convoy"])
        offset_x, offset_y = table[self.robot_id]
        offset_x *= self.dist
        offset_y *= self.dist
        yaw = self.quaternion_to_yaw(pose_msg.orientation)

        new_pose = PoseStamped()
        new_pose.header.frame_id = "map"
        new_pose.header.stamp = rclpy.time.Time().to_msg()
        new_pose.pose.position.x = pose_msg.position.x + offset_x * math.cos(yaw) - offset_y * math.sin(yaw)
        new_pose.pose.position.y = pose_msg.position.y + offset_x * math.sin(yaw) + offset_y * math.cos(yaw)
        new_pose.pose.position.z = pose_msg.position.z
        new_pose.pose.orientation = pose_msg.orientation
        return new_pose

    def get_point(self):
        if self.current_pose is None:
            self.print_log("warn", f"Waiting for /{self.leader}/pose data...")
            return
        slot = self.compute_live_pose(self.current_pose)
        if slot is None:
            return

        current_x = slot.pose.position.x
        current_y = slot.pose.position.y
        current_z = slot.pose.orientation.z
        current_w = slot.pose.orientation.w

        move_dist = math.hypot(current_x - self.prev_x, current_y - self.prev_y)
        current_yaw = 2.0 * math.atan2(current_z, current_w)
        prev_yaw = 2.0 * math.atan2(self.prev_z, self.prev_w)
        yaw_diff = abs(math.atan2(math.sin(current_yaw - prev_yaw), math.cos(current_yaw - prev_yaw)))

        if move_dist > 0.10 or yaw_diff > 0.08:
            now = self.get_clock().now()
            if (now - self.last_goal_time).nanoseconds < 800000000:
                return
            self.last_goal_time = now

            self.goal_pose.pose.position.x = current_x
            self.goal_pose.pose.position.y = current_y
            self.goal_pose.pose.orientation.z = current_z
            self.goal_pose.pose.orientation.w = current_w
            self.goal_pose.header.stamp = rclpy.time.Time().to_msg()
            self.pub_robot_pose.publish(self.goal_pose)

            self.prev_z = current_z
            self.prev_w = current_w
            self.prev_x = current_x
            self.prev_y = current_y

def main(args=None):
    rclpy.init(args=args)
    node = TfListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

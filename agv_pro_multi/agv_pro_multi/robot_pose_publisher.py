#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
import tf2_ros


class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__('robot_pose_publisher')

        self.declare_parameter('robot_id', 'robot1')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.declare_parameter('global_frame', 'map')
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.declare_parameter('base_frame', f'{self.robot_id}/base_link')
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.declare_parameter('rate', 10.0)
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.declare_parameter('warn_interval', 5.0)
        self.warn_interval = self.get_parameter('warn_interval').get_parameter_value().double_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_pose = self.create_publisher(PoseStamped, f'/{self.robot_id}/pose', latched_qos)

        self.last_warn = self.get_clock().now()
        self.timer = self.create_timer(1.0 / max(self.rate, 0.1), self.timer_callback)
        self.get_logger().info(
            f'Publishing {self.global_frame} -> {self.base_frame} '
            f'to /{self.robot_id}/pose at {self.rate:.1f} Hz')

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException as error:
            now = self.get_clock().now()
            if (now - self.last_warn).nanoseconds >= self.warn_interval * 1e9:
                self.last_warn = now
                self.get_logger().warn(f'Could not transform: {error}')
            return

        msg = PoseStamped()
        msg.header.stamp = transform.header.stamp
        msg.header.frame_id = self.global_frame
        msg.pose.position.x = transform.transform.translation.x
        msg.pose.position.y = transform.transform.translation.y
        msg.pose.position.z = transform.transform.translation.z
        msg.pose.orientation = transform.transform.rotation
        self.pub_pose.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

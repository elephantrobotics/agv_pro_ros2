#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


DYNAMIC_QOS = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST)
STATIC_QOS = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class TfAggregator(Node):
    def __init__(self):
        super().__init__('tf_aggregator')

        self.robots = self.declare_parameter(
            'robots', ['robot1', 'robot2', 'robot3']).value
        self.report_interval = self.declare_parameter('report_interval', 10.0).value

        self.pub_tf = self.create_publisher(TFMessage, '/tf', DYNAMIC_QOS)
        self.pub_tf_static = self.create_publisher(TFMessage, '/tf_static', STATIC_QOS)

        self.counts = {}
        for robot_name in self.robots:
            self.counts[robot_name] = [0, 0]
            self.create_subscription(
                TFMessage, '/{}/tf'.format(robot_name),
                self.make_callback(robot_name, 0, self.pub_tf), DYNAMIC_QOS)
            self.create_subscription(
                TFMessage, '/{}/tf_static'.format(robot_name),
                self.make_callback(robot_name, 1, self.pub_tf_static), STATIC_QOS)
            self.get_logger().info(
                'Aggregating /{0}/tf and /{0}/tf_static'.format(robot_name))

        if self.report_interval > 0.0:
            self.create_timer(self.report_interval, self.report)

    def make_callback(self, robot_name, index, publisher):
        def callback(msg):
            if not msg.transforms:
                return
            self.counts[robot_name][index] += len(msg.transforms)
            publisher.publish(msg)
        return callback

    def report(self):
        parts = ['{} {}/{}'.format(name, count[0], count[1])
                 for name, count in self.counts.items()]
        self.get_logger().info('Forwarded (dynamic/static): ' + '  '.join(parts))


def main():
    rclpy.init()
    node = TfAggregator()
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

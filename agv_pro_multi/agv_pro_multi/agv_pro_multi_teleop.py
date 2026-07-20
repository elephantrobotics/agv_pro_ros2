#!/usr/bin/env python3
import sys
import select

import geometry_msgs.msg
import rclpy
from rclpy.node import Node


msg = """
teleop_twist_keyboard multi
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


class MultiTeleopKeyboard(Node):
    def __init__(self, name):
        super().__init__(name)

        self.robot_names = self.declare_parameter(
            'robot_names', ['robot1', 'robot2', 'robot3']).value
        self.stamped = self.declare_parameter('stamped', False).value
        self.frame_id = self.declare_parameter('frame_id', '').value
        self.speed = self.declare_parameter('speed', 0.25).value
        self.turn = self.declare_parameter('turn', 0.5).value
        self.speed_limit = self.declare_parameter('speed_limit', 1.5).value
        self.turn_limit = self.declare_parameter('turn_limit', 1.5).value

        if self.stamped:
            self.TwistMsg = geometry_msgs.msg.TwistStamped
        else:
            self.TwistMsg = geometry_msgs.msg.Twist

        self.pubs = []
        for robot_name in self.robot_names:
            topic = '/{}/cmd_vel'.format(robot_name)
            self.pubs.append(self.create_publisher(self.TwistMsg, topic, 10))
            self.get_logger().info('publishing to {}'.format(topic))

        self.settings = self.get_settings()

    def get_settings(self):
        import termios
        return termios.tcgetattr(sys.stdin)

    def restore_settings(self, old_settings):
        import termios
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def getKey(self):
        import tty
        import termios
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

    def create_twist_msg(self, x, y, z, th):
        twist_msg = self.TwistMsg()
        if self.stamped:
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.frame_id
            twist = twist_msg.twist
        else:
            twist = twist_msg

        twist.linear.x = x * self.speed
        twist.linear.y = y * self.speed
        twist.linear.z = z * self.speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th * self.turn
        return twist_msg

    def create_stop_msg(self):
        return self.create_twist_msg(0, 0, 0, 0)

    def publish_all(self, twist_msg):
        for pub in self.pubs:
            pub.publish(twist_msg)


def main():
    rclpy.init()
    teleop = MultiTeleopKeyboard('agv_pro_multi_teleop')

    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0

    try:
        print(msg)
        print('controlling: {}'.format(teleop.robot_names))
        print(teleop.vels(teleop.speed, teleop.turn))

        while True:
            key = teleop.getKey()

            if key in moveBindings:
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings:
                teleop.speed = min(teleop.speed_limit, teleop.speed * speedBindings[key][0])
                teleop.turn = min(teleop.turn_limit, teleop.turn * speedBindings[key][1])

                if teleop.speed == teleop.speed_limit:
                    print("Linear speed limit reached!")
                if teleop.turn == teleop.turn_limit:
                    print("Angular speed limit reached!")

                print(teleop.vels(teleop.speed, teleop.turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            twist_msg = teleop.create_twist_msg(x, y, z, th)
            teleop.publish_all(twist_msg)

    except Exception as e:
        print(e)

    finally:
        teleop.publish_all(teleop.create_stop_msg())
        teleop.restore_settings(teleop.settings)
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

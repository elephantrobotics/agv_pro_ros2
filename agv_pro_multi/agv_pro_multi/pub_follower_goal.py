#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped,Twist,PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.node import Node
from std_msgs.msg import UInt16,Bool,Int16
import time
from visualization_msgs.msg import MarkerArray
from agv_pro_msgs.msg import FollowerPoint

# from transforms3d.euler import euler2quat, quat2euler
import math
class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._client = ActionClient(self, NavigateThroughPoses, '/robot1/navigate_through_poses')
        self.get_waypoints = self.create_subscription(MarkerArray,"/waypoints",self.get_waypointsPoseCallBack,1)
        from rclpy.qos import QoSProfile, DurabilityPolicy
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.queue_publisher = self.create_publisher(FollowerPoint, '/queue_type', latched_qos)
        self.goal_pose = PoseStamped()
        self.orinal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.orinal_pose.header.frame_id = "map"
        
        
        self.prev_queue = None
        self.prev_dist = None
        self.declare_parameter("queue_name", "convoy")
        self.queue = self.get_parameter("queue_name").get_parameter_value().string_value #row-行,convoy-护卫队,column-列
        
        self.declare_parameter("dist", 1.5)
        self.dist = self.get_parameter("dist").get_parameter_value().double_value 
        if self.dist < 1.5:
            self.dist = 1.5
        self.timer = self.create_timer(0.5, self.on_timer)
        
    def get_waypointsPoseCallBack(self,msg):
        #print("get the pose: ")
        #print(len(msg.markers))
        #print(msg.markers[len(msg.markers)-2].pose)
        self.goal_pose.pose.position.x = msg.markers[len(msg.markers)-2].pose.position.x
        self.goal_pose.pose.position.y = msg.markers[len(msg.markers)-2].pose.position.y  
        self.goal_pose.pose.orientation.x = msg.markers[len(msg.markers)-2].pose.orientation.x
        self.goal_pose.pose.orientation.y = msg.markers[len(msg.markers)-2].pose.orientation.y
        self.goal_pose.pose.orientation.z = msg.markers[len(msg.markers)-2].pose.orientation.z
        self.goal_pose.pose.orientation.w = msg.markers[len(msg.markers)-2].pose.orientation.w
        self.send_goal()

    def on_timer(self):

        current_queue = self.get_parameter("queue_name").get_parameter_value().string_value
        current_dist = self.get_parameter("dist").get_parameter_value().double_value
        if current_dist < 1.0:
            current_dist = 1.0
        
        # 检查参数是否发生变化
        if current_queue != self.prev_queue or current_dist != self.prev_dist:
            self.update_queue(current_queue, current_dist)

            self.prev_queue = current_queue
            self.prev_dist = current_dist


    def update_queue(self, queue_type, dist):
        self.queue = queue_type if queue_type in ("column", "row", "convoy") else "convoy"
        self.dist = dist
        queue_msg = FollowerPoint()
        queue_msg.dist = self.dist
        queue_msg.queue = self.queue
        self.queue_publisher.publish(queue_msg)
        print(f"queue type: {self.queue}, dist: {self.dist:.2f} m.")

    def send_goal(self):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses.append(self.goal_pose)
        self._client.wait_for_server()
        self._client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        result = future.result()

        if not result.accepted:
            self.get_logger().error('Goal was rejected!')
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        result_msg = result
        #print(result_msg)
        result.get_result_async().add_done_callback(self.result_callback)
        
    def result_callback(self, future):
        result_msg = future.result()
        print("the result_status is ",result_msg.status)
        if result_msg.status==4:
            self.pub_rotate = False
            print("Goal reached.")


        
    def feedback_callback(self,feedback_msg):
        if feedback_msg.feedback.distance_remaining<0.10:
            print("Done.")



def main(args=None):
    rclpy.init(args=args)
    client = NavigationClient()
    rclpy.spin(client)

if __name__ == '__main__':
    main()

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import time
from rclpy.node import Node
#from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class ZielPublisher(Node):

    def __init__(self):
        super().__init__('ziel_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.status_publisher_ = self.create_publisher(String, 'status', 10)
        self.subscription = self.create_subscription(PoseStamped, 'current_pose', self.pose_callback, 10)
        self.current_x = 0.0
        self.current_y = 0.0
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_goal_pose)
        self.i = 0
        self.x = 0.0
        self.y = 0.0
        self.w = 0.0


    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

    def publish_goal_pose(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = self.w
        self.publisher_.publish(goal_pose)
        print ("123")

    def publish_progress(self):
        progress_msg = String()
        progress_msg.data = f'Progress: x={self.current_x:.2f}m, y={self.current_y:.2f}m'
        self.status_publisher_.publish(progress_msg)

def main(args=None):
    print("hahah")
    rclpy.init(args=args)
    ziel_publisher = ZielPublisher()
    # Set your goal coordinates
    goal_x = 0.999
    goal_y = 0.5
    ziel_publisher.x = 0.0
    ziel_publisher.y = 0.0
    ziel_publisher.w = 0.5
    #ziel_publisher.publish_goal_pose()

    ziel_publisher.publish_progress()  # Publish initial progress
    rclpy.spin(ziel_publisher)
    
    #to set the end speed = 0
    ziel_publisher.set_velocity(0.0)
    ziel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


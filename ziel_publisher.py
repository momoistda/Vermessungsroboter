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
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

#create a class, inherit from node:
class ZielPublisher(Node):

    def __init__(self):
        super().__init__('ziel_publisher')  #use init method from class, create a node called ziel_publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) #创建一个发布者，用于发布String类型的消息到名为'cmd_vel'的主题，队列大小为1。
        self.status_publisher_ = self.create_publisher(String, 'status', 10) #create a publisher to show the status
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        #timer_period = 0.5  # seconds for callback
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.i = 0
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.target_x = 0.9
        self.target_y = 0.9
        self.tolerance = 0.05 #tolerance = 5cm
        
        self.timer_started = False
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def timer_callback(self):
        distance_to_goal = ((self.target_x - self.current_x) ** 2 + (self.target_y - self.current_y) ** 2) ** 0.5
        if distance_to_goal > self.tolerance:
            cmd = Twist()
            cmd.linear.x = 0.1 if abs(self.target_x - self.current_x) > self.tolerance else 0.0
            cmd.angular.z = 0.1 if abs(self.target_y - self.current_y) > self.tolerance else 0.0
            self.publisher_.publish(cmd)

            progress_msg = String()
            progress_msg.data = f'Progress: x={self.current_x:.2f}m, y={self.current_y:.2f}m'
            self.status_publisher_.publish(progress_msg)
            self.get_logger().info(progress_msg.data)
        else:
            self.publisher_.publish(Twist())  # Stop the robot
            ziel_msg = String()
            ziel_msg.data = 'Ziel erreicht'
            self.status_publisher_.publish(ziel_msg)
            self.get_logger().info(ziel_msg.data)
            self.timer.cancel()



def main(args=None):
    print ("Hallo")
    rclpy.init(args=args)
    ziel_publisher = ZielPublisher()
    rclpy.spin(ziel_publisher)
    ziel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

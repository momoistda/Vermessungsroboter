import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class GoalReachedChecker(Node):

    def __init__(self):
        super().__init__('goal_reached_checker')
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription_goal = self.create_subscription(
            Point,
            '/goal_point',
            self.goal_callback,
            10)
        self.publisher_reached = self.create_publisher(Bool, '/goal_reached', 10)
        self.current_position = None
        self.goal_position = None
        self.tolerance = 0.5  # 目标点到达的距离容忍范围

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.check_goal_reached()

    def goal_callback(self, msg):
        self.goal_position = (msg.x, msg.y)

    def check_goal_reached(self):
        if self.current_position and self.goal_position:
            distance = self.euclidean_distance(self.current_position, self.goal_position)
            if distance <= self.tolerance:
                self.get_logger().info(f'Goal reached at ({self.goal_position[0]}, {self.goal_position[1]})')
                self.publisher_reached.publish(Bool(data=True))

    def euclidean_distance(self, point1, point2):
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5


def main(args=None):
    rclpy.init(args=args)
    goal_reached_checker = GoalReachedChecker()
    rclpy.spin(goal_reached_checker)
    goal_reached_checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

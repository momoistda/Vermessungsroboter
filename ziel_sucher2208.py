import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class ZielSucher(Node):

    def __init__(self):
        super().__init__('ziel_sucher')
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription_goal_reached = self.create_subscription(
            Bool,
            '/goal_reached',
            self.goal_reached_callback,
            10)
        self.publisher_ = self.create_publisher(Point, '/goal_point', 10)
        self.published = False
        self.waiting_for_goal_reached = False

    def map_callback(self, msg):
        if not self.published or (self.published and not self.waiting_for_goal_reached):
            unknown_points = self.find_unknown_points(msg)
            if unknown_points:
                nearest_point = self.find_nearest_point(unknown_points)
                self.publish_goal_point(nearest_point)
                self.published = True
                self.waiting_for_goal_reached = True

    def goal_reached_callback(self, msg):
        if msg.data:
            self.get_logger().info('Goal has been reached. Searching for the next unknown point.')
            self.waiting_for_goal_reached = False  # 准备发布下一个目标点

    def find_unknown_points(self, map_data):
        unknown_points = []
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y

        for i in range(height):
            for j in range(width):
                index = i * width + j
                if map_data.data[index] == -1:  # -1 表示未知区域
                    x = origin_x + j * resolution
                    y = origin_y + i * resolution
                    unknown_points.append((x, y))
        return unknown_points

    def find_nearest_point(self, unknown_points):
        robot_position = (0, 0)  # 假设当前位置是 (0, 0)；根据需要获取实际的机器人位置
        nearest_point = min(unknown_points, key=lambda point: self.euclidean_distance(point, robot_position))
        return nearest_point

    def euclidean_distance(self, point1, point2):
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

    def publish_goal_point(self, point):
        goal_point_msg = Point()
        goal_point_msg.x = point[0]
        goal_point_msg.y = point[1]
        goal_point_msg.z = 0.0  # 假设在2D平面上，z坐标为0
        self.publisher_.publish(goal_point_msg)
        self.get_logger().info(f'Publishing goal point: ({goal_point_msg.x}, {goal_point_msg.y})')


def main(args=None):
    rclpy.init(args=args)
    ziel_sucher = ZielSucher()
    rclpy.spin(ziel_sucher)
    ziel_sucher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

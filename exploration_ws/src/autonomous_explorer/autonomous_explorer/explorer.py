import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import math

class Explorer(Node):

    def __init__(self):

        super().__init__('explorer')

        self.map_data = None
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.goal_active = False

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)

        self.timer = self.create_timer(6.0, self.explore)

        self.get_logger().info("Improved frontier explorer running")

    def map_callback(self, msg):
        self.map_data = msg

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def explore(self):

        if self.map_data is None:
            return

        if self.goal_active:
            return

        msg = self.map_data

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        data = msg.data

        frontier_points = []

        for y in range(2, height-2):
            for x in range(2, width-2):

                index = y * width + x

                if data[index] != 0:
                    continue

                neighbors = [
                    data[index-1],
                    data[index+1],
                    data[index-width],
                    data[index+width]
                ]

                if -1 in neighbors:
                    frontier_points.append((x,y))

        if len(frontier_points) == 0:
            self.get_logger().info("Exploration complete")
            return

        best_frontier = None
        best_score = float('inf')

        for (x,y) in frontier_points:

            goal_x = origin.x + x * resolution
            goal_y = origin.y + y * resolution

            dist = math.sqrt((goal_x-self.robot_x)**2 +
                             (goal_y-self.robot_y)**2)

            if dist < 0.8:
                continue

            # count unknown cells nearby (information gain)
            info_gain = 0

            for dy in range(-3,4):
                for dx in range(-3,4):

                    nx = x + dx
                    ny = y + dy

                    i = ny * width + nx

                    if i < len(data) and data[i] == -1:
                        info_gain += 1

            score = dist - 0.02 * info_gain

            if score < best_score:
                best_score = score
                best_frontier = (goal_x,goal_y)

        if best_frontier is None:
            return

        goal = PoseStamped()

        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = best_frontier[0]
        goal.pose.position.y = best_frontier[1]
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)

        self.goal_active = True

        self.get_logger().info(
            f"Exploring frontier at {best_frontier}"
        )

        self.create_timer(18.0, self.reset_goal)

    def reset_goal(self):
        self.goal_active = False


def main(args=None):

    rclpy.init(args=args)

    node = Explorer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

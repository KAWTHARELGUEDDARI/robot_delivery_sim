import rclpy # pyright: ignore[reportMissingImports]
from rclpy.node import Node # pyright: ignore[reportMissingImports]
from robot_delivery_sim.robot import Robot # pyright: ignore[reportMissingImports]
from robot_delivery_sim.grid import Grid # pyright: ignore[reportMissingImports]
from robot_delivery_sim.pathfinding import a_star # pyright: ignore[reportMissingImports]
from visualization_msgs.msg import Marker, MarkerArray # pyright: ignore[reportMissingImports]
from geometry_msgs.msg import Point # pyright: ignore[reportMissingImports]
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.grid = Grid(10, 10, obstacles={(2, 2), (3, 2), (4, 2)})
        self.robot = Robot(start_pos=(0, 0))
        goal = (5, 5)
        path = a_star(self.grid, self.robot.position, goal)
        self.robot.set_path(path)

        # Publishers pour les marqueurs RViz
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.timer = self.create_timer(1.0, self.move_robot)

        # Publier les obstacles initialement
        self.publish_obstacles()
        self.get_logger().info('Robot Controller started')

    def publish_obstacles(self):
        marker_array = MarkerArray()
        for idx, (x, y) in enumerate(self.grid.obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def publish_path(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Épaisseur de la ligne
        marker.color.b = 1.0  # Bleu
        marker.color.a = 1.0
        for x, y in self.robot.path:
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.0
            marker.points.append(point)
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def publish_robot_position(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.robot.position[0])
        marker.pose.position.y = float(self.robot.position[1])
        marker.pose.position.z = 0.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0  # Rouge
        marker.color.a = 1.0
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def move_robot(self):
        if not self.robot.path:
            self.get_logger().info(f'Robot arrived at {self.robot.position}')
            self.timer.cancel()
            self.publish_robot_position()
            return
        self.robot.move_step()
        self.publish_path()
        self.publish_robot_position()
        self.get_logger().info(f'Robot moved to {self.robot.position}')

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
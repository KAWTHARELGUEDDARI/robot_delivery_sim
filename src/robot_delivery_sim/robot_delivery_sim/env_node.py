import rclpy  # pyright: ignore[reportMissingImports]
from rclpy.node import Node  # pyright: ignore[reportMissingImports]
from robot_delivery_sim.grid import Grid # pyright: ignore[reportMissingImports]

class EnvNode(Node):
    def __init__(self):
        super().__init__('env_node')
        self.grid = Grid(10, 10, obstacles={(2,2), (3,2), (4,2)})
        self.get_logger().info('Environnement initialisé.')

def main(args=None):
    rclpy.init(args=args)
    node = EnvNode()
    rclpy.spin(node)
    rclpy.shutdown()

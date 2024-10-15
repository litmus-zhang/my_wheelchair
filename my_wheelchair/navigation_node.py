import rclpy
from rclpy.node import Node

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        # Node implementation

def main(args=None):
    print("Hello from my_wheelchair, navigation node")
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

class SensorsNode(Node):
    def __init__(self):
        super().__init__('sensors_node')
        # Node implementation

def main(args=None):
    print("Hello from my_wheelchair, sensors node")
    rclpy.init(args=args)
    node = SensorsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
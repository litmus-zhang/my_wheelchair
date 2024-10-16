import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class HeadlessImageViewerNode(Node):
    def __init__(self):
        super().__init__('headless_image_viewer_node')
        self.subscription = self.create_subscription(
            Image,
            'detected_objects',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, 'viewer_image', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # If you want to do any processing on the image, do it here
        # For example, you could resize the image:
        # cv_image = cv2.resize(cv_image, (640, 480))
        
        # Convert back to ROS Image message and publish
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.publisher.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = HeadlessImageViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
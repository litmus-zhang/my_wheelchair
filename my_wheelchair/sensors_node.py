import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class SensorsNode(Node):
    def __init__(self):
        super().__init__('sensors_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'detected_objects', 10)
        self.bridge = CvBridge()
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(msg)
    
    def avoid_obstacles(self):
        # Implement obstacle avoidance logic here
        # For now, just publish a stop command
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Perform object detection (this is a simple color-based detection)
        # Replace this with a more sophisticated algorithm as needed
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw bounding boxes around detected objects
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        cv2.imshow("Object Detection", cv_image)
        cv2.waitKey(1)
        # Publish the image with detected objects
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


def main(args=None):
    print("Hello from my_wheelchair, sensors x camera node")
    rclpy.init(args=args)
    node = SensorsNode()
    # node.timer_callback()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import RPi.GPIO as GPIO

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        # Node implementation
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription = self.create_subscription(
            String,
            'voice_command',
            self.voice_command_callback,
            10)
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        
        # Load predefined locations
        self.locations = self.load_locations()
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        self.front_wheel_pwm_pin = 17
        self.front_wheel_dir_pin = 27
        self.rear_ight_dir_pin = 23
        self.rear_right_dir_pin = 24
        self.rear_left_pwm_pin = 18
        self.rear_left_dir_pin = 22
        
        GPIO.setup(self.left_pwm_pin, GPIO.OUT)
        GPIO.setup(self.left_dir_pin, GPIO.OUT)
        GPIO.setup(self.right_pwm_pin, GPIO.OUT)
        GPIO.setup(self.right_dir_pin, GPIO.OUT)
        
        self.left_pwm = GPIO.PWM(self.left_pwm_pin, 1000)
        self.right_pwm = GPIO.PWM(self.right_pwm_pin, 1000)
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        
    def cmd_vel_callback(self, msg):
        left_speed = msg.linear.x - msg.angular.z
        right_speed = msg.linear.x + msg.angular.z
        
        self.set_motor_speed(left_speed, right_speed)


    def load_locations(self):
        # In a real scenario, load this from a file or parameter server
        return {
            "kitchen": {"x": 1.0, "y": 1.0, "z": 0.0},
            "living room": {"x": -1.0, "y": 2.0, "z": 0.0},
            "bedroom": {"x": 2.0, "y": -1.0, "z": 0.0}
        }

    def voice_command_callback(self, msg):
        command = msg.data.lower()
        for location, coordinates in self.locations.items():
            if location in command:
                self.send_goal(coordinates)
                return
        self.get_logger().info(f"Unknown location in command: {command}")
        

    def send_goal(self, coordinates):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = coordinates['x']
        goal_msg.pose.position.y = coordinates['y']
        goal_msg.pose.position.z = coordinates['z']
        goal_msg.pose.orientation.w = 1.0
        self.publisher_.publish(goal_msg)
        self.get_logger().info(f"Sent goal: {coordinates}")
        
        
    def set_motor_speed(self, left_speed, right_speed):
        # Set left motor
        GPIO.output(self.left_dir_pin, GPIO.HIGH if left_speed >= 0 else GPIO.LOW)
        self.left_pwm.ChangeDutyCycle(abs(left_speed) * 100)
        
        # Set right motor
        GPIO.output(self.right_dir_pin, GPIO.HIGH if right_speed >= 0 else GPIO.LOW)
        self.right_pwm.ChangeDutyCycle(abs(right_speed) * 100)

def main(args=None):
    print("Hello from my_wheelchair, navigation node")
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

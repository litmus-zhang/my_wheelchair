import nltk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
from nltk.tokenize import word_tokenize
from nltk.corpus import stopwords
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_ros


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        # Node implementation
        # Initialize NLTK
        try:
            nltk.data.find('tokenizers/punkt')
        except LookupError:
            nltk.download('punkt')
            nltk.download('stopwords')
        
        # Create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Create transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
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
        
        
        self.command_patterns = {
            'navigation': [
                r'(?:can you )?(?:please )?(?:go|navigate|move|head|take me) (?:to )?(?:the )?(\w+)',
                r'(?:i want to go|lets go|take me) (?:to )?(?:the )?(\w+)',
                r'(?:can you )?(?:show|tell|guide) me (?:the way|how to get) (?:to )?(?:the )?(\w+)'
            ],
            'location_query': [
                r'(?:where|what) (?:is|are) (?:the )?(?:available )?locations?',
                r'(?:show|list|tell me) (?:all )?(?:the )?(?:available )?locations'
            ],
            'position_query': [
                r'(?:where|what) (?:is|are) (?:my|our|the current) (?:location|position)',
                r'(?:tell|show) me (?:my|our|the current) (?:location|position)'
            ]
        }
        
        
        # Load predefined locations
        self.locations = self.load_locations()
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.front_wheel_pwm_pin = 17
        self.front_wheel_left_dir_pin = 27
        self.front_wheel_right_dir_pin = 23
        self.rear_wheel_forward_dir_pin = 24
        self.rear_wheel_backward_dir_pin = 22
        self.rear_wheel_pwm_pin = 18
        
        GPIO.setup(self.front_wheel_pwm_pin, GPIO.OUT)
        GPIO.setup(self.front_wheel_left_dir_pin, GPIO.OUT)
        GPIO.setup(self.front_wheel_right_dir_pin, GPIO.OUT)
        GPIO.setup(self.rear_wheel_forward_dir_pin, GPIO.OUT)
        GPIO.setup(self.rear_wheel_backward_dir_pin, GPIO.OUT)
        GPIO.setup(self.rear_wheel_pwm_pin, GPIO.OUT)
    
        
        self.rear_wheel_pwm_pin = GPIO.PWM(self.rear_wheel_pwm_pin, 1000)
        self.front_wheel_pwm_pin = GPIO.PWM(self.front_wheel_pwm_pin, 1000)
        self.rear_wheel_pwm_pin.start(0)
        self.front_wheel_pwm_pin.start(0)
        
    def cmd_vel_callback(self, msg):
        left_speed = msg.linear.x - msg.angular.z
        right_speed = msg.linear.x + msg.angular.z
        
        self.set_motor_speed(left_speed, right_speed)
    
    
    def parse_voice_command(self, command):
        """Parse voice command using NLTK"""
        tokens = word_tokenize(command.lower())
        stop_words = set(stopwords.words('english'))
        filtered_tokens = [w for w in tokens if w not in stop_words]
        
        # Check for navigation commands
        if any(word in filtered_tokens for word in ['go', 'navigate', 'move', 'take']):
            # Extract location from command
            for token in filtered_tokens:
                if token in self.locations:
                    return ('navigate', token)
        
        # Check for location queries
        if any(word in filtered_tokens for word in ['where', 'locations', 'list']):
            return ('list_locations', None)
        
        # Check for position queries
        if any(word in filtered_tokens for word in ['position', 'current']):
            return ('get_position', None)
            
        return (None, None)


    def load_locations(self):
        # In a real scenario, load this from a file or parameter server
        return {
            "kitchen": {"x": 1.0, "y": 1.0, "z": 0.0},
            "living room": {"x": -1.0, "y": 2.0, "z": 0.0},
            "bedroom": {"x": 2.0, "y": -1.0, "z": 0.0}
        }

    
    def basic_movement(self, msg):
        # get command
        command = msg.data.lower()
        
        self.get_logger().info(f"Direction in command: {command}")
        
        # check if command is "Forward". "Backward", "Left", "Right"
        if "forward" in command:
            # Move forward
            GPIO.output(self.rear_wheel_forward_dir_pin, GPIO.HIGH )
            time.sleep(5)
            
        elif "backward" in command:
            # Move backward
            GPIO.output(self.rear_wheel_backward_dir_pin, GPIO.HIGH )
            time.sleep(5)
        else: 
            # No Movement
            GPIO.output(self.rear_wheel_backward_dir_pin, GPIO.LOW )
            GPIO.output(self.rear_wheel_forward_dir_pin, GPIO.LOW )
            
            
        
        
        
        
        # GPIO.output(self.front_wheel_left_dir_pin, GPIO.HIGH )
        # GPIO.output(self.rear_wheel_forward_dir_pin, GPIO.HIGH )
        # print("Turning left")
        # time.sleep(5)
        # GPIO.output(self.front_wheel_left_dir_pin, GPIO.LOW )
        # GPIO.output(self.rear_wheel_forward_dir_pin, GPIO.LOW )
        
        # time.sleep(5)
        # GPIO.output(self.front_wheel_right_dir_pin, GPIO.HIGH )
        # GPIO.output(self.rear_wheel_backward_dir_pin, GPIO.HIGH )
        # print("Turning RIght")
        # time.sleep(5)
        # GPIO.output(self.front_wheel_right_dir_pin, GPIO.LOW )
        # GPIO.output(self.rear_wheel_backward_dir_pin, GPIO.LOW )
        
        
        
        

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
        # Set front motor
        GPIO.output(self.front_wheel_left_dir_pin, GPIO.HIGH if left_speed >= 0 else GPIO.LOW)
        GPIO.output(self.front_wheel_right_dir_pin, GPIO.HIGH if left_speed >= 0 else GPIO.LOW)
        self.front_wheel_pwm_pin.ChangeDutyCycle(abs(left_speed) * 100)
        
        # Set rears motors
        GPIO.output(self.rear_wheel_forward_dir_pin, GPIO.HIGH if right_speed >= 0 else GPIO.LOW)
        GPIO.output(self.rear_wheel_backward_dir_pin, GPIO.HIGH if right_speed >= 0 else GPIO.LOW)
        self.rear_wheel_pwm_pin.ChangeDutyCycle(abs(right_speed) * 100)

def main(args=None):
    print("Hello from my_wheelchair, navigation node")
    
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

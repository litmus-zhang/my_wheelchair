import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import pyttsx3

class VoiceRecognitionNode(Node):
    def __init__(self):
        super().__init__('voice_feedback_node')
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)
        self.subscription = self.create_subscription(
            String,
            'navigation_result',
            self.navigation_result_callback,
            10)
        self.timer = self.create_timer(1.0, self.listen_for_command)
        self.recognizer = sr.Recognizer()
        self.engine = pyttsx3.init()

    def listen_for_command(self):
        with sr.Microphone() as source:
            self.get_logger().info("Listening for a command...")
            audio = self.recognizer.listen(source)

        try:
            command = self.recognizer.recognize_google(audio)
            self.get_logger().info(f"Recognized command: {command}")
            msg = String()
            msg.data = command
            self.publisher_.publish(msg)
        except sr.UnknownValueError:
            self.get_logger().info("Could not understand audio")
        except sr.RequestError as e:
            self.get_logger().error(f"Could not request results: {e}")

    def navigation_result_callback(self, msg):
        self.get_logger().info(f"Navigation result: {msg.data}")
        self.engine.say(msg.data)
        self.engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
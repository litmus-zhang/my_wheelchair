import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class VoiceRecognitionNode(Node):
    def __init__(self):
        super().__init__('voice_recognition_node')
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)
        self.timer = self.create_timer(1.0, self.listen_for_command)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

    def listen_for_command(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            self.get_logger().info('Listening for a command...')
            audio = self.recognizer.listen(source)

        try:
            command = self.recognizer.recognize_google(audio)
            self.get_logger().info(f'Recognized command: {command}')
            msg = String()
            msg.data = command
            self.publisher_.publish(msg)
        except sr.UnknownValueError:
            self.get_logger().info('Could not understand the audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Could not request results; {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
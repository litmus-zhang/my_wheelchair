import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
import speech_recognition as sr
import networkx as nx
import pyaudio


class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        
        # Subscribe to current position updates
        self.position_subscription = self.create_subscription(
            PoseStamped,
            'current_position',
            self.position_callback,
            10)
        
        # Publisher for sending route to navigation node
        self.route_publisher = self.create_publisher(Path, 'route', 10)
        
        # Publisher for sending directions to navigation node
        self.direction_publisher = self.create_publisher(String, 'direction', 10)
        
        # Initialize current position
        self.current_position = None
        
        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        
        pa = pyaudio.PyAudio()
        default_input_device_info = pa.get_default_input_device_info()
        device_index = default_input_device_info['index']
        print(f"device_index: {device_index}")
        # self.timer = self.create_timer(1.0, self.listen_for_command)
        self.microphone = sr.Microphone(device_index=device_index)
        
        # Initialize graph for pathfinding
        self.graph = self.create_graph()
        
        # Timer for periodic path planning
        self.create_timer(1.0, self.plan_and_publish_route)

    def position_callback(self, msg):
        self.current_position = msg.pose

    def get_current_location(self):
        if self.current_position is None:
            self.get_logger().warn("Current position not available")
            return None
        return (self.current_position.position.x, self.current_position.position.y)

    def get_destination(self):
        with sr.Microphone() as source:
            self.get_logger().info("Say your destination...")
            audio = self.recognizer.listen(source)
            
            try:
                destination = self.recognizer.recognize_whisper(audio, language="english", model="small")
                self.get_logger().info(f"Destination recognized: {destination}")
                return destination
            except sr.UnknownValueError:
                self.get_logger().warn("Could not understand audio")
            except sr.RequestError as e:
                self.get_logger().error(f"Could not request results; {e}")
            return None

    def create_graph(self):
        # Create a simple graph for pathfinding
        # In a real scenario, this would be much more complex and based on actual map data
        G = nx.Graph()
        G.add_edge("entrance", "hallway", weight=5)
        G.add_edge("hallway", "living room", weight=3)
        G.add_edge("hallway", "kitchen", weight=4)
        G.add_edge("living room", "bedroom", weight=6)
        return G

    def path_to_destination(self, start, end):
        try:
            path = nx.shortest_path(self.graph, start, end, weight='weight')
            return path
        except nx.NetworkXNoPath:
            self.get_logger().warn(f"No path found from {start} to {end}")
            return None

    def plan_and_publish_route(self):
        current_location = self.get_current_location()
        if current_location is None:
            return

        destination = self.get_destination()
        if destination is None:
            return

        path = self.path_to_destination(current_location, destination)
        if path is None:
            return

        # Create and publish Path message
        route_msg = Path()
        route_msg.header.stamp = self.get_clock().now().to_msg()
        route_msg.header.frame_id = "map"
        
        for point in path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            route_msg.poses.append(pose)
        
        self.route_publisher.publish(route_msg)

        # Publish first direction
        if len(path) > 1:
            direction = self.get_direction(path[0], path[1])
            direction_msg = String()
            direction_msg.data = direction
            self.direction_publisher.publish(direction_msg)

    def get_direction(self, point1, point2):
        # Simplified direction calculation
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        if abs(dx) > abs(dy):
            return "east" if dx > 0 else "west"
        else:
            return "north" if dy > 0 else "south"

def main(args=None):
    print("Hello from my_wheelchair, mapping node")
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
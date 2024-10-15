import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class NavigationCommandNode(Node):
    def __init__(self):
        super().__init__('navigation_feedback_node')
        self.subscription = self.create_subscription(
            String,
            'voice_command',
            self.voice_command_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'navigation_result', 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def voice_command_callback(self, msg):
        self.get_logger().info(f"Received voice command: {msg.data}")
        # Here you would implement logic to convert the voice command to a goal pose
        goal_pose = self.convert_command_to_pose(msg.data)
        self.navigate_to_pose(goal_pose)

    def convert_command_to_pose(self, command):
        # This is a placeholder. You need to implement logic to convert
        # voice commands to actual poses based on your map and environment
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 1.0  # Example values
        goal_pose.pose.position.y = 1.0
        goal_pose.pose.orientation.w = 1.0
        return goal_pose

    def navigate_to_pose(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_to_pose_client.wait_for_server()
        
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.result == True:
            self.publisher_.publish(String(data='Navigation successful'))
        else:
            self.publisher_.publish(String(data='Navigation failed'))

def main(args=None):
    rclpy.init(args=args)
    node = NavigationCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavGoalClient(Node):
    def __init__(self):
        super().__init__('nav_goal_client', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw=0.0):
        print("Waiting for /navigate_to_pose action server...", flush=True)
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            print("Action server /navigate_to_pose not available!", flush=True)
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = Time(seconds=0).to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        print(f"Sending autonomous goal: x={x}, y={y}...", flush=True)
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f"[NAV FEEDBACK] Distance remaining: {feedback.distance_remaining:.2f} m | Time: {feedback.navigation_time.sec}s", flush=True)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal was rejected by planner/controller server!", flush=True)
            return

        print("Goal accepted! Robot is navigating...", flush=True)
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        print(f"Goal finished with status code: {status}!", flush=True)
        rclpy.shutdown()

def main():
    rclpy.init()
    x = float(sys.argv[1]) if len(sys.argv) > 1 else -0.5
    y = float(sys.argv[2]) if len(sys.argv) > 2 else -0.5
    client = NavGoalClient()
    if client.send_goal(x, y):
        rclpy.spin(client)

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

class RobotTracker(Node):
    def __init__(self):
        super().__init__('robot_tracker')
        self.bridge = CvBridge()
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_cam = self.create_subscription(Image, '/camera/image_raw', self.cam_cb, 10)
        self.last_img = None
        self.last_x = None
        self.last_y = None
        self.start_x = None
        self.start_y = None
        self.frame_count = 0
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.out_dir = '/mnt/c/Users/Jaya/Downloads/RC/nav_frames'
        os.makedirs(self.out_dir, exist_ok=True)
        print("RobotTracker initialized. Saving frames to", self.out_dir, flush=True)

    def odom_cb(self, msg):
        self.last_x = msg.pose.pose.position.x
        self.last_y = msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x = self.last_x
            self.start_y = self.last_y

    def cam_cb(self, msg):
        try:
            self.last_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            pass

    def timer_cb(self):
        if self.last_x is not None and self.last_img is not None:
            self.frame_count += 1
            dist = ((self.last_x - self.start_x)**2 + (self.last_y - self.start_y)**2)**0.5
            cv_img = self.last_img.copy()
            # Draw HUD telemetry
            hud_text = f"Frame {self.frame_count} | X: {self.last_x:.2f} Y: {self.last_y:.2f} | Dist: {dist:.2f}m"
            cv2.putText(cv_img, hud_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(cv_img, "Ackermann RPP Autonomous Nav", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            frame_path = os.path.join(self.out_dir, f"frame_{self.frame_count:03d}.png")
            cv2.imwrite(frame_path, cv_img)
            # Also keep latest frame as robot_live_view.png
            cv2.imwrite('/mnt/c/Users/Jaya/Downloads/RC/robot_live_view.png', cv_img)
            print(f"Tracking: x={self.last_x:.3f}, y={self.last_y:.3f}, dist={dist:.3f}m -> saved {frame_path}", flush=True)

def main():
    rclpy.init()
    node = RobotTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

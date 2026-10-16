import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.saved = False

    def callback(self, msg):
        if not self.saved:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite('/mnt/c/Users/Jaya/Downloads/RC/robot_camera_view.png', cv_img)
            self.saved = True
            print("Successfully saved robot camera frame to /mnt/c/Users/Jaya/Downloads/RC/robot_camera_view.png")
            raise SystemExit

def main():
    rclpy.init()
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

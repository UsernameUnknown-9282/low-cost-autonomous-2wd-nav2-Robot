import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

rclpy.init()
node = Node("diag_pub")
pub = node.create_publisher(Twist, "/cmd_vel", 10)
msg = Twist()
msg.linear.x = 0.4
print("Publishing /cmd_vel linear.x = 0.4 m/s...")
for _ in range(20):
    pub.publish(msg)
    time.sleep(0.1)
print("Finished test twist publish.")
rclpy.shutdown()

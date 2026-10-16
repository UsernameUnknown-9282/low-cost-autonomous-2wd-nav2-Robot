import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import time

rclpy.init()
node = Node("tf_checker")
buf = Buffer()
listener = TransformListener(buf, node)

start = time.time()
while time.time() - start < 3.0:
    rclpy.spin_once(node, timeout_sec=0.1)

try:
    tf_val = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
    t = tf_val.transform.translation
    print(f"Robot current pose in map frame: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}")
except Exception as e:
    print(f"Lookup error: {e}")

rclpy.shutdown()

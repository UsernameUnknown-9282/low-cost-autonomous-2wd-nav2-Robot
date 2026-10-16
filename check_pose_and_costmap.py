import rclpy
from rclpy.node import Node
import tf2_ros
from nav_msgs.msg import OccupancyGrid
import numpy as np

class CostmapChecker(Node):
    def __init__(self):
        super().__init__('costmap_checker')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.sub_costmap = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_cb, qos
        )
        self.costmap = None

    def costmap_cb(self, msg):
        self.costmap = msg

def main():
    rclpy.init()
    checker = CostmapChecker()
    print("Waiting for TF and Costmap...")
    for _ in range(50):
        rclpy.spin_once(checker, timeout_sec=0.1)
        if checker.costmap is not None and checker.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
            break

    try:
        t = checker.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        print(f"Robot pose in map: x={t.transform.translation.x:.3f}, y={t.transform.translation.y:.3f}")
    except Exception as e:
        print(f"TF lookup error: {e}")

    if checker.costmap:
        info = checker.costmap.info
        print(f"Costmap: res={info.resolution}, w={info.width}, h={info.height}, origin=({info.origin.position.x}, {info.origin.position.y})")
        data = np.array(checker.costmap.data).reshape((info.height, info.width))
        
        def get_cost(x, y):
            mx = int((x - info.origin.position.x) / info.resolution)
            my = int((y - info.origin.position.y) / info.resolution)
            if 0 <= mx < info.width and 0 <= my < info.height:
                return data[my, mx], mx, my
            return None, mx, my

        for pt in [(-2.0, -0.5), (-1.23, -0.5), (-0.5, -0.5), (0.0, 0.0), (-1.0, -0.5), (-1.5, -0.5)]:
            cost, mx, my = get_cost(pt[0], pt[1])
            print(f"Point {pt}: cost={cost} (grid: {mx}, {my})")
    else:
        print("Costmap not received.")

    checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

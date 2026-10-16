import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped

def main():
    rclpy.init()
    node = Node('initial_pose_publisher')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = Time(seconds=0, nanoseconds=0).to_msg()
    msg.pose.pose.position.x = -2.0
    msg.pose.pose.position.y = -0.5
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068

    print("Publishing zero-timestamp initial pose to AMCL...")
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)

    print("Done publishing initial pose.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

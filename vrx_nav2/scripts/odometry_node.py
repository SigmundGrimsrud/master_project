#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from gazebo_msgs.msg import LinkStates

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        # Publisher for /odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # Subscriber to Gazebo link states
        self.link_states_sub = self.create_subscription(
            LinkStates, '/gazebo/link_states', self.link_states_callback, 10)
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # Boat link name in Gazebo
        self.boat_link_name = 'catamaran::base_link'

    def link_states_callback(self, msg):
        # Find the index of the boat's link
        try:
            boat_index = msg.name.index(self.boat_link_name)
        except ValueError:
            self.get_logger().warn(f"Link {self.boat_link_name} not found in Gazebo link states.")
            return

        # Extract pose and twist
        boat_pose = msg.pose[boat_index]
        boat_twist = msg.twist[boat_index]

        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'catamaran/base_link'
        odom_msg.pose.pose = boat_pose
        odom_msg.twist.twist = boat_twist
        self.odom_pub.publish(odom_msg)

        # Broadcast transform from odom to base_link
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'catamaran/base_link'
        transform.transform.translation.x = boat_pose.position.x
        transform.transform.translation.y = boat_pose.position.y
        transform.transform.translation.z = boat_pose.position.z
        transform.transform.rotation = boat_pose.orientation
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class CmdVelToThrustConverter(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_thrust_converter')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('propellar_base', 5.5),       # Distance between thrusters (meters)
                ('linear_scaling', 100.0),   # Scaling factor for linear velocity
                ('angular_scaling', 30.0),  # Scaling factor for angular velocity
                ('max_thrust', 1000.0),     # Maximum thrust output
                ('min_thrust', -1000.0),    # Minimum thrust output (reverse)
            ]
        )
        
        # Retrieve parameters
        self.propellar_base = self.get_parameter('propellar_base').value
        self.linear_scaling = self.get_parameter('linear_scaling').value
        self.angular_scaling = self.get_parameter('angular_scaling').value
        self.max_thrust = self.get_parameter('max_thrust').value
        self.min_thrust = self.get_parameter('min_thrust').value
        
        # Publishers for thrust commands
        self.left_thrust_pub = self.create_publisher(
            Float64,
            '/catamaran/thrusters/rear_left/thrust',
            10
        )
        self.right_thrust_pub = self.create_publisher(
            Float64,
            '/catamaran/thrusters/rear_right/thrust',
            10
        )
        
        # Subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
    
    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Calculate components
        linear_component = linear_x * self.linear_scaling
        angular_component = angular_z * self.angular_scaling * (self.propellar_base / 2.0)
        
        # Compute thrust for each side
        left_thrust = linear_component - angular_component
        right_thrust = linear_component + angular_component
        
        # Clamp thrust values within limits
        left_thrust = max(min(left_thrust, self.max_thrust), self.min_thrust)
        right_thrust = max(min(right_thrust, self.max_thrust), self.min_thrust)
        
        # Publish thrust commands
        self.left_thrust_pub.publish(Float64(data=left_thrust))
        self.right_thrust_pub.publish(Float64(data=right_thrust))

def main(args=None):
    rclpy.init(args=args)
    converter = CmdVelToThrustConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
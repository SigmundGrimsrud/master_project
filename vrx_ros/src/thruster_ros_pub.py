# thruster_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')

        # Create publishers for each thruster
        self.thruster_pub_rear_left = self.create_publisher(
            Float64,
            '/catamaran/thrusters/rear_left',  # Topic for rear left thruster
            10)
        
        self.thruster_pub_rear_right = self.create_publisher(
            Float64,
            '/catamaran/thrusters/rear_right',  # Topic for rear right thruster
            10)

    def set_rear_left_thrust(self, value: float):
        """Set thrust for the rear left thruster"""
        msg = Float64()
        msg.data = float(value)
        self.thruster_pub_rear_left.publish(msg)
        self.get_logger().info(f'Rear Left Thruster Command: {msg.data} N')

    def set_rear_right_thrust(self, value: float):
        """Set thrust for the rear right thruster"""
        msg = Float64()
        msg.data = float(value)
        self.thruster_pub_rear_right.publish(msg)
        self.get_logger().info(f'Rear Right Thruster Command: {msg.data} N')
    
    '''
    def move_forward(self, value: float):
        """Set thrust for both rear thrusters"""
        assert type(value) == float or type(value) == int
        self.set_rear_left_thrust(value)
        self.set_rear_right_thrust(value)
    '''
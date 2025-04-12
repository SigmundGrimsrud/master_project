import rclpy
from geometry_msgs.msg import Pose, Twist

class CmdVelTransform:
    def __init__(self):
        self.cmd_vel_sub = rclpy.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    # Side thrusters
    # /catamaran/thrusters/front_right/thrust
    # /catamaran/thrusters/front_left/thrust
    # Rear thrusters
    # /catamaran/thrusters/rear_right/thrust
    # /catamaran/thrusters/rear_left/thrust

    def cmd_vel_callback(self, msg):
        # X forward, Y left, Z up

        self.angular_velocity = msg.angular.z

        if self.angular_velocity > 0:
            ...
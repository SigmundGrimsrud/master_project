#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class SetDatum(Node):
    def __init__(self):
        super().__init__('set_datum')
        self.pub = self.create_publisher(NavSatFix, '/datum', 10)
        self.timer = self.create_timer(1.0, self.publish_datum)

    def publish_datum(self):
        datum = NavSatFix()
        datum.latitude = 58.72058
        datum.longitude = 9.23418
        self.pub.publish(datum)

def main(args=None):
    rclpy.init(args=args)
    node = SetDatum()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
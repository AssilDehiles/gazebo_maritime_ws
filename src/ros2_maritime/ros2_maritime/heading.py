#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float64

class HeadingNode(Node):
    def __init__(self):
        super().__init__("heading_node")
        self.declare_parameter("magnetic_declination", 0.0087)
        self.decl = self.get_parameter("magnetic_declination").value
        self.sub = self.create_subscription(MagneticField, "/imu/mag", self.callback, 10)
        self.pub = self.create_publisher(Float64, "/usv/heading_deg", 10)
        self.get_logger().info("Noeud cap demarre")
    def callback(self, msg):
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        heading_rad = math.atan2(my, mx) - self.decl
        heading_rad = (heading_rad + 2 * math.pi) % (2 * math.pi)
        out = Float64()
        out.data = math.degrees(heading_rad)
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HeadingNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

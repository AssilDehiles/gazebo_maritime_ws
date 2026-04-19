#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float64

# Seuil minimal de norme de champ magnétique valide (µT)
_MAG_MIN_NORM = 1e-6


class HeadingNode(Node):
    def __init__(self):
        super().__init__("heading_node")
        self.declare_parameter("magnetic_declination", 0.0087)  # rad (~0.5° pour Alger)
        self.decl = self.get_parameter("magnetic_declination").value
        self.create_subscription(MagneticField, "/imu/mag", self.callback, 10)
        self.pub = self.create_publisher(Float64, "/usv/heading_deg", 10)
        self.get_logger().info("Heading node started")

    def callback(self, msg):
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y

        # Rejette les mesures de champ nul ou quasi-nul (interférence / capteur HS)
        if math.hypot(mx, my) < _MAG_MIN_NORM:
            self.get_logger().warn(
                "Magnetic field magnitude near zero — heading discarded", throttle_duration_sec=5.0
            )
            return

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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuRelay(Node):
    """
    Relaie les messages IMU en corrigeant le frame_id à 'base_link',
    requis par robot_localization (EKF).
    """

    def __init__(self):
        super().__init__("imu_relay")
        self.sub = self.create_subscription(Imu, "/sensor/imu", self.callback, 100)
        self.pub = self.create_publisher(Imu, "/sensor/imu_fixed", 100)
        self.get_logger().info("IMU relay started")

    def callback(self, msg):
        msg.header.frame_id = "base_link"
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImuRelay())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

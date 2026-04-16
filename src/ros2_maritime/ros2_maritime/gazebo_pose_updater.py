#!/usr/bin/env python3
import rclpy, math, subprocess, time
from rclpy.node import Node
from nav_msgs.msg import Odometry

class GazeboPoseUpdater(Node):
    def __init__(self):
        super().__init__("gazebo_pose_updater")
        self.last_update = 0.0
        self.min_interval = 0.1  # 10 Hz max
        self.create_subscription(Odometry, "/usv/odometry", self.odom_cb, 10)
        self.get_logger().info("Pose updater demarre - 10Hz")

    def odom_cb(self, msg):
        now = time.time()
        if now - self.last_update < self.min_interval:
            return
        self.last_update = now
        x  = msg.pose.pose.position.x
        y  = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        req = f"name: 'wamv' position: {{x: {x:.3f}, y: {y:.3f}, z: 0.25}} "
        req += f"orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}"
        subprocess.Popen([
            "gz", "service",
            "-s", "/world/usv_monde_leger/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "500",
            "--req", req
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GazeboPoseUpdater())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

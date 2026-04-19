#!/usr/bin/env python3
import rclpy
import subprocess
from rclpy.node import Node
from nav_msgs.msg import Odometry


class GazeboPoseUpdater(Node):
    def __init__(self):
        super().__init__("gazebo_pose_updater")
        # Horodatage du dernier envoi (nanosecondes ROS)
        self._last_update_ns = 0
        self._min_interval_ns = int(0.1 * 1e9)  # 10 Hz max
        self.create_subscription(Odometry, "/usv/odometry", self.odom_cb, 10)
        self.get_logger().info("Pose updater started — 10 Hz")

    def odom_cb(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_update_ns < self._min_interval_ns:
            return
        self._last_update_ns = now_ns

        x  = msg.pose.pose.position.x
        y  = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Les valeurs sont formatées en flottants : pas de risque d'injection
        req = (
            f"name: 'wamv' "
            f"position: {{x: {x:.3f}, y: {y:.3f}, z: 0.25}} "
            f"orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}"
        )
        proc = subprocess.Popen(
            [
                "gz", "service",
                "-s", "/world/usv_monde_leger/set_pose",
                "--reqtype", "gz.msgs.Pose",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "500",
                "--req", req,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        # Vérification non-bloquante : log si le process se termine en erreur
        ret = proc.poll()
        if ret is not None and ret != 0:
            err = proc.stderr.read().decode(errors="replace").strip()
            self.get_logger().warn(f"gz service failed (code {ret}): {err}")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GazeboPoseUpdater())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

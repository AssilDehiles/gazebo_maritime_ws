#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class DynamicsSim(Node):
    def __init__(self):
        super().__init__("dynamics_sim")

        # Paramètres physiques du USV (configurables via ROS params)
        self.declare_parameter("mass",    180.0)   # kg
        self.declare_parameter("inertia", 446.0)   # kg·m²
        self.declare_parameter("Xu",      -50.0)   # amortissement surge
        self.declare_parameter("Yv",     -100.0)   # amortissement sway
        self.declare_parameter("Nr",     -100.0)   # amortissement yaw
        self.declare_parameter("thruster_arm", 2.0)  # demi-entraxe thrusters (m)

        self.m   = self.get_parameter("mass").value
        self.Iz  = self.get_parameter("inertia").value
        self.Xu  = self.get_parameter("Xu").value
        self.Yv  = self.get_parameter("Yv").value
        self.Nr  = self.get_parameter("Nr").value
        self.L   = self.get_parameter("thruster_arm").value

        # État cinématique
        self.x   = 0.0
        self.y   = 0.0
        self.psi = 0.0  # cap (rad)
        self.u   = 0.0  # vitesse surge
        self.v   = 0.0  # vitesse sway
        self.r   = 0.0  # taux de lacet

        # Commandes de poussée courantes
        self.F_left  = 0.0
        self.F_right = 0.0

        self.dt = 0.01  # 100 Hz

        self.create_subscription(
            Float64, "/left_thrust_cmd",
            lambda m: setattr(self, "F_left",  m.data), 10
        )
        self.create_subscription(
            Float64, "/right_thrust_cmd",
            lambda m: setattr(self, "F_right", m.data), 10
        )
        self.pub = self.create_publisher(Odometry, "/usv/odometry", 10)
        self.create_timer(self.dt, self.update)
        self.get_logger().info("Dynamics simulator started")

    def update(self):
        Fu = self.F_left + self.F_right
        Mz = self.L * (self.F_right - self.F_left)

        # Dynamique du corps (modèle 3-DDL maneuvering)
        self.u   += ((Fu + self.Xu * self.u) / self.m)  * self.dt
        self.v   += ((self.Yv * self.v)       / self.m)  * self.dt
        self.r   += ((Mz + self.Nr * self.r)  / self.Iz) * self.dt

        # Cinématique dans le repère monde
        self.x   += (self.u * math.cos(self.psi) - self.v * math.sin(self.psi)) * self.dt
        self.y   += (self.u * math.sin(self.psi) + self.v * math.cos(self.psi)) * self.dt
        self.psi += self.r * self.dt

        # Construction et publication de l'odométrie
        o = Odometry()
        o.header.stamp           = self.get_clock().now().to_msg()
        o.header.frame_id        = "odom"
        o.child_frame_id         = "base_link"
        o.pose.pose.position.x   = self.x
        o.pose.pose.position.y   = self.y
        o.pose.pose.orientation.z = math.sin(self.psi / 2)
        o.pose.pose.orientation.w = math.cos(self.psi / 2)
        o.twist.twist.linear.x   = self.u
        o.twist.twist.angular.z  = self.r
        self.pub.publish(o)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DynamicsSim())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

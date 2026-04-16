#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

class DynamicsSim(Node):
    def __init__(self):
        super().__init__("dynamics_sim")
        self.x=0.0; self.y=0.0; self.psi=0.0
        self.u=0.0; self.v=0.0; self.r=0.0
        self.m=180.0; self.Iz=446.0
        self.Xu=-50.0; self.Yv=-100.0; self.Nr=-100.0
        self.L=2.0; self.dt=0.01
        self.F_left=0.0; self.F_right=0.0
        self.create_subscription(Float64,"/left_thrust_cmd",lambda m: setattr(self,"F_left",m.data),10)
        self.create_subscription(Float64,"/right_thrust_cmd",lambda m: setattr(self,"F_right",m.data),10)
        self.pub = self.create_publisher(Odometry,"/usv/odometry",10)
        self.create_timer(self.dt, self.update)
        self.get_logger().info("Simulateur dynamique demarre")
    def update(self):
        Fu = self.F_left + self.F_right
        Nr = self.L * (self.F_right - self.F_left)
        self.u += ((Fu + self.Xu*self.u)/self.m)*self.dt
        self.v += ((self.Yv*self.v)/self.m)*self.dt
        self.r += ((Nr + self.Nr*self.r)/self.Iz)*self.dt
        self.x += (self.u*math.cos(self.psi)-self.v*math.sin(self.psi))*self.dt
        self.y += (self.u*math.sin(self.psi)+self.v*math.cos(self.psi))*self.dt
        self.psi += self.r*self.dt
        o=Odometry()
        o.header.stamp=self.get_clock().now().to_msg()
        o.header.frame_id="odom"
        o.child_frame_id="base_link"
        o.pose.pose.position.x=self.x
        o.pose.pose.position.y=self.y
        o.pose.pose.orientation.z=math.sin(self.psi/2)
        o.pose.pose.orientation.w=math.cos(self.psi/2)
        o.twist.twist.linear.x=self.u
        o.twist.twist.angular.z=self.r
        self.pub.publish(o)
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DynamicsSim())
    rclpy.shutdown()
if __name__ == "__main__":
    main()

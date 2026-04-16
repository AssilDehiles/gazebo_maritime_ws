#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")
        # Waypoint cible (latitude, longitude)
        self.declare_parameter("target_lat", 36.7548)
        self.declare_parameter("target_lon", 3.0588)
        self.declare_parameter("acceptance_radius", 2.0)
        self.target_lat = self.get_parameter("target_lat").value
        self.target_lon = self.get_parameter("target_lon").value
        self.acceptance_radius = self.get_parameter("acceptance_radius").value
        # Etat courant
        self.current_lat = None
        self.current_lon = None
        self.current_heading = 0.0
        self.reached = False
        # Subscribers
        self.create_subscription(Odometry, "/usv/odometry", self.odom_cb, 10)
        self.create_subscription(Float64, "/usv/heading_deg", self.heading_cb, 10)
        # Publishers commandes moteurs
        self.pub_left  = self.create_publisher(Float64, "/left_thrust_cmd",  10)
        self.pub_right = self.create_publisher(Float64, "/right_thrust_cmd", 10)
        # Timer navigation 10Hz
        self.create_timer(0.1, self.navigate)
        self.get_logger().info(f"Navigateur demarre — cible: ({self.target_lat:.5f}, {self.target_lon:.5f})")

    def odom_cb(self, msg):
        # Convertir position locale (m) en GPS approximatif
        self.current_lat = 36.7538 + msg.pose.pose.position.y / 111000.0
        self.current_lon = 3.0588  + msg.pose.pose.position.x / (111000.0 * 0.809)


    def heading_cb(self, msg):
        self.current_heading = msg.data

    def haversine(self, lat1, lon1, lat2, lon2):
        """Distance en metres entre deux points GPS"""
        R = 6371000.0
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def bearing(self, lat1, lon1, lat2, lon2):
        """Cap desire en degres (0=Nord, 90=Est)"""
        dlon = math.radians(lon2 - lon1)
        x = math.sin(dlon) * math.cos(math.radians(lat2))
        y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlon)
        return (math.degrees(math.atan2(x, y)) + 360) % 360

    def navigate(self):
        if self.current_lat is None or self.reached:
            return
        # Calculer distance et cap vers la cible
        distance = self.haversine(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
        desired_bearing = self.bearing(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
        # Erreur de cap
        heading_error = desired_bearing - self.current_heading
        if heading_error > 180:  heading_error -= 360
        if heading_error < -180: heading_error += 360
        self.get_logger().info(f"Distance: {distance:.1f}m | Cap desire: {desired_bearing:.1f} | Erreur: {heading_error:.1f}")
        # Verifier si cible atteinte
        if distance < self.acceptance_radius:
            self.get_logger().info("CIBLE ATTEINTE !")
            self.stop()
            self.reached = True
            return
        # Gains du controleur
        Kp_thrust = 0.5   # gain vitesse
        Kp_yaw    = 0.8   # gain cap
        thrust_max = 80.0
        # Commande de base proportionnelle a la distance
        base_thrust = min(Kp_thrust * distance, thrust_max)
        # Correction de cap
        yaw_correction = Kp_yaw * heading_error
        yaw_correction = max(-40.0, min(40.0, yaw_correction))
        # Commandes gauche/droite
        left_thrust  = base_thrust - yaw_correction
        right_thrust = base_thrust + yaw_correction
        self.publish_thrust(left_thrust, right_thrust)

    def publish_thrust(self, left, right):
        l, r = Float64(), Float64()
        l.data, r.data = float(left), float(right)
        self.pub_left.publish(l)
        self.pub_right.publish(r)

    def stop(self):
        self.publish_thrust(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Navigator())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

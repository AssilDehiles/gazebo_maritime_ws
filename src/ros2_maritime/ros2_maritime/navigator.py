#!/usr/bin/env python3
"""
navigator.py — contrôleur GPS multi-waypoints pour USV différentiel (wamv)

Mission :
  Parcourt séquentiellement une liste de waypoints GPS.  Quand la distance au
  waypoint courant passe sous `acceptance_radius`, le nœud passe automatiquement
  au suivant.  Après le dernier waypoint, la mission est terminée (moteurs arrêtés).

Contrôle :
  • PID cap    : erreur de cap (°) → correction différentielle gauche/droite
  • PID thrust : distance (m)      → poussée de base

Les deux contrôleurs utilisent :
  - Filtre dérivé du 1er ordre     : élimine l'amplification du bruit mesure
  - Anti-windup par back-calculation (Åström-Hägglund) : empêche le dépassement
    dû à la saturation prolongée de l'intégrateur

Priorité cap : la poussée est réduite linéairement quand l'erreur de cap
dépasse `heading_priority_angle` (90 ° par défaut).  Le bateau pivote d'abord,
avance ensuite.

Paramétrage de la liste de waypoints :
  ros2 run ros2_maritime navigator --ros-args \\
    -p waypoints_lat:="[36.7548, 36.7558, 36.7568]" \\
    -p waypoints_lon:="[3.0588,  3.0600,  3.0612]"

  Ou via un fichier YAML (recommandé pour les missions longues) :
    navigator:
      ros__parameters:
        waypoints_lat: [36.7548, 36.7558]
        waypoints_lon: [3.0588,  3.0600]
        acceptance_radius: 2.0

Topics publiés :
  /left_thrust_cmd   (std_msgs/Float64)
  /right_thrust_cmd  (std_msgs/Float64)
  /usv/waypoint_index (std_msgs/Int32)  — index 0-based du waypoint courant
"""
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from nav_msgs.msg import Odometry


# ─────────────────────────────────────────────────────────────────────────────
class PIDController:
    """
    PID discret avec filtre dérivé et anti-windup par back-calculation.

    Filtre dérivé (passe-bas 1er ordre, coefficient N) :
        alpha      = 1 / (1 + N·dt)
        d_filt[k]  = alpha·d_filt[k-1] + (1-alpha)·(e[k]-e[k-1])/dt

        N → ∞ : dérivée pure (aucun filtrage)
        N → 0 : dérivée complètement filtrée (D inactif)
        Valeurs typiques : 5–20

    Anti-windup par back-calculation :
        u_raw  = Kp·e + Ki·I + Kd·d_filt
        u_sat  = clamp(u_raw, out_min, out_max)
        I[k]  += dt·(e + (u_sat - u_raw) / Tt)
        Tt     = sqrt(Kp/Ki)   [constante de temps de tracking]

        Quand u_raw dépasse la borne de saturation, le terme (u_sat - u_raw)/Tt
        pousse l'intégrateur dans la direction opposée, annulant progressivement
        l'excès accumulé sans rupture brutale.
    """

    def __init__(
        self,
        kp: float, ki: float, kd: float,
        out_min: float, out_max: float,
        int_min: float = None, int_max: float = None,
        N: float = 10.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.int_min = int_min if int_min is not None else out_min
        self.int_max = int_max if int_max is not None else out_max
        self.N = N

        # Constante de tracking anti-windup : Tt = sqrt(Kp/Ki)
        self._Tt = math.sqrt(kp / ki) if ki > 0.0 else float("inf")

        self._integral   = 0.0
        self._d_filtered = 0.0
        self._prev_error = 0.0

    def reset(self):
        """Remet à zéro l'état interne (appeler entre deux waypoints)."""
        self._integral   = 0.0
        self._d_filtered = 0.0
        self._prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        # ── Dérivée filtrée ───────────────────────────────────────────────
        alpha            = 1.0 / (1.0 + self.N * dt)
        raw_deriv        = (error - self._prev_error) / dt
        self._d_filtered = alpha * self._d_filtered + (1.0 - alpha) * raw_deriv

        # ── Sortie non saturée ────────────────────────────────────────────
        output_unsat = (
            self.kp * error
            + self.ki * self._integral
            + self.kd * self._d_filtered
        )

        # ── Saturation ────────────────────────────────────────────────────
        output = max(self.out_min, min(self.out_max, output_unsat))

        # ── Anti-windup back-calculation ──────────────────────────────────
        if self.ki != 0.0 and self._Tt != float("inf"):
            self._integral += dt * (error + (output - output_unsat) / self._Tt)
            self._integral  = max(self.int_min, min(self.int_max, self._integral))

        self._prev_error = error
        return output


# ─────────────────────────────────────────────────────────────────────────────
class Navigator(Node):

    def __init__(self):
        super().__init__("navigator")

        # ── Paramètres mission ────────────────────────────────────────────
        # Listes parallèles lat/lon ; un seul élément = mode waypoint unique.
        self.declare_parameter("waypoints_lat", [36.7548])
        self.declare_parameter("waypoints_lon", [3.0588])
        self.declare_parameter("acceptance_radius", 2.0)

        # Origine locale = origine du monde Gazebo
        self.declare_parameter("origin_lat", 36.7538)
        self.declare_parameter("origin_lon",  3.0588)

        # ── Paramètres PID cap ────────────────────────────────────────────
        self.declare_parameter("kp_yaw",  0.30)
        self.declare_parameter("ki_yaw",  0.001)
        self.declare_parameter("kd_yaw",  0.05)
        self.declare_parameter("N_yaw",  10.0)

        # ── Paramètres PID thrust ─────────────────────────────────────────
        self.declare_parameter("kp_thrust",  0.15)
        self.declare_parameter("ki_thrust",  0.0005)
        self.declare_parameter("kd_thrust",  0.02)
        self.declare_parameter("N_thrust",   5.0)

        # ── Paramètres comportement ───────────────────────────────────────
        # Seuil de priorité cap : au-delà (°), poussée réduite jusqu'à 0
        self.declare_parameter("heading_priority_angle", 90.0)
        # Saturation de la distance en entrée du PID thrust (évite windup)
        self.declare_parameter("dist_clamp", 50.0)

        # ── Lecture et validation des waypoints ───────────────────────────
        lats = list(self.get_parameter("waypoints_lat").value)
        lons = list(self.get_parameter("waypoints_lon").value)

        if len(lats) == 0:
            raise ValueError("waypoints_lat must contain at least one entry")
        if len(lats) != len(lons):
            raise ValueError(
                f"waypoints_lat ({len(lats)}) and waypoints_lon ({len(lons)}) "
                "must have the same length"
            )

        # Liste de tuples (lat, lon) dans l'ordre de parcours
        self._waypoints: list = list(zip(lats, lons))
        self._wp_idx: int     = 0          # index du waypoint courant
        self._mission_done: bool = False

        # ── Autres paramètres ─────────────────────────────────────────────
        self.acceptance_radius = self.get_parameter("acceptance_radius").value
        self.origin_lat        = self.get_parameter("origin_lat").value
        self.origin_lon        = self.get_parameter("origin_lon").value
        self._cos_lat          = math.cos(math.radians(self.origin_lat))
        self._heading_prio     = self.get_parameter("heading_priority_angle").value
        self._dist_clamp       = self.get_parameter("dist_clamp").value
        self.dt                = 0.1   # 10 Hz

        # ── Contrôleurs PID ───────────────────────────────────────────────
        self.pid_yaw = PIDController(
            kp=self.get_parameter("kp_yaw").value,
            ki=self.get_parameter("ki_yaw").value,
            kd=self.get_parameter("kd_yaw").value,
            out_min=-40.0, out_max=40.0,
            int_min= -5.0, int_max= 5.0,
            N=self.get_parameter("N_yaw").value,
        )
        self.pid_thrust = PIDController(
            kp=self.get_parameter("kp_thrust").value,
            ki=self.get_parameter("ki_thrust").value,
            kd=self.get_parameter("kd_thrust").value,
            out_min=0.0, out_max=50.0,
            int_min=0.0, int_max=20.0,
            N=self.get_parameter("N_thrust").value,
        )

        # ── État capteurs ─────────────────────────────────────────────────
        self.current_lat     = None
        self.current_lon     = None
        self.current_heading = 0.0

        # ── ROS I/O ───────────────────────────────────────────────────────
        self.create_subscription(Odometry, "/usv/odometry",    self.odom_cb,    10)
        self.pub_left   = self.create_publisher(Float64, "/left_thrust_cmd",    10)
        self.pub_right  = self.create_publisher(Float64, "/right_thrust_cmd",   10)
        self.pub_wp_idx = self.create_publisher(Int32,   "/usv/waypoint_index", 10)
        self.create_timer(self.dt, self.navigate)

        # Log mission au démarrage
        self.get_logger().info(
            f"Navigator started — {len(self._waypoints)} waypoint(s) | "
            f"acceptance: {self.acceptance_radius} m"
        )
        self._log_current_waypoint()

    # ── Helpers mission ───────────────────────────────────────────────────────

    @property
    def _target(self) -> tuple:
        """Retourne (lat, lon) du waypoint courant."""
        return self._waypoints[self._wp_idx]

    def _log_current_waypoint(self):
        lat, lon = self._target
        self.get_logger().info(
            f"  WP {self._wp_idx + 1}/{len(self._waypoints)} → "
            f"({lat:.6f}, {lon:.6f})"
        )

    def _advance_waypoint(self):
        """Passe au waypoint suivant ou termine la mission."""
        self.get_logger().info(
            f"WP {self._wp_idx + 1}/{len(self._waypoints)} reached!"
        )
        self.stop()
        self.pid_yaw.reset()
        self.pid_thrust.reset()

        self._wp_idx += 1

        if self._wp_idx >= len(self._waypoints):
            self._mission_done = True
            self.get_logger().info(
                f"Mission complete — all {len(self._waypoints)} waypoint(s) reached."
            )
        else:
            self._log_current_waypoint()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_cb(self, msg):
        """Position locale (m) → GPS approximatif via l'origine monde."""
        self.current_lat = (
            self.origin_lat + msg.pose.pose.position.y / 111_000.0
        )
        self.current_lon = (
            self.origin_lon + msg.pose.pose.position.x / (111_000.0 * self._cos_lat)
        )
        # Cap depuis quaternion odometry (convention nautique)
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw_rad = 2.0 * math.atan2(qz, qw)
        # yaw_rad = 0 → bateau pointe vers +x (Est)
        # cap nautique : 0=Nord → rotation de +90°
        self.current_heading = (90.0 - math.degrees(yaw_rad)) % 360.0

    # ── Géodésie ──────────────────────────────────────────────────────────────

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        """Distance en mètres entre deux points GPS."""
        R = 6_371_000.0
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(math.radians(lat1))
            * math.cos(math.radians(lat2))
            * math.sin(dlon / 2) ** 2
        )
        return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    @staticmethod
    def bearing(lat1, lon1, lat2, lon2):
        """Cap vers la cible en degrés (0 = Nord, 90 = Est)."""
        dlon = math.radians(lon2 - lon1)
        x = math.sin(dlon) * math.cos(math.radians(lat2))
        y = (
            math.cos(math.radians(lat1)) * math.sin(math.radians(lat2))
            - math.sin(math.radians(lat1))
            * math.cos(math.radians(lat2))
            * math.cos(dlon)
        )
        return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0

    # ── Boucle de navigation ──────────────────────────────────────────────────

    def navigate(self):
        if self.current_lat is None or self._mission_done:
            return

        # Publie l'index courant pour monitoring externe (RViz, rqt, etc.)
        idx_msg = Int32()
        idx_msg.data = self._wp_idx
        self.pub_wp_idx.publish(idx_msg)

        target_lat, target_lon = self._target

        distance = self.haversine(
            self.current_lat, self.current_lon,
            target_lat,       target_lon,
        )
        desired_bearing = self.bearing(
            self.current_lat, self.current_lon,
            target_lat,       target_lon,
        )

        # Erreur de cap normalisée dans [-180°, 180°]
        heading_error = desired_bearing - self.current_heading
        if heading_error >  180.0:
            heading_error -= 360.0
        elif heading_error < -180.0:
            heading_error += 360.0

        self.get_logger().info(
            f"[WP {self._wp_idx + 1}/{len(self._waypoints)}] "
            f"Dist: {distance:.1f} m | Bearing: {desired_bearing:.1f}° "
            f"| HeadErr: {heading_error:+.1f}°"
        )

        # ── Waypoint atteint → passage au suivant ─────────────────────────
        if distance < self.acceptance_radius:
            self._advance_waypoint()
            return

        # ── PID cap ───────────────────────────────────────────────────────
        yaw_correction = self.pid_yaw.compute(heading_error, self.dt)

        # ── Facteur de priorité cap ───────────────────────────────────────
        # |err| = 0°           → factor = 1.0  (pleine poussée)
        # |err| = prio_angle/2 → factor = 0.5
        # |err| ≥ prio_angle   → factor = 0.0  (pivot pur en place)
        heading_factor = max(
            0.0,
            1.0 - abs(heading_error) / self._heading_prio
        )

        # ── PID thrust ────────────────────────────────────────────────────
        dist_clamped = min(distance, self._dist_clamp)
        base_thrust  = self.pid_thrust.compute(dist_clamped, self.dt) * heading_factor

        # ── Commandes différentielles ─────────────────────────────────────
        self.publish_thrust(base_thrust + yaw_correction,
                            base_thrust - yaw_correction)

    # ── Utilitaires ───────────────────────────────────────────────────────────

    def publish_thrust(self, left: float, right: float):
        l, r = Float64(), Float64()
        l.data, r.data = float(left), float(right)
        self.pub_left.publish(l)
        self.pub_right.publish(r)

    def stop(self):
        self.publish_thrust(0.0, 0.0)


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Navigator())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

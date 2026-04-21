#!/usr/bin/env python3
"""
teleop_key.py — contrôle clavier temps réel du WAM-V (propulsion différentielle)

Touches :
  ↑  Avancer          left=50  right=50
  ↓  Reculer          left=-50 right=-50
  ←  Tourner gauche   left=20  right=50
  →  Tourner droite   left=50  right=20
  ESPACE  Stop        left=0   right=0
  Q       Quitter

Affichage temps réel (position x,y depuis /usv/odometry + commandes actives).
"""
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from pynput import keyboard


# ── Constantes ────────────────────────────────────────────────────────────────
CMDS = {
    "forward":  ( 50.0,  50.0),
    "backward": (-50.0, -50.0),
    "left":     ( 20.0,  50.0),
    "right":    ( 50.0,  20.0),
    "stop":     (  0.0,   0.0),
}

# Touches directionnelles maintenues (relâcher = stop)
_DIRECTIONAL_KEYS = {
    keyboard.Key.up,
    keyboard.Key.down,
    keyboard.Key.left,
    keyboard.Key.right,
}

_ACTION_LABELS = {
    "forward":  "AVANCER  ↑",
    "backward": "RECULER  ↓",
    "left":     "GAUCHE   ←",
    "right":    "DROITE   →",
    "stop":     "STOP     ■",
}

HELP = (
    "  ↑  Avancer    ↓  Reculer    ←  Gauche    →  Droite    "
    "ESPACE Stop    Q Quitter"
)


# ── Nœud ROS2 ─────────────────────────────────────────────────────────────────
class TeleopKeyNode(Node):

    def __init__(self):
        super().__init__("teleop_key")

        self.pub_left  = self.create_publisher(Float64, "/left_thrust_cmd",  10)
        self.pub_right = self.create_publisher(Float64, "/right_thrust_cmd", 10)
        self.create_subscription(Odometry, "/usv/odometry", self._odom_cb, 10)

        # Verrou protégeant les paires (left, right) : écrites depuis le thread
        # pynput, lues depuis le thread du ROS2 executor.
        self._cmd_lock = threading.Lock()
        self._left:   float = 0.0
        self._right:  float = 0.0

        # Données odométrie — écrites depuis le thread executor, lues depuis
        # le thread principal (affichage).  Lecture/écriture float en CPython
        # est atomique via le GIL ; pas de lock nécessaire ici.
        self._x:     float = 0.0
        self._y:     float = 0.0
        self._action: str  = "stop"

        self.quit_flag = threading.Event()

        # Publie les commandes courantes à 10 Hz
        self.create_timer(0.1, self._publish)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

    def _publish(self):
        with self._cmd_lock:
            left  = self._left
            right = self._right
        l, r = Float64(), Float64()
        l.data, r.data = left, right
        self.pub_left.publish(l)
        self.pub_right.publish(r)

    # ── Interface clavier ─────────────────────────────────────────────────────

    def set_cmd(self, action: str):
        left, right = CMDS[action]
        with self._cmd_lock:
            self._left  = left
            self._right = right
        self._action = action

    def display(self):
        label = _ACTION_LABELS.get(self._action, "?")
        with self._cmd_lock:
            left  = self._left
            right = self._right
        line = (
            f"\r  x={self._x:+8.2f} m  y={self._y:+8.2f} m"
            f"   |  {label}"
            f"   |  L={left:+5.0f} N  R={right:+5.0f} N   "
        )
        sys.stdout.write(line)
        sys.stdout.flush()


# ── Listener pynput ───────────────────────────────────────────────────────────

def _make_listener(node: TeleopKeyNode) -> keyboard.Listener:

    def on_press(key):
        if key == keyboard.Key.up:
            node.set_cmd("forward")
        elif key == keyboard.Key.down:
            node.set_cmd("backward")
        elif key == keyboard.Key.left:
            node.set_cmd("left")
        elif key == keyboard.Key.right:
            node.set_cmd("right")
        elif key == keyboard.Key.space:
            node.set_cmd("stop")
        elif hasattr(key, "char") and key.char in ("q", "Q"):
            node.set_cmd("stop")
            node.quit_flag.set()
            return False  # arrête le listener pynput

    def on_release(key):
        # Seules les touches directionnelles ont un comportement "mort-homme".
        # Espace est déjà un stop explicite ; le relâcher ne change rien.
        if key in _DIRECTIONAL_KEYS:
            node.set_cmd("stop")

    return keyboard.Listener(on_press=on_press, on_release=on_release)


# ── Point d'entrée ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyNode()
    listener = _make_listener(node)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()
    listener.start()

    print("\n── AquaRob Téléopération clavier ──")
    print(HELP)
    print()

    try:
        while not node.quit_flag.is_set() and rclpy.ok():
            node.display()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        listener.stop()
        # Ordre critique : shutdown d'abord → spin() retourne → join → destroy.
        # Inverser cet ordre (destroy avant shutdown) cause un crash car spin()
        # utilise encore le nœud.
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()

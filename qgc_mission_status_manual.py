import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from mavros_msgs.msg import WaypointList, WaypointReached, RCIn, ManualControl
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped

import math
import threading
import sys
import select
import termios
import tty


class QgcMissionBridge(Node):
    def __init__(self):
        super().__init__('qgc_mission_bridge')

        self.waypoints = []
        self.last_reached = 0
        self.position = None

        qos = qos_profile_sensor_data
        qos_mission = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Suscripciones MAVROS misión
        self.create_subscription(WaypointList, '/mavros/mission/waypoints', self.mission_callback, qos_mission)
        self.create_subscription(WaypointReached, '/mavros/mission/reached', self.reached_callback, qos_mission)

        # Suscripciones estado UGV
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos)
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, qos)
        self.create_subscription(RCIn, '/mavros/rc/in', self.rc_callback, qos)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_body', self.vel_callback, qos)

        # Publicador del waypoint deseado
        self.goal_pub = self.create_publisher(TwistStamped, 'ugv/mission_goal', 10)

        # Publicador ManualControl para control manual por teclado
        self.manual_control_pub = self.create_publisher(ManualControl, '/mavros/manual_control/send', 10)

        # --- Estado del teclado (Terminal) ---
        try:
            self.settings = termios.tcgetattr(sys.stdin)
        except termios.error:
            self.settings = None
            self.get_logger().error("No se pudo obtener el TTY de la terminal. Asegúrate de usar 'docker exec -it'")

        self.current_key = ''
        self.last_key_time = self.get_clock().now()
        self._keys_lock = threading.Lock()

        # Valores normalizados ManualControl (-1000 a 1000)
        self.THROTTLE_VAL = 150.0
        self.STEER_VAL    = 200.0

        # Timer de publicación RC (10 Hz)
        self.rc_timer = self.create_timer(0.1, self._rc_publish_loop)

        # Lanzar el lector de terminal en un hilo separado si el TTY es válido
        if self.settings is not None:
            self.kb_thread = threading.Thread(target=self._read_keyboard_loop, daemon=True)
            self.kb_thread.start()

        self.get_logger().info("QgcMissionBridge iniciado. Control por terminal activo (W/A/S/D) — Usa ESPACIO para frenar.")

    # -------------------------
    # MISIÓN
    # -------------------------
    def mission_callback(self, msg):
        self.waypoints = msg.waypoints
        self.last_reached = 0
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f"WP[{i}] lat:{wp.x_lat} lon:{wp.y_long}")
        
        if self.position is not None:
            self.publish_next_goal()
            self.get_logger().info(f"Misión cargada con {len(self.waypoints)} waypoints.")
        else:
            self.get_logger().warn("Misión cargada pero sin GPS aún, esperando...")

    def reached_callback(self, msg):
        self.last_reached = msg.wp_seq
        self.get_logger().info(
            f"Waypoint alcanzado {msg.wp_seq} → siguiente waypoint[{msg.wp_seq + 1}] de {len(self.waypoints)} totales"
        )
        self.publish_next_goal()

    def publish_next_goal(self):
        if not self.waypoints:
            return

        next_wp = self.last_reached + 1
        if next_wp >= len(self.waypoints):
            self.get_logger().info("Misión completada.")
            return

        wp = self.waypoints[next_wp]

        goal = TwistStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.twist.linear.x = wp.x_lat
        goal.twist.linear.y = wp.y_long
        goal.twist.linear.z = wp.z_alt

        self.goal_pub.publish(goal)
        self.get_logger().info(f"Siguiente objetivo → WP {next_wp}  lat:{wp.x_lat}  lon:{wp.y_long}")

    # -------------------------
    # ESTADO UGV
    # -------------------------
    def gps_callback(self, msg):
        self.position = msg
        self.get_logger().info(f"GPS UGV → lat:{msg.latitude}, lon:{msg.longitude}", throttle_duration_sec=5.0)

    def imu_callback(self, msg):
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.get_logger().info(
            f"Roll:{math.degrees(roll):.1f}°  Pitch:{math.degrees(pitch):.1f}°  Yaw:{math.degrees(yaw):.1f}°",
            throttle_duration_sec=5.0
        )

    def rc_callback(self, msg):
        ch1 = msg.channels[0]
        ch2 = msg.channels[1]
        self.get_logger().info(f"RC → ch1:{ch1} ch2:{ch2}", throttle_duration_sec=5.0)

    def vel_callback(self, msg):
        self.get_logger().info(f"VEL → {msg.twist.linear.x:.2f} m/s", throttle_duration_sec=5.0)

    # -------------------------
    # CONTROL MANUAL POR TERMINAL (HILO)
    # -------------------------
    def _read_keyboard_loop(self):
        """ Lee continuamente caracteres individuales desde la STDIN de la terminal """
        while rclpy.ok():
            try:
                # Cambiar terminal a modo RAW (captura el caracter inmediatamente sin esperar al Enter)
                tty.setraw(sys.stdin.fileno())
                # Esperar un máximo de 0.1 segundos a que ingrese un dato
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                
                if rlist:
                    key = sys.stdin.read(1).lower()
                else:
                    key = ''
            except Exception:
                key = ''
            finally:
                # Retornar la terminal a su modo original para no romper el comportamiento normal de linux
                if self.settings:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

            if key in ('w', 'a', 's', 'd', ' '):
                with self._keys_lock:
                    self.current_key = key
                    self.last_key_time = self.get_clock().now()

    def _rc_publish_loop(self):
        now = self.get_clock().now()

        with self._keys_lock:
            if (now - self.last_key_time).nanoseconds / 1e9 > 0.5:
                self.current_key = ''
            key = self.current_key

        if not key:
            return

        x = 0.0   # throttle
        y = 0.0   # steering

        if key == 'w':
            x = self.THROTTLE_VAL
        elif key == 's':
            x = -self.THROTTLE_VAL
        elif key == 'a':
            y = -self.STEER_VAL
        elif key == 'd':
            y = self.STEER_VAL
        # espacio: x=0, y=0 → parada

        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x = 0.0
        msg.y = y
        msg.z = x
        msg.r = 0.0
        self.manual_control_pub.publish(msg)

        self.get_logger().info(
            f"ManualControl | Tecla: {key.upper()} | x(throttle): {x} | y(steer): {y}",
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = QgcMissionBridge()
    try:
        rclpy.spin(node)
    finally:
        if node.settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


if __name__ == '__main__':
    main()
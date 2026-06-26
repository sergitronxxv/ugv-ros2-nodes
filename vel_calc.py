import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import WaypointList, WaypointReached, OverrideRCIn
from mavros_msgs.srv import WaypointPull, SetMode, WaypointClear

import math

# Ganancias PI Velocidad Lineal
Kp = 0.5
Ki = 0.05

# Ganancia velocidad angular
K_yaw = 0.25

# Límite anti-windup
INTEGRAL_MAX = 20.0

VX_MAX = 2.0   # m/s máximo
WZ_MAX = 2.0   # rad/s máximo

# Umbral de llegada al waypoint
ARRIVAL_DIST = 3.0  # metros

# Tolerancia para considerar dos waypoints "iguales" en coordenadas.
# MAVLink/MAVROS pueden reenviar la misma misión con micro-diferencias de
# redondeo (conversión int<->float), así que comparar con != es demasiado
# estricto. ~1e-7 grados ya equivale a ~1 cm en el ecuador, más que suficiente.
COORD_TOLERANCE = 1e-7

# Compara si el waypoint home cambia entre misiones
HOME_WP_IN_COMPARISON = False

# Si el error de yaw hacia el siguiente waypoint supera este ángulo,
# no tiene sentido avanzar (iríamos casi de lado o de espaldas al objetivo):
# se gira sobre el propio eje (solo steering) hasta encarar mejor el WP.
MAX_YAW_ERROR_FOR_THROTTLE = math.radians(30.0)

# Tasa máxima de frenado al cortar el throttle (m/s por segundo).
# En vez de pasar vx a 0.0 de golpe -algo que la inercia del rover no
# puede seguir, y que provoca que siga desplazándose mientras ya gira o
# mientras ya debería estar parado-, se reduce vx progresivamente a esta
# tasa en cada ciclo de control hasta llegar a 0.
VX_DECEL_RATE = 3.0  # m/s^2

# Tasa de rampa ascendente para aceleración limpia.
VX_ACCEL_RATE = 2.0  # m/s^2

# --- Fusión de YAW: SOLO GPS COURSE + GIROSCOPIO RELATIVO ---
# Ya no se usa el yaw absoluto de la IMU/brújula como referencia.
# - Por encima de GPS_SPEED_THRESHOLD: yaw = rumbo GPS (course over ground),
#   filtrado con EMA (gps_yaw_filtered).
# - Por debajo del umbral (parado o girando sobre el eje, sin curso GPS
#   fiable): se PROPAGA el último yaw GPS bueno integrando la velocidad
#   angular del giroscopio (yaw_rate), que sí es fiable en reposo.
#   Esto evita tanto "saltos" (congelar y luego saltar) como depender de
#   una brújula con problemas.

# Velocidad mínima de avance durante el giro de corrección de rumbo.
# En vez de pivotar puro (vx=0), avanzamos un poco para generar SIEMPRE
# curso GPS válido y no depender de la integración por giroscopio más
# tiempo del estrictamente necesario.
VX_MIN_GIRO = 0.1  # m/s

# Si el yaw lleva más de este tiempo propagándose solo por giroscopio
# (sin confirmación de GPS), se considera "no fiable" y se avisa/limita
# la velocidad angular para no fiarse de una deriva acumulada.
MAX_TIEMPO_SIN_GPS_YAW = 4.0  # s


class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        self.waypoints        = []
        self.last_reached     = 0
        self.next_wp          = 0
        self.position         = None
        self.initial_pull_done = False
        self.mision           = False

        self.integral  = 0.0
        self.last_time = self.get_clock().now()

        # Último vx realmente publicado. Se usa para poder aplicar una
        # rampa de frenado (VX_DECEL_RATE) en vez de cortar a 0 de golpe.
        self.vx_actual = 0.0

        # --- Variables para estimación desde GPS ---
        self.prev_lat = None
        self.prev_lon = None
        self.prev_time = None
        self.last_gps_yaw = None
        self.gps_yaw_filtered = None
        self.gps_wz_filtered = 0.0
        self.gps_speed = 0.0

        # Yaw "de trabajo": se actualiza con curso GPS cuando hay velocidad
        # suficiente, y se propaga integrando yaw_rate (giroscopio) cuando
        # no la hay. Es la ÚNICA referencia de orientación absoluta que usa
        # el control_loop; la IMU ya no aporta yaw absoluto, solo yaw_rate.
        self.yaw_estimado = None
        self.tiempo_sin_gps_yaw = 0.0

        # Parámetros de filtrado/fusión
        self.GPS_SPEED_THRESHOLD = 0.3
        self.GPS_EMA_ALPHA = 0.3
        self.GPS_WZ_BETA = 0.5
        self.COMPLEMENTARY_ALPHA = 0.95

        # Autoarranque
        self.auto_start_enabled = True
        self.auto_start_vx = 0.3
        self.auto_start_timeout = 5.0
        self.auto_start_start_time = None

        qos = qos_profile_sensor_data
        qos_mission = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Suscripciones
        self.create_subscription(WaypointList, '/mavros/mission/waypoints', self.mission_callback, qos_mission)
        # self.create_subscription(WaypointReached, '/mavros/mission/reached', self.reached_callback, qos_mission)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos)
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, qos)

        # Publicadores
        self.cmd_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos)
        # self.rc_pub  = self.create_publisher(OverrideRCIn, '/mavros/rc/override', qos)

        # Cliente para cambiar modo
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Cliente para pull de misión
        self.pull_client = self.create_client(WaypointPull, '/mavros/mission/pull')

        # Cliente para borrar misión anterior al arranque
        self.clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Petición periódica de misión (Pull)
        self._pull_in_progress = False
        self.pull_timer = self.create_timer(2.0, self._periodic_pull)

        self.yaw_rate = 0.0
        self.yaw_rate_from_imu = 0.0

        # Borrar misión anterior al arrancar (no bloqueante, con reintento)
        self._clear_done = False
        self.create_timer(1.0, self._initial_clear)

        self.get_logger().info("VelocityController iniciado. Esperando misión nueva.")

    # -------------------------
    # ARRANQUE: BORRAR MISIÓN
    # -------------------------
    def _initial_clear(self):
        if self._clear_done:
            return  # ya ejecutado, el timer se cancela a sí mismo

        if not self.clear_client.service_is_ready():
            self.get_logger().warn("WaypointClear no listo aún, reintentando...")
            return

        future = self.clear_client.call_async(WaypointClear.Request())
        future.add_done_callback(self._clear_done_cb)
        self._clear_done = True  # no reintentar aunque el callback tarde

    def _clear_done_cb(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Misión anterior borrada. Listo para recibir misión nueva.")
            else:
                self.get_logger().warn("WaypointClear respondió sin éxito.")
        except Exception as e:
            self.get_logger().error(f"Error en WaypointClear: {e}")

    def _periodic_pull(self):
        if self.mision or not self._clear_done or self._pull_in_progress:
            return

        if not self.pull_client.service_is_ready():
            return

        self._pull_in_progress = True
        self.get_logger().info("Buscando misión en la FCU (Pull)...")
        future = self.pull_client.call_async(WaypointPull.Request())
        future.add_done_callback(self._pull_done_cb)

    def _pull_done_cb(self, future):
        self._pull_in_progress = False
        try:
            result = future.result()
            if result.wp_received > 0:
                self.get_logger().info(f"Pull exitoso: la FCU tiene {result.wp_received} waypoints.")
        except Exception as e:
            self.get_logger().error(f"Error en WaypointPull: {e}")

    # -------------------------
    # CAMBIO DE MODO
    # -------------------------
    def set_mode_guided(self):
        if not self.mode_client.service_is_ready():
            self.get_logger().warn("SetMode no listo")
            return
        req = SetMode.Request()
        req.custom_mode = "GUIDED"
        future = self.mode_client.call_async(req)
        future.add_done_callback(self._set_mode_done_cb)
        self.get_logger().warn("CAMBIO DE MODO → GUIDED (enviado, esperando confirmación)")

    def set_mode_manual(self):
        if not self.mode_client.service_is_ready():
            self.get_logger().warn("SetMode no listo")
            return
        req = SetMode.Request()
        req.custom_mode = "MANUAL"
        future = self.mode_client.call_async(req)
        future.add_done_callback(self._set_mode_manual_done_cb)
        self.get_logger().warn("CAMBIO DE MODO → MANUAL (enviado, esperando confirmación)")

    def _set_mode_manual_done_cb(self, future):
        try:
            result = future.result()
            if result.mode_sent:
                self.get_logger().warn("CAMBIO DE MODO → MANUAL confirmado por FCU")
            else:
                self.get_logger().error("FCU RECHAZÓ el cambio a MANUAL (mode_sent=False)")
        except Exception as e:
            self.get_logger().error(f"Error en SetMode: {e}")

    def _set_mode_done_cb(self, future):
        try:
            result = future.result()
            if result.mode_sent:
                self.get_logger().warn("CAMBIO DE MODO → GUIDED confirmado por FCU")
            else:
                self.get_logger().error("FCU RECHAZÓ el cambio a GUIDED (mode_sent=False)")
        except Exception as e:
            self.get_logger().error(f"Error en SetMode: {e}")

    # -------------------------
    # COMPARACIÓN DE MISIONES
    # -------------------------
    def _waypoints_equal(self, wp_a, wp_b):
        """Compara dos waypoints individuales con tolerancia numérica."""
        return (
            abs(wp_a.x_lat - wp_b.x_lat) <= COORD_TOLERANCE and
            abs(wp_a.y_long - wp_b.y_long) <= COORD_TOLERANCE
        )

    def _is_same_mission(self, waypoints_nuevos):
        """
        Devuelve True si waypoints_nuevos representa la misma misión que
        self.waypoints (misma cantidad de WPs y mismas coordenadas dentro
        de la tolerancia definida). Permite ignorar el WP 0 (home) si así
        se configura, porque la FCU puede reenviarlo con leves variaciones
        aunque la ruta real no haya cambiado.
        """
        if len(self.waypoints) != len(waypoints_nuevos):
            return False

        viejos = self.waypoints
        nuevos = waypoints_nuevos

        if not HOME_WP_IN_COMPARISON and len(viejos) > 0:
            viejos = viejos[1:]
            nuevos = nuevos[1:]

        for wp_viejo, wp_nuevo in zip(viejos, nuevos):
            if not self._waypoints_equal(wp_viejo, wp_nuevo):
                return False

        return True

    # -------------------------
    # CALLBACKS
    # -------------------------
    def mission_callback(self, msg):
        if len(msg.waypoints) == 0:
            self.get_logger().info("Lista de waypoints vacía recibida, ignorando.")
            return

        if self._is_same_mission(msg.waypoints):
            # Si MAVROS nos vuelve a enviar la misión que ya estamos ejecutando,
            # la ignoramos para no resetear el progreso ni forzar el modo GUIDED.
            self.get_logger().info("Misión recibida es igual a la actual (coords dentro de tolerancia), ignorando.")
            return

        # Si llegamos aquí, es una misión NUEVA o MODIFICADA desde Mission Planner
        self.waypoints = msg.waypoints
        self.last_reached = 0
        self.next_wp = 0
        self.integral = 0.0
        self.mision = True

        self.get_logger().info(f"NUEVA Misión cargada: {len(self.waypoints)} waypoints. Iniciando ruta.")
        self.set_mode_guided()

    def gps_callback(self, msg):
        now = self.get_clock().now()
        lat = msg.latitude
        lon = msg.longitude

        if self.prev_lat is None:
            self.prev_lat = lat
            self.prev_lon = lon
            self.prev_time = now
            self.position = msg
            self.get_logger().info(f"GPS: primer fix recibido lat={lat:.6f} lon={lon:.6f}")
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            return

        dlat = lat - self.prev_lat
        dlon = lon - self.prev_lon
        dx = dlon * 111320.0 * math.cos(math.radians(lat))
        dy = dlat * 110540.0
        dist = math.hypot(dx, dy)
        self.gps_speed = dist / dt

        if self.gps_speed > self.GPS_SPEED_THRESHOLD:
            yaw_gps = math.atan2(dy, dx)
            if self.gps_yaw_filtered is None:
                self.gps_yaw_filtered = yaw_gps
            else:
                dtheta = math.atan2(math.sin(yaw_gps - self.gps_yaw_filtered),
                                    math.cos(yaw_gps - self.gps_yaw_filtered))
                self.gps_yaw_filtered += self.GPS_EMA_ALPHA * dtheta
                self.gps_yaw_filtered = math.atan2(math.sin(self.gps_yaw_filtered),
                                                    math.cos(self.gps_yaw_filtered))

            if self.last_gps_yaw is not None:
                yaw_diff = math.atan2(math.sin(self.gps_yaw_filtered - self.last_gps_yaw),
                                    math.cos(self.gps_yaw_filtered - self.last_gps_yaw))
                wz_gps = yaw_diff / dt
            else:
                wz_gps = 0.0

            self.gps_wz_filtered = self.GPS_WZ_BETA * self.gps_wz_filtered + (1 - self.GPS_WZ_BETA) * wz_gps
            self.last_gps_yaw = self.gps_yaw_filtered

            # Curso GPS fiable: esta es la ÚNICA fuente de yaw absoluto.
            # Resincronizamos yaw_estimado con ella y reseteamos el
            # contador de "tiempo sin confirmación GPS".
            self.yaw_estimado = self.gps_yaw_filtered
            self.tiempo_sin_gps_yaw = 0.0

            self.get_logger().info(
                f"GPS: speed={self.gps_speed:.2f}m/s yaw_filt={math.degrees(self.gps_yaw_filtered):.1f}° "
                f"wz_gps={wz_gps:.3f} dt={dt:.3f}"
            )
        else:
            self.gps_wz_filtered *= 0.9
            # Por debajo del umbral el curso GPS es ruido, no lo usamos.
            # gps_yaw_filtered y last_gps_yaw se invalidan para que, al
            # recuperar velocidad, NO se produzca un salto con un valor
            # viejo: se reconstruyen desde cero con el primer curso fiable.
            self.gps_yaw_filtered = None
            self.last_gps_yaw = None
            self.get_logger().info(f"GPS: speed={self.gps_speed:.2f}m/s (bajo umbral, sin actualizar yaw)")

        self.prev_lat = lat
        self.prev_lon = lon
        self.prev_time = now
        self.position = msg

    def imu_callback(self, msg):
        gyro_wz = msg.angular_velocity.z
        fused_wz = self.COMPLEMENTARY_ALPHA * gyro_wz + (1 - self.COMPLEMENTARY_ALPHA) * self.gps_wz_filtered
        self.yaw_rate = max(-WZ_MAX, min(WZ_MAX, fused_wz))

        # NOTA: ya NO se extrae yaw absoluto de la orientación de la IMU
        # (msg.orientation), porque depende de la brújula y es justo la
        # fuente de los saltos/errores que queremos evitar. La IMU solo
        # aporta yaw_rate (velocidad angular), que es fiable en reposo.

        now = self.get_clock().now()
        dt_imu = 0.0
        if hasattr(self, '_last_imu_time'):
            dt_imu = (now - self._last_imu_time).nanoseconds / 1e9
        self._last_imu_time = now

        if self.yaw_estimado is not None and dt_imu > 0:
            if self.gps_speed <= self.GPS_SPEED_THRESHOLD:
                # Sin curso GPS fiable: propagamos el último yaw bueno
                # integrando la velocidad angular medida (gyro+GPS fusionados).
                self.yaw_estimado = math.atan2(
                    math.sin(self.yaw_estimado + self.yaw_rate * dt_imu),
                    math.cos(self.yaw_estimado + self.yaw_rate * dt_imu)
                )
                self.tiempo_sin_gps_yaw += dt_imu
            # Si hay velocidad suficiente, gps_callback ya se encarga de
            # resincronizar yaw_estimado directamente con el curso GPS;
            # aquí no hace falta tocarlo (evita pisarlo dos veces por ciclo).

        # Log limitado a ~2 Hz para no inundar el journal
        if not hasattr(self, '_last_imu_log') or (now - self._last_imu_log).nanoseconds / 1e9 > 0.5:
            self._last_imu_log = now
            yaw_txt = f"{math.degrees(self.yaw_estimado):.1f}°" if self.yaw_estimado is not None else "N/A"
            self.get_logger().info(
                f"IMU: gyro_wz={gyro_wz:.3f} yaw_rate_fused={self.yaw_rate:.3f} "
                f"yaw_estimado={yaw_txt} t_sin_gps={self.tiempo_sin_gps_yaw:.1f}s"
            )

    # -------------------------
    # CONTROL LOOP
    # -------------------------
    def control_loop(self):
        if self.position is None:
            return

        if not self.mision:
            self.publish_velocity(0.0, 0.0)
            return

        # --- ARRANQUE EN FRÍO: aún no tenemos ningún yaw de referencia ---
        # (rover parado desde el boot: sin curso GPS válido todavía, y ya
        # no usamos yaw absoluto de la brújula). Avanzamos recto a baja
        # velocidad hasta conseguir el primer curso GPS fiable; sin esto
        # nunca se podría decidir hacia dónde girar.
        if self.yaw_estimado is None:
            if self.auto_start_start_time is None:
                self.auto_start_start_time = self.get_clock().now()
                self.get_logger().info(
                    "Sin yaw de referencia aún (arranque en frío). "
                    "Avanzando recto a baja velocidad para obtener curso GPS..."
                )

            elapsed = (self.get_clock().now() - self.auto_start_start_time).nanoseconds / 1e9
            if elapsed > self.auto_start_timeout:
                self.get_logger().warn(
                    "Timeout esperando curso GPS fiable en arranque en frío. "
                    "Reintentando (revisa fix GPS / RTK)."
                )
                self.auto_start_start_time = None  # reintenta el ciclo

            self.publish_velocity(self.auto_start_vx, 0.0)
            return

        self.auto_start_start_time = None  # ya tenemos yaw, no hace falta más

        self.next_wp = self.last_reached + 1
        if self.next_wp >= len(self.waypoints):
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now
            if dt > 0:
                vx_frenado = self._ramped_vx(0.0, dt)
            else:
                vx_frenado = self.vx_actual
            self.publish_velocity(vx_frenado, 0.0)

            if vx_frenado != 0.0:
                # Aún frenando: no cambiamos de modo todavía, para no dejar
                # el rover deslizando en MANUAL sin control de velocidad.
                return

            self.set_mode_manual()
            self.mision = False
            self.get_logger().info("Misión finalizada. Cambiando a MANUAL y borrando misión de la FCU.")

            # --- Limpiar la FCU y la memoria local ---
            if self.clear_client.service_is_ready():
                self.clear_client.call_async(WaypointClear.Request())
            self.waypoints = [] # Vaciamos la lista local

            return

        wp = self.waypoints[self.next_wp]

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0:
            return

        dlat = wp.x_lat - self.position.latitude
        dlon = wp.y_long - self.position.longitude
        dx = dlon * 111320.0 * math.cos(math.radians(self.position.latitude))
        dy = dlat * 110540.0
        dist_error = math.hypot(dx, dy)

        if dist_error < ARRIVAL_DIST:
            vx_frenado = self._ramped_vx(0.0, dt)
            self.publish_velocity(vx_frenado, 0.0)
            self.get_logger().info(f"WP {self.next_wp} alcanzado por distancia.")
            self.last_reached = self.next_wp
            self.integral = 0.0
            return

        # --- Orientación deseada hacia el waypoint (se calcula ANTES del
        # throttle, porque decide si avanzar a fondo o solo corregir rumbo) ---
        yaw_des = math.atan2(dy, dx)
        yaw_error = math.atan2(math.sin(yaw_des - self.yaw_estimado),
                                math.cos(yaw_des - self.yaw_estimado))

        if self.tiempo_sin_gps_yaw > MAX_TIEMPO_SIN_GPS_YAW:
            self.get_logger().warn(
                f"Yaw propagado solo por giroscopio desde hace "
                f"{self.tiempo_sin_gps_yaw:.1f}s (sin confirmación GPS). "
                f"Puede haber deriva acumulada."
            )

        # --- Decidir si "merece la pena" meter throttle a fondo ---
        # Si el WP está muy desalineado (p.ej. a 180° "en el culo"), avanzar
        # a tope solo nos alejaría del objetivo. En ese caso corregimos
        # rumbo con un AVANCE MÍNIMO (no pivote puro a vx=0): así seguimos
        # generando curso GPS válido durante el giro y evitamos depender
        # de la integración por giroscopio más tiempo del necesario, que
        # es justo lo que producía los saltos/movimientos erráticos.
        if abs(yaw_error) > MAX_YAW_ERROR_FOR_THROTTLE:
            vx = self._ramped_vx(VX_MIN_GIRO, dt)

            wz = K_yaw * yaw_error - 0.1 * self.yaw_rate
            wz = max(-WZ_MAX, min(WZ_MAX, wz))
            self.get_logger().info(
                f"Corrigiendo rumbo con avance mínimo. Error: {math.degrees(yaw_error):.1f}°"
            )
            # No acumulamos integral de distancia mientras solo corregimos
            # rumbo, para que al encarar el WP no salga un golpe de vx por
            # integral ya cargada.
        else:
            self.integral += dist_error * dt
            self.integral = min(self.integral, INTEGRAL_MAX)
            vx_deseado = min(Kp * dist_error + Ki * self.integral, VX_MAX)
            vx = self._ramped_vx(vx_deseado, dt)

            wz = K_yaw * yaw_error - 0.1 * self.yaw_rate
            wz = max(-WZ_MAX, min(WZ_MAX, wz))

        self.get_logger().info(
            f"yaw_est:{math.degrees(self.yaw_estimado):.1f}° "
            f"yaw_des:{math.degrees(yaw_des):.1f}° "
            f"yaw_error:{math.degrees(yaw_error):.1f}° "
            f"vx:{vx:.2f} wz:{wz:.3f} t_sin_gps:{self.tiempo_sin_gps_yaw:.1f}s"
        )

        self.publish_velocity(vx, wz)

    # -------------------------
    # PUBLICADORES / RAMPA
    # -------------------------
    def _ramped_vx(self, vx_deseado, dt):
        """
        Aplica una rampa progresiva tanto para acelerar como para frenar,
        evitando cambios bruscos que la inercia del UGV no pueda seguir.
        """
        if vx_deseado > self.vx_actual:
            # Rampa ascendente (Aceleración)
            max_subida = VX_ACCEL_RATE * dt
            self.vx_actual = min(vx_deseado, self.vx_actual + max_subida)
        elif vx_deseado < self.vx_actual:
            # Rampa descendente (Frenado)
            max_caida = VX_DECEL_RATE * dt
            self.vx_actual = max(vx_deseado, self.vx_actual - max_caida)
        
        return self.vx_actual

    def publish_velocity(self, vx, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = vx
        msg.twist.angular.z = wz
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


if __name__ == '__main__':
    main()
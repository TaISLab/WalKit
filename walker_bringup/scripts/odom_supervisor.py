#!/usr/bin/env python3
"""Supervisor del gyro para la odometria laser-inercial: conmuta las covarianzas que ve el EKF.

Por que existe. Con el prior del gyro en el ICP (ros2_laser_scan_matcher) el EKF se construye alrededor del gyro: la
covarianza de rumbo del laser es deliberadamente holgada (0.2 rad2, walker_lsm.yaml) y el rumbo lo da el gyro, con el
laser como ancla lenta. Eso da 1.9 deg de rumbo en los bags limpios... y ~4.5 deg si el gyro desaparece, porque
robot_localization no sabe conmutar covarianzas. Este nodo lo hace desde fuera:

    /scan_odom        --(rumbo del laser: covarianza fuerte si el gyro esta mal)-->  /scan_odom_ekf
    /bwt901cl/imu     --(varianza del gyro inflada si esta mal)------------------>  /imu_ekf

y el EKF se alimenta de /scan_odom_ekf y /imu_ekf (lo cablea walker_laser_imu_odom.launch.py).

Que vigila (solo fallos INTRINSECOS del gyro, a proposito):
  - sin mensajes mas de imu_timeout, o sin TF base_frame <- frame del IMU (robot_localization tampoco lo usaria);
  - tasa no plausible (|wz| > imu_max_rate) o NaN: gyro saturado o corrupto;
  - giro del laser >= turn_min_deg en la ventana y el gyro dice lo contrario (signo) o casi nada (muerto/congelado):
    signo invertido por TF/montaje mal declarado, gyro atascado;
  - sesgo: en tramos casi rectos (el laser gira < straight_yaw_deg en la ventana, donde es fiable) el gyro integra
    mas de bias_max rad/s de diferencia con el laser.
Los fallos que solo se ven en ciertas maniobras (signo: solo en giros; sesgo: solo en rectas) quedan LATCHED: la
ausencia de bandera no es evidencia de que el gyro este bien (en recta un gyro con el signo cambiado marca ~0 igual
que uno bueno). Solo se limpian con evidencia positiva: un giro donde el gyro concuerda, o una recta donde coincide.
Un simple DESACUERDO laser-gyro NO conmuta: en los bags donde el scan matching deriva en rumbo, el gyro tiene razon
y el laser no; creer al laser ahi empeoraria justo lo que el prior del gyro arregla. (Si el que falla es el laser,
quien arbitra es el propio scan matcher: ICP sin semilla frente al sembrado.)

Salida de estado: ~/status (diagnostic_msgs/DiagnosticStatus), OK o WARN con el motivo, y un log en cada cambio.
"""
import collections
import math

import rclpy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rotate(q, v):
    """v rotado por el cuaternion q (x, y, z, w)."""
    x, y, z, w = q
    # t = 2 * cross(q.xyz, v); v' = v + w*t + cross(q.xyz, t)
    tx = 2 * (y * v[2] - z * v[1])
    ty = 2 * (z * v[0] - x * v[2])
    tz = 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx))


class OdomSupervisor(Node):
    def __init__(self):
        super().__init__("odom_supervisor")
        d = self.declare_parameter
        self.scan_in = d("scan_in", "/scan_odom").value
        self.scan_out = d("scan_out", "/scan_odom_ekf").value
        self.imu_in = d("imu_in", "/bwt901cl/imu").value
        self.imu_out = d("imu_out", "/imu_ekf").value
        self.base_frame = d("base_frame", "base_footprint").value
        self.heading_var_degraded = d("heading_var_degraded", 0.0012).value   # rad2: el laser manda
        self.imu_var_degraded = d("imu_var_degraded", 1.0).value               # (rad/s)2: el EKF lo ignora
        self.imu_timeout = d("imu_timeout", 1.0).value
        self.imu_max_rate = d("imu_max_rate", 10.0).value
        self.window_s = d("window_s", 8.0).value
        self.turn_min = math.radians(d("turn_min_deg", 45.0).value)
        self.dead_ratio = d("dead_ratio", 0.3).value
        self.stationary_xy = d("stationary_xy", 0.05).value
        self.stationary_yaw = math.radians(d("stationary_yaw_deg", 2.0).value)
        self.bias_max = d("bias_max", 0.02).value
        self.straight_yaw = math.radians(d("straight_yaw_deg", 10.0).value)
        self.persist_s = d("persist_s", 2.0).value
        self.startup_grace_s = d("startup_grace_s", 10.0).value
        self.recover_s = d("recover_s", 5.0).value

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)
        self.q_base_imu = None
        self.imu_buf = collections.deque()          # (t, wz en base_frame)
        self.last_imu_t = None
        self.implausible_until = -1.0
        self.prev = None                            # (t, x, y, yaw) del scan anterior
        self.window = collections.deque()           # (t, dyaw_laser, dyaw_gyro|None, dpos)
        self.bad_since = None                       # primera vez que la ventana sale mala con el motivo actual
        self.bad_reason = ""
        self.latch = ""                             # motivo pegajoso (ver docstring)
        self.t_start = None
        self.t_prev = 0.0
        self.ok_since = None
        self.state = "STARTING"                     # el EKF ve covarianzas de "gyro degradado" hasta tener IMU+TF
        self.reason = "esperando al IMU"
        self.stats = {}

        self.pub_s = self.create_publisher(Odometry, self.scan_out, 10)
        self.pub_i = self.create_publisher(Imu, self.imu_out, 50)
        self.pub_status = self.create_publisher(DiagnosticStatus, "~/status", 10)
        self.create_subscription(Odometry, self.scan_in, self.on_scan, 10)
        self.create_subscription(Imu, self.imu_in, self.on_imu, qos_profile_sensor_data)
        self.create_timer(0.1, self.on_timer)
        self.create_timer(1.0, self.publish_status)

    # ---------------------------------------------------------------- tiempo
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------- IMU
    def on_imu(self, m):
        if self.q_base_imu is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, m.header.frame_id, rclpy.time.Time())
                r = tf.transform.rotation
                self.q_base_imu = (r.x, r.y, r.z, r.w)
            except Exception:
                self.relay_imu(m, degraded=True)
                return
        w = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        wz = rotate(self.q_base_imu, w)[2]
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.last_imu_t = self.now_s()
        if not math.isfinite(wz) or abs(wz) > self.imu_max_rate:
            self.implausible_until = self.now_s() + 1.0
        else:
            self.imu_buf.append((t, wz))
            while self.imu_buf and self.imu_buf[0][0] < t - 3 * self.window_s:
                self.imu_buf.popleft()
        self.relay_imu(m, self.state not in ("OK",))

    def relay_imu(self, m, degraded):
        if degraded:
            for k in (0, 4, 8):
                m.angular_velocity_covariance[k] = max(m.angular_velocity_covariance[k], self.imu_var_degraded)
        self.pub_i.publish(m)

    def integrate(self, t0, t1):
        """Integral del yaw rate entre t0 y t1 (regla del trapecio); None si el IMU no cubre el intervalo."""
        b = self.imu_buf
        if len(b) < 2 or t1 <= t0 or b[0][0] > t0 + 0.1 or b[-1][0] < t1 - 0.1:
            return None
        acc = covered = 0.0
        for (ta, wa), (tb, wb) in zip(b, list(b)[1:]):
            if tb <= t0 or ta >= t1:
                continue
            if tb - ta > 0.1:
                return None
            lo, hi = max(ta, t0), min(tb, t1)
            span = tb - ta
            fa = wa + (wb - wa) * (lo - ta) / span if span > 1e-9 else wa
            fb = wa + (wb - wa) * (hi - ta) / span if span > 1e-9 else wa
            acc += 0.5 * (fa + fb) * (hi - lo)
            covered += hi - lo
        return acc + 0.5 * (b[0][1] + b[-1][1]) * max(0.0, (t1 - t0) - covered)

    # ------------------------------------------------------------------ scan
    def on_scan(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        yaw = yaw_of(m.pose.pose.orientation)
        if self.prev is not None and t > self.prev[0]:
            dl = wrap(yaw - self.prev[3])
            dg = self.integrate(self.prev[0], t)
            self.window.append((t, dl, dg, math.hypot(p.x - self.prev[1], p.y - self.prev[2])))
        while self.window and self.window[0][0] < t - self.window_s:
            self.window.popleft()
        self.prev = (t, p.x, p.y, yaw)
        self.evaluate_window(t)
        if self.state != "OK":
            m.pose.covariance[35] = min(m.pose.covariance[35], self.heading_var_degraded) \
                if m.pose.covariance[35] > 0 else self.heading_var_degraded
        self.pub_s.publish(m)

    def evaluate_window(self, t):
        valid = [w for w in self.window if w[2] is not None]
        if len(valid) < 5:
            return
        sl = sum(w[1] for w in valid)
        sg = sum(w[2] for w in valid)
        path = sum(w[3] for w in valid)
        span = valid[-1][0] - (valid[0][0] - 0.2)
        bad = ""
        good_turn = good_straight = False
        if abs(sl) >= self.turn_min:
            if sg * sl < 0 or abs(sg) < self.dead_ratio * abs(sl):
                bad = "gyro contradice al laser en un giro (signo/TF o gyro muerto)"
            else:
                good_turn = True
        elif abs(sl) < self.straight_yaw and span > 0.6 * self.window_s:
            drift = abs(sg - sl) / span
            if drift > self.bias_max:
                bad = f"sesgo del gyro ({math.degrees(sg - sl) / span:+.1f} deg/s respecto al laser)"
            elif drift < 0.5 * self.bias_max:
                good_straight = True
        self.stats = dict(laser_turn_deg=round(math.degrees(sl), 1), gyro_turn_deg=round(math.degrees(sg), 1),
                          path_m=round(path, 2), window_s=round(span, 1))
        if bad:
            if self.bad_since is None or bad[:6] != self.bad_reason[:6]:
                self.bad_since, self.bad_reason = t, bad
            if t - self.bad_since >= self.persist_s:
                self.latch = bad
        else:
            self.bad_since = None
            if good_turn and self.latch.startswith("gyro contradice"):
                self.latch = ""
            elif good_straight and self.latch.startswith("sesgo"):
                self.latch = ""

    # ----------------------------------------------------------- estado/timer
    def on_timer(self):
        now = self.now_s()
        if self.t_start is None or abs(now - self.t_prev) > 1.0:    # arranque, o salto del reloj (sim time)
            self.t_start = now
        self.t_prev = now
        reason = ""
        if self.q_base_imu is None and self.last_imu_t is None:
            reason = "sin IMU todavia"
        elif self.last_imu_t is None or now - self.last_imu_t > self.imu_timeout:
            reason = "IMU sin mensajes"
        elif now < self.implausible_until:
            reason = "gyro fuera de rango / NaN"
        elif self.latch:
            reason = self.latch
        if self.state == "STARTING":
            if not reason:
                self.state, self.reason = "OK", ""
                self.get_logger().info("gyro OK")
            elif now - self.t_start > self.startup_grace_s:
                self.state, self.reason = "DEGRADED", reason
                self.get_logger().warn(f"gyro DEGRADADO ({reason}) [t={now:.1f}]: el rumbo pasa a seguir al laser")
            return
        if reason:
            self.ok_since = None
            if self.state == "OK":
                self.state, self.reason = "DEGRADED", reason
                self.get_logger().warn(f"gyro DEGRADADO ({reason}) [t={now:.1f}]: el rumbo pasa a seguir al laser")
        elif self.state != "OK":
            if self.ok_since is None:
                self.ok_since = now
            elif now - self.ok_since >= self.recover_s:
                self.state, self.reason = "OK", ""
                self.get_logger().info(f"gyro recuperado [t={now:.1f}]: el rumbo vuelve a apoyarse en el gyro")
                self.window.clear()
                self.bad_since = None

    def publish_status(self):
        s = DiagnosticStatus()
        s.name = "odom_supervisor"
        s.hardware_id = "witmotion"
        s.level = DiagnosticStatus.OK if self.state in ("OK", "STARTING") else DiagnosticStatus.WARN
        s.message = "gyro OK" if self.state == "OK" else f"gyro degradado: {self.reason}"
        s.values = [KeyValue(key=k, value=str(v)) for k, v in self.stats.items()]
        self.pub_status.publish(s)


def main():
    rclpy.init()
    node = OdomSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

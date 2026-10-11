#!/usr/bin/env python3
"""Rele + colector del arnes de ajuste (lo lanza harness.py, no se usa a mano).

  /scan_odom_raw  -> /scan_odom   con covarianza y/o sello sobrescritos
  <imu_topic>     -> /imu_tuned   con varianza / sesgo / signo del gyro sobrescritos
  /odom (salida del EKF) -> CSV   (t_ns, x, y, yaw), volcado al recibir SIGINT
"""
import math

import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class Tune(Node):
    def __init__(self):
        super().__init__("tune_node")
        d = self.declare_parameter
        self.out = d("out", "/tmp/odom.csv").value
        self.pc = list(d("pose_cov", [0.0] * 6).value)
        self.tc = list(d("twist_cov", [0.0] * 6).value)
        self.override_scan = d("override_scan_cov", False).value
        self.scan_dt = d("scan_stamp_offset", 0.0).value
        self.gyro_var = d("gyro_var", 0.0).value        # >0: sobrescribe la varianza de wx, wy, wz
        self.gyro_bias = d("gyro_bias_z", 0.0).value
        self.gyro_sign = d("gyro_sign", 1.0).value
        imu_topic = d("imu_topic", "/bwt901cl/imu").value
        self.rows = []
        self.pub_s = self.create_publisher(Odometry, "/scan_odom", 10)
        self.pub_i = self.create_publisher(Imu, "/imu_tuned", 50)
        self.create_subscription(Odometry, "/scan_odom_raw", self.on_scan, 10)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)

    def on_scan(self, m):
        if self.override_scan:
            for i, k in enumerate((0, 7, 14, 21, 28, 35)):
                m.pose.covariance[k] = self.pc[i]
                m.twist.covariance[k] = self.tc[i]
        if self.scan_dt:
            ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec + int(self.scan_dt * 1e9)
            m.header.stamp = Time(sec=ns // 10**9, nanosec=ns % 10**9)
        self.pub_s.publish(m)

    def on_imu(self, m):
        m.angular_velocity.z = self.gyro_sign * m.angular_velocity.z - self.gyro_bias
        if self.gyro_var > 0:
            for k in (0, 4, 8):
                m.angular_velocity_covariance[k] = self.gyro_var
        self.pub_i.publish(m)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        self.rows.append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))

    def dump(self):
        with open(self.out, "w") as f:
            for r in self.rows:
                f.write("%d,%.6f,%.6f,%.6f\n" % r)


def main():
    rclpy.init()
    n = Tune()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.dump()
        try:
            rclpy.shutdown()
        except Exception:
            pass


main()

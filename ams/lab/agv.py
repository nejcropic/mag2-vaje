#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Odometrija iz dveh koles na SKUPNI KRMILJENI enoti in njenega kota.

x, y: sredina zadnje NEKRMILJENE osi; +x naprej, +y levo, +phi v levo.
Pred voznjo preveri nastavitve spodaj. Glej tudi navodila_agv.md.
"""

import math
import rclpy

from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_cmd, qos_sensor, pose_to_msg, get_frame_prefix
from amsagv.agvapi import Agv, find_line_edges
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from amsagv_msgs.msg import LineStamped


# Priblizek iz meritve 110.000-120.000 countov / m. Umeri vsako kolo posebej.
COUNTS_PER_METER = 115_000.0
WHEEL_COUNTS_PER_REV = 8192  # Modul vrnjenih 13-bitnih odcitkov koles.

HEADING_ZERO = 0  # zacetna vrednost enkoderja oz. tam kjer starta - je poravnan na nuli

D = 0.1207 # razdalja med osema


def normalize_angle(angle):
    """Kot v radianih v obmocju [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def heading_angle(counts):
    cpr = 16384
    # Absolutni kot glede na rocno izmerjeno niclo, brez sestevanja razlik.
    return normalize_angle((counts - HEADING_ZERO)* 2.0 * math.pi / cpr)


def wheel_delta(current, previous):
    cpr = 8192
    half = cpr // 2
    return (current - previous + half) % cpr - half


def odometry_step(x, y, phi, ds, gamma):
    # ds je povprecen premik koles SKUPNE KRMILJENE enote.
    dphi = ds * math.sin(gamma) / D
    ds_body = ds * math.cos(gamma)  # Premik sredine zadnje nekrmiljene osi.
    phi_mid = phi + dphi / 2.0
    x += ds_body * math.cos(phi_mid)
    y += ds_body * math.sin(phi_mid)
    return x, y, normalize_angle(phi + dphi), dphi


def main():
    with Agv() as robot:

        def handle_cmd_vel(msg):
            robot.set_vel(msg.linear.x, msg.angular.z)

        # Main loop, executed at 50 Hz
        def handle_timer():
            nonlocal x, y, phi, gamma, fd
            nonlocal left_prev, right_prev, gamma_prev
            nonlocal left_distance, right_distance, print_counter

            robot.read_sensors()
            enc_left, enc_right, enc_heading = robot.get_encoders()
            t = node.get_clock().now().to_msg()

            # Kot sprednje enote glede na rocno doloceno niclo, v radianih.
            gamma = heading_angle(enc_heading)
            gamma_mid = normalize_angle(
                gamma_prev + normalize_angle(gamma - gamma_prev) / 2.0
            )

            dl = wheel_delta(enc_left, left_prev) / COUNTS_PER_METER
            dr = wheel_delta(enc_right, right_prev) / COUNTS_PER_METER * -1 # mnozis z -1, ker sta kontra obrnjena
            ds = (dl + dr) / 2.0

            # Tricikel: premik merita kolesi skupne sprednje krmiljene enote.
            x, y, phi, _ = odometry_step(x, y, phi, ds, gamma_mid)
            fd += abs(ds)
            left_distance += dl
            right_distance += dr

            left_prev = enc_left
            right_prev = enc_right
            gamma_prev = gamma

            # Odometry message; prenos gamma v z ohranjen iz originala.
            msg_odom.header.stamp = t
            msg_odom.pose.pose = pose_to_msg(x, y, phi)
            msg_odom.pose.pose.position.z = gamma
            pub_odom.publish(msg_odom)

            # Line sensor
            line_values = robot.get_line_values()
            edge_left, edge_right = find_line_edges(line_values)

            msg_line.header.stamp = t
            msg_line.line.values = line_values
            msg_line.line.left = float(edge_left) if edge_left is not None else float('nan')
            msg_line.line.right = float(edge_right) if edge_right is not None else float('nan')
            msg_line.line.heading = gamma
            msg_line.line.distance = fd
            pub_line.publish(msg_line)

            # Izpis 5 Hz; odometrija se racuna 50 Hz, tudi med mirovanjem.
            print_counter += 1
            if print_counter >= 10:
                print_counter = 0
                print(
                    f'x={x:.3f} m | y={y:.3f} m | '
                    f'phi={math.degrees(phi):.2f} deg | '
                    f'alfa={math.degrees(gamma):.2f} deg | '
                    f'enc_heading={enc_heading} | '
                    f'levo={left_distance:.3f} m | '
                    f'desno={right_distance:.3f} m | '
                    f'razdalja={fd:.3f} m'
                )

        rclpy.init()
        node = Node('agv')

        try:
            prefix = get_frame_prefix(node)
            param_odom_frame_id = node.declare_parameter(
                'odom_frame_id', f'{prefix}odom'
            ).value
            param_agv_frame_id = node.declare_parameter(
                'agv_frame_id', f'{prefix}agv'
            ).value

            pub_odom = node.create_publisher(Odometry, 'odom', qos_sensor)
            pub_line = node.create_publisher(LineStamped, 'line', qos_sensor)
            sub_cmd_vel = node.create_subscription(
                Twist, 'cmd_vel', handle_cmd_vel, qos_cmd
            )

            msg_line = LineStamped()
            msg_odom = Odometry()
            msg_odom.header.frame_id = param_odom_frame_id
            msg_odom.child_frame_id = param_agv_frame_id

            # Odometry initial state
            x = y = phi = fd = 0.0
            left_distance = right_distance = 0.0
            print_counter = 0
            robot.read_sensors()
            left_prev, right_prev, enc_heading = robot.get_encoders()
            gamma = heading_angle(enc_heading)
            gamma_prev = gamma

            print(
                f'Zacetek: levi={left_prev}, desni={right_prev}, '
                f'enc_heading={enc_heading}, HEADING_ZERO={HEADING_ZERO}, '
                f'alfa={math.degrees(gamma):.2f} deg'
            )

            timer = node.create_timer(1.0 / 50.0, handle_timer)
            try:
                rclpy.spin(node)
            finally:
                robot.set_vel(0.0, 0.0)

        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()

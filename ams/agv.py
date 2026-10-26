#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.executors import ExternalShutdownException
from amsagv.ams import Node, qos_cmd, qos_sensor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from amsagv_msgs.msg import LineStamped
from amsagv.ams import pose_to_msg, get_frame_prefix
from amsagv.agvapi import Agv, find_line_edges
import math


COUNTS_PER_METER = 120_000
def main():
    with Agv() as robot:
        # Handle velocity commands
        def handle_cmd_vel(msg):
            robot.set_vel(msg.linear.x, msg.angular.z)

        def counts_to_rad(counts: int, cpr: int = 8192) -> float:
            return counts * 2 * math.pi / cpr

        def counts_to_meters(counts: int):
            return counts / COUNTS_PER_METER

        # Main loop, executed at 50 Hz
        def handle_timer():
            nonlocal x, y, phi, gamma, fd, left_prev, right_prev, central_start
            
            # robot specs
            dt = 0.02
            d = 0.0 # TODO: vpiši z modela
            t = node.get_clock().now().to_msg()

            # Read sensors
            robot.read_sensors()

            # Read encoders
            enc_left, enc_right, enc_heading = robot.get_encoders()           
            #print(f'Encoders: {enc_left=}, {enc_right=}, {enc_heading=}')

            # Odometry
            ### get positions of the wheels and heading in radians
            alpha = counts_to_rad(enc_heading - central_start) 
            alpha_deg = math.degrees(alpha)
            pos_left = counts_to_meters(enc_left - left_prev)
            pos_right = counts_to_meters(enc_right - right_prev)

            ### get distance
            v_s = (pos_left + pos_right) / (2 * dt) # m/s
            omega = (v_s / d) * math.sin(alpha) # rad/s
            x += v_s * math.cos(alpha) * math.cos(phi)
            y += v_s * math.cos(alpha) * math.sin(phi)
            phi += omega 

            print(f"Current position: ({x:.2f}, {y:.2f}, {alpha_deg:.2f})")

            # update previous encoder values
            left_prev = enc_left
            right_prev = enc_right

            # Odometry message
            msg_odom.header.stamp = t
            msg_odom.pose.pose = pose_to_msg(x, y, phi)
            msg_odom.pose.pose.position.z = gamma
            # Publish odometry message
            pub_odom.publish(msg_odom)

            #
            # Line sensor
            #

            # Line-sensor values
            line_values = robot.get_line_values()
            # Left and right line edge
            edge_left, edge_right = find_line_edges(line_values)

            # Line-sensor message
            msg_line.header.stamp = t
            msg_line.line.values = line_values
            msg_line.line.left = float(edge_left) if edge_left is not None else float('nan')
            msg_line.line.right = float(edge_right) if edge_right is not None else float('nan')
            msg_line.line.heading = gamma
            msg_line.line.distance = fd
            # Publish line-sensor message
            pub_line.publish(msg_line)



        rclpy.init()
        node = Node('agv')
        
        try:
            prefix = get_frame_prefix(node)
            # Name of the odometry frame
            param_odom_frame_id = node.declare_parameter('odom_frame_id', f'{prefix}odom').value
            # Name of the AGV frame
            param_agv_frame_id = node.declare_parameter('agv_frame_id', f'{prefix}agv').value

            # Odometry publisher
            pub_odom = node.create_publisher(Odometry, 'odom', qos_sensor)
            # Line sensor publisher
            pub_line = node.create_publisher(LineStamped, 'line', qos_sensor)
            # Velocity commands subscriber.
            sub_cmd_vel = node.create_subscription(Twist, 'cmd_vel', handle_cmd_vel, qos_cmd)

            # Line-sensor message
            msg_line = LineStamped()

            # Odometry message
            msg_odom = Odometry()
            msg_odom.header.frame_id = param_odom_frame_id
            msg_odom.child_frame_id = param_agv_frame_id

            # Odometry initial state
            x, y, phi, gamma = 0.0, 0.0, 0.0, 0.0 # Robot configuration
            fd = 0.0 # Travelled distance of the front cart
            robot.read_sensors()
            left_prev, right_prev, central_start = robot.get_encoders()

            timer = node.create_timer(1.0/50.0, handle_timer)

            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()



if __name__ == '__main__':
    main()

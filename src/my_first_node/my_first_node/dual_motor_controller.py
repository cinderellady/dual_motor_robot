#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双电机 ROS 2 控制节点（支持单电机离线）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32

from my_first_node.motor_driver import M0603AMotor

WHEEL_BASE = 0.30
WHEEL_RADIUS = 0.05
MAX_RPM = 380

def twist_to_rpm(vx: float, wz: float) -> tuple:
    left_linear = vx - wz * WHEEL_BASE / 2.0
    right_linear = vx + wz * WHEEL_BASE / 2.0
    left_rpm = (left_linear / (2 * 3.14159 * WHEEL_RADIUS)) * 60.0
    right_rpm = (right_linear / (2 * 3.14159 * WHEEL_RADIUS)) * 60.0
    return max(-MAX_RPM, min(MAX_RPM, left_rpm)), max(-MAX_RPM, min(MAX_RPM, right_rpm))


class DualMotorController(Node):
    def __init__(self):
        super().__init__('dual_motor_controller')

        # 参数
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('left_id', 1)
        self.declare_parameter('right_id', 2)
        self.declare_parameter('pub_hz', 20)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        left_id = self.get_parameter('left_id').value
        right_id = self.get_parameter('right_id').value
        pub_hz = self.get_parameter('pub_hz').value

        if not M0603AMotor.init_serial(port, baud):
            raise RuntimeError("串口初始化失败")

        self.left = M0603AMotor(left_id)
        self.right = M0603AMotor(right_id)

        # 初始化电机，记录在线状态
        self.left_online = self.left.init()
        self.right_online = self.right.init()

        if not self.left_online and not self.right_online:
            self.get_logger().error("两个电机均离线，节点退出")
            raise RuntimeError("无可用电机")
        elif not self.left_online:
            self.get_logger().warn("左电机离线，将只控制右电机")
        elif not self.right_online:
            self.get_logger().warn("右电机离线，将只控制左电机")

        # 发布者（即使电机离线也创建，但不会发布数据）
        self.l_speed = self.create_publisher(Float32, '/left/actual_speed', 10)
        self.l_current = self.create_publisher(Float32, '/left/current', 10)
        self.l_temp = self.create_publisher(Float32, '/left/temperature', 10)
        self.l_ticks = self.create_publisher(Int32, '/left/odom_ticks', 10)
        self.l_angle = self.create_publisher(Float32, '/left/position', 10)

        self.r_speed = self.create_publisher(Float32, '/right/actual_speed', 10)
        self.r_current = self.create_publisher(Float32, '/right/current', 10)
        self.r_temp = self.create_publisher(Float32, '/right/temperature', 10)
        self.r_ticks = self.create_publisher(Int32, '/right/odom_ticks', 10)
        self.r_angle = self.create_publisher(Float32, '/right/position', 10)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_timer(1.0 / pub_hz, self.publish_states)

        self.get_logger().info("双电机节点已启动")

    def cmd_callback(self, msg: Twist):
        left_rpm, right_rpm = twist_to_rpm(msg.linear.x, msg.angular.z)
        if self.left_online:
            self.left.set_speed(int(left_rpm))
        if self.right_online:
            self.right.set_speed(int(right_rpm))

    def publish_states(self):
        # 左电机
        if self.left_online:
            fb = self.left.get_feedback()
            if fb:
                self.l_speed.publish(Float32(data=fb['actual_rpm']))
                self.l_current.publish(Float32(data=fb['current_A']))
                self.l_temp.publish(Float32(data=float(fb['temp_c'])))
            odom = self.left.get_odom()
            if odom:
                self.l_ticks.publish(Int32(data=odom['ticks']))
                self.l_angle.publish(Float32(data=odom['angle_deg']))

        # 右电机
        if self.right_online:
            fb = self.right.get_feedback()
            if fb:
                self.r_speed.publish(Float32(data=fb['actual_rpm']))
                self.r_current.publish(Float32(data=fb['current_A']))
                self.r_temp.publish(Float32(data=float(fb['temp_c'])))
            odom = self.right.get_odom()
            if odom:
                self.r_ticks.publish(Int32(data=odom['ticks']))
                self.r_angle.publish(Float32(data=odom['angle_deg']))

    def shutdown_hook(self):
        self.get_logger().info("关闭节点，停止电机")
        if self.left_online:
            self.left.set_speed(0)
            self.left.disable()
        if self.right_online:
            self.right.set_speed(0)
            self.right.disable()
        M0603AMotor.close_serial()


def main(args=None):
    rclpy.init(args=args)
    node = DualMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import time

from my_first_node.motor_driver import M0603AMotor   # 假设你的驱动类

class DualMotorControllerStep(Node):
    def __init__(self):
        super().__init__('dual_motor_controller_step')
        
        # 参数
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('left_id', 1)
        self.declare_parameter('right_id', 2)
        self.declare_parameter('max_linear_speed', 0.3)   # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('reverse_angular', True)   # 方向修正
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        left_id = self.get_parameter('left_id').value
        right_id = self.get_parameter('right_id').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.reverse_angular = self.get_parameter('reverse_angular').value
        
        # 初始化电机
        if not M0603AMotor.init_serial(port, baud):
            raise RuntimeError("串口初始化失败")
        self.left = M0603AMotor(left_id)
        self.right = M0603AMotor(right_id)
        self.left_online = self.left.init()
        self.right_online = self.right.init()
        if not self.left_online and not self.right_online:
            raise RuntimeError("无可用电机")
        
        # 速度控制定时器（用于自动停止）
        self.active_timer = None
        self.active_twist = Twist()
        
        # 订阅动作指令
        self.step_sub = self.create_subscription(String, '/step_cmd', self.step_callback, 10)
        # 可选：同时保留 /cmd_vel 用于传统连续控制（调试用）
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.get_logger().info("步进式电机控制器已启动")
        self.get_logger().info(f"参数: 线速度={self.max_linear}m/s, 角速度={self.max_angular}rad/s")
    
    def stop_motors(self):
        """停止电机"""
        twist = Twist()
        self._send_twist(twist)
        if self.active_timer:
            self.active_timer.cancel()
            self.active_timer = None
    
    def _send_twist(self, twist):
        """将Twist命令转换为左右轮速度并发送"""
        # 这里重用你原先的 twist_to_rpm 逻辑，复制过来略作修改
        # 因为你的 dual_motor_controller 中已有，此处直接实现
        vx = twist.linear.x
        wz = twist.angular.z
        left_rpm, right_rpm = self.twist_to_rpm(vx, wz)
        # 注意方向修正：如果你原先的节点中有 right_rpm = -right_rpm，这里保持一致
        right_rpm = -right_rpm   # 根据你原来逻辑
        if self.left_online:
            self.left.set_speed(int(left_rpm))
        if self.right_online:
            self.right.set_speed(int(right_rpm))
    
    def twist_to_rpm(self, vx, wz):
        WHEEL_BASE = 0.30
        WHEEL_RADIUS = 0.05
        MAX_RPM = 380
        left_linear = vx - wz * WHEEL_BASE / 2.0
        right_linear = vx + wz * WHEEL_BASE / 2.0
        left_rpm = (left_linear / (2 * 3.14159 * WHEEL_RADIUS)) * 60.0
        right_rpm = (right_linear / (2 * 3.14159 * WHEEL_RADIUS)) * 60.0
        left_rpm = max(-MAX_RPM, min(MAX_RPM, left_rpm))
        right_rpm = max(-MAX_RPM, min(MAX_RPM, right_rpm))
        return left_rpm, right_rpm
    
    def step_callback(self, msg):
        """处理动作指令，字符串格式: 'f 0.2' 或 'l 90'"""
        try:
            parts = msg.data.strip().split()
            if len(parts) != 2:
                self.get_logger().warn(f"无效指令: {msg.data}")
                return
            action = parts[0].lower()
            value = float(parts[1])
            
            twist = Twist()
            duration = 0.0
            
            if action == 'f':   # 前进
                twist.linear.x = self.max_linear
                duration = value / self.max_linear
                self.get_logger().info(f"前进 {value} 米，运行 {duration:.2f} 秒")
            elif action == 'b':   # 后退
                twist.linear.x = -self.max_linear
                duration = value / self.max_linear
                self.get_logger().info(f"后退 {value} 米，运行 {duration:.2f} 秒")
            elif action == 'l':   # 左转
                ang_rad = value * 3.14159 / 180.0
                wz = self.max_angular
                if self.reverse_angular:
                    wz = -wz
                twist.angular.z = wz
                duration = ang_rad / self.max_angular
                self.get_logger().info(f"左转 {value} 度，运行 {duration:.2f} 秒")
            elif action == 'r':   # 右转
                ang_rad = value * 3.14159 / 180.0
                wz = -self.max_angular
                if self.reverse_angular:
                    wz = -wz
                twist.angular.z = wz
                duration = ang_rad / self.max_angular
                self.get_logger().info(f"右转 {value} 度，运行 {duration:.2f} 秒")
            else:
                self.get_logger().warn(f"未知动作: {action}")
                return
            
            # 执行动作
            self._send_twist(twist)
            # 设置定时器自动停止
            if self.active_timer:
                self.active_timer.cancel()
            self.active_timer = threading.Timer(duration, self.stop_motors)
            self.active_timer.daemon = True
            self.active_timer.start()
            
        except Exception as e:
            self.get_logger().error(f"处理指令出错: {e}")
    
    def cmd_vel_callback(self, msg):
        """保留传统连续控制（可选）"""
        self._send_twist(msg)
        # 如果有动作定时器，取消它
        if self.active_timer:
            self.active_timer.cancel()
            self.active_timer = None
        self.get_logger().debug("连续控制模式")
    
    def shutdown_hook(self):
        self.stop_motors()
        if self.left_online:
            self.left.disable()
        if self.right_online:
            self.right.disable()
        M0603AMotor.close_serial()

def main(args=None):
    rclpy.init(args=args)
    node = DualMotorControllerStep()
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
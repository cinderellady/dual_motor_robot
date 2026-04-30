#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import time

from my_first_node.motor_driver import M0603AMotor

# ========== 方向配置（根据实际接线调整） ==========
# 如果前进时实际后退，设为 True
INVERT_LINEAR = False
# 如果右转实际左转，设为 True
INVERT_ANGULAR = False
# 如果右轮速度符号需要取反（通常取决于电机安装方向），设为 True
INVERT_RIGHT_WHEEL = False   # 注意：你原始代码中用了 -right_rpm，这里改为 False 则取消取反
# =================================================

class DualMotorControllerStep(Node):
    def __init__(self):
        super().__init__('dual_motor_controller_step')
        
        # 参数
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('left_id', 1)
        self.declare_parameter('right_id', 2)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        left_id = self.get_parameter('left_id').value
        right_id = self.get_parameter('right_id').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        
        # 初始化电机
        if not M0603AMotor.init_serial(port, baud):
            raise RuntimeError("串口初始化失败")
        self.left = M0603AMotor(left_id)
        self.right = M0603AMotor(right_id)
        self.left_online = self.left.init()
        self.right_online = self.right.init()
        if not self.left_online and not self.right_online:
            raise RuntimeError("无可用电机")
        
        self.active_timer = None
        self.step_sub = self.create_subscription(String, '/step_cmd', self.step_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.get_logger().info("步进式电机控制器已启动（方向可配置）")
        self.get_logger().info(f"INVERT_LINEAR={INVERT_LINEAR}, INVERT_ANGULAR={INVERT_ANGULAR}, INVERT_RIGHT_WHEEL={INVERT_RIGHT_WHEEL}")
    
    def stop_motors(self):
        twist = Twist()
        self._send_twist(twist)
        if self.active_timer:
            self.active_timer.cancel()
            self.active_timer = None
    
    def _send_twist(self, twist):
        # 应用方向修正
        vx = twist.linear.x
        wz = twist.angular.z
        if INVERT_LINEAR:
            vx = -vx
        if INVERT_ANGULAR:
            wz = -wz
        
        left_rpm, right_rpm = self.twist_to_rpm(vx, wz)
        
        # 右轮取反修正
        if INVERT_RIGHT_WHEEL:
            right_rpm = -right_rpm
        
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
        try:
            parts = msg.data.strip().split()
            if len(parts) != 2:
                self.get_logger().warn(f"无效指令: {msg.data}")
                return
            action = parts[0].lower()
            value = float(parts[1])
            
            twist = Twist()
            duration = 0.0
            
            if action == 'f':
                twist.linear.x = self.max_linear
                duration = value / self.max_linear
            elif action == 'b':
                twist.linear.x = -self.max_linear
                duration = value / self.max_linear
            elif action == 'l':
                twist.angular.z = self.max_angular
                duration = (value * 3.14159 / 180.0) / self.max_angular
            elif action == 'r':
                twist.angular.z = -self.max_angular
                duration = (value * 3.14159 / 180.0) / self.max_angular
            else:
                self.get_logger().warn(f"未知动作: {action}")
                return
            
            self._send_twist(twist)
            if self.active_timer:
                self.active_timer.cancel()
            self.active_timer = threading.Timer(duration, self.stop_motors)
            self.active_timer.daemon = True
            self.active_timer.start()
            
        except Exception as e:
            self.get_logger().error(f"处理指令出错: {e}")
    
    def cmd_vel_callback(self, msg):
        self._send_twist(msg)
        if self.active_timer:
            self.active_timer.cancel()
            self.active_timer = None
    
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
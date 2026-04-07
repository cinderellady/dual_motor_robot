#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
最简单的单电机 ROS 2 控制节点
订阅 /motor_speed (Int32) -> 设置电机转速 (RPM)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# 导入你已有的电机控制类（假设 motor_speed_test.py 在同一目录）
# 如果不在同一目录，请修改导入路径或拷贝类定义过来
from my_first_node.motor_speed_test import M0603ASpeedMotor

class SingleMotorController(Node):
    def __init__(self):
        super().__init__('single_motor_controller')

        # 声明参数：串口、电机ID、波特率（方便修改）
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('baudrate', 38400)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        motor_id = self.get_parameter('motor_id').get_parameter_value().integer_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # 创建电机对象并连接
        self.motor = M0603ASpeedMotor(port, motor_id, baudrate)
        if not self.motor.connect():
            self.get_logger().error('电机串口连接失败，节点退出')
            raise RuntimeError('电机连接失败')

        # 初始化电机：使能 + 切换到速度环
        self.get_logger().info('正在初始化电机...')
        self.motor.init_motor()   # 内部调用 enable() 和切换速度环
        self.get_logger().info('电机初始化完成')

        # 创建订阅者：接收 /motor_speed (Int32)
        self.subscription = self.create_subscription(
            Int32,
            '/motor_speed',
            self.speed_callback,
            10
        )
        self.get_logger().info('节点已启动，等待 /motor_speed 话题上的转速指令...')

        # 注册关闭回调，确保退出时电机停止并失能
        self.get_logger().info('注册关闭回调，退出时将停止电机')
        # self.add_on_shutdown_hook(self.shutdown_hook)

    def speed_callback(self, msg: Int32):
        """接收到转速指令时的回调函数"""
        rpm = msg.data
        # 限制范围（-380 ~ 380）
        if rpm < -380:
            rpm = -380
            self.get_logger().warn('转速超出下限，已限制为 -380 RPM')
        elif rpm > 380:
            rpm = 380
            self.get_logger().warn('转速超出上限，已限制为 380 RPM')

        self.get_logger().info(f'设置转速: {rpm} RPM')
        # 调用电机控制类的方法设置转速
        self.motor.set_speed(rpm)

    def shutdown_hook(self):
        """节点关闭时的清理工作：停止电机并失能"""
        self.get_logger().info('节点关闭，停止电机并失能...')
        self.motor.set_speed(0)   # 速度归零
        self.motor.disable()      # 失能电机
        self.motor.disconnect()   # 关闭串口

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SingleMotorController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        if node is not None:
            node.shutdown_hook()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
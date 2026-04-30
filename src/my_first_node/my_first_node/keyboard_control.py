#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import time

class KeyboardArrowControl(Node):
    def __init__(self):
        super().__init__('keyboard_arrow_control')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_cmd)  # 10 Hz
        self.get_logger().info("""
        控制说明:
        ===========
        上下箭头:     前进 / 后退
        左右箭头:     左转 / 右转
        W / S  :     增加 / 减小最大线速度 (当前: %.2f m/s)
        A / D  :     减小 / 增加最大角速度 (当前: %.2f rad/s)
        空格键   :     紧急停车
        R       :     重置速度限制
        Ctrl+C  :     退出节点
        """ % (0.5, 1.0))

        # 速度限制 (初始值)
        self.max_linear = 0.5      # m/s
        self.max_angular = 1.0     # rad/s

        # 当前控制指令 (由箭头按键设置)
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0

        # 按键状态（支持按住连续运动，松开后延迟停）
        self.last_active_time = time.time()
        self.active_timeout = 0.5  # 秒，无按键后等待多久自动归零

        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)

    def get_arrow_key(self):
        """读取单个字符，支持箭头键（转义序列）和字母键"""
        tty.setraw(sys.stdin.fileno())
        if select.select([sys.stdin], [], [], 0.01)[0]:
            key = sys.stdin.read(1)
            if key == '\x1b':          # ESC 开头（箭头键）
                seq = sys.stdin.read(2)
                if seq == '[A':
                    key = 'UP'
                elif seq == '[B':
                    key = 'DOWN'
                elif seq == '[C':
                    key = 'RIGHT'
                elif seq == '[D':
                    key = 'LEFT'
                else:
                    key = None
            elif key == ' ':
                key = 'SPACE'
            elif key in ('w', 'W'):
                key = 'W'
            elif key in ('s', 'S'):
                key = 'S'
            elif key in ('a', 'A'):
                key = 'A'
            elif key in ('d', 'D'):
                key = 'D'
            elif key in ('r', 'R'):
                key = 'RESET'
            elif key == '\x03':      # Ctrl+C
                raise KeyboardInterrupt
            else:
                key = None
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_arrow_key()
                if key is not None:
                    self.last_active_time = time.time()
                    handled = True
                    if key == 'UP':
                        self.cmd_linear = self.max_linear
                        self.cmd_angular = 0.0
                    elif key == 'DOWN':
                        self.cmd_linear = -self.max_linear
                        self.cmd_angular = 0.0
                    elif key == 'LEFT':
                        self.cmd_linear = 0.0
                        self.cmd_angular = self.max_angular
                    elif key == 'RIGHT':
                        self.cmd_linear = 0.0
                        self.cmd_angular = -self.max_angular
                    elif key == 'SPACE':
                        self.cmd_linear = 0.0
                        self.cmd_angular = 0.0
                        self.get_logger().info("紧急停车")
                    elif key == 'W':
                        self.max_linear = min(2.0, self.max_linear + 0.05)
                        self.get_logger().info(f"线速度上限 = {self.max_linear:.2f} m/s")
                        # 如果当前正在前进/后退，立即更新当前指令
                        if self.cmd_linear > 0:
                            self.cmd_linear = self.max_linear
                        elif self.cmd_linear < 0:
                            self.cmd_linear = -self.max_linear
                    elif key == 'S':
                        self.max_linear = max(0.05, self.max_linear - 0.05)
                        self.get_logger().info(f"线速度上限 = {self.max_linear:.2f} m/s")
                        if self.cmd_linear > 0:
                            self.cmd_linear = self.max_linear
                        elif self.cmd_linear < 0:
                            self.cmd_linear = -self.max_linear
                    elif key == 'D':
                        self.max_angular = min(2.5, self.max_angular + 0.1)
                        self.get_logger().info(f"角速度上限 = {self.max_angular:.2f} rad/s")
                        if self.cmd_angular > 0:
                            self.cmd_angular = self.max_angular
                        elif self.cmd_angular < 0:
                            self.cmd_angular = -self.max_angular
                    elif key == 'A':
                        self.max_angular = max(0.1, self.max_angular - 0.1)
                        self.get_logger().info(f"角速度上限 = {self.max_angular:.2f} rad/s")
                        if self.cmd_angular > 0:
                            self.cmd_angular = self.max_angular
                        elif self.cmd_angular < 0:
                            self.cmd_angular = -self.max_angular
                    elif key == 'RESET':
                        self.max_linear = 0.5
                        self.max_angular = 1.0
                        self.get_logger().info("速度限制已重置 (线:0.5, 角:1.0)")
                        if self.cmd_linear > 0:
                            self.cmd_linear = self.max_linear
                        elif self.cmd_linear < 0:
                            self.cmd_linear = -self.max_linear
                        if self.cmd_angular > 0:
                            self.cmd_angular = self.max_angular
                        elif self.cmd_angular < 0:
                            self.cmd_angular = -self.max_angular
                    else:
                        handled = False

                    if handled:
                        self.get_logger().info(f"发布 Twist: 线速度={self.cmd_linear:.2f}, 角速度={self.cmd_angular:.2f}")
                else:
                    # 无按键，检查超时后自动停止
                    if time.time() - self.last_active_time > self.active_timeout:
                        if self.cmd_linear != 0.0 or self.cmd_angular != 0.0:
                            self.cmd_linear = 0.0
                            self.cmd_angular = 0.0
                            self.get_logger().debug("自动停止")

                # 保持节点响应其他ROS事件
                rclpy.spin_once(self, timeout_sec=0.01)

        except KeyboardInterrupt:
            pass
        finally:
            # 停止运动
            self.cmd_linear = 0.0
            self.cmd_angular = 0.0
            self.publish_cmd()
            time.sleep(0.1)
            self.get_logger().info("键盘控制节点退出")

    def publish_cmd(self):
        twist = Twist()
        twist.linear.x = self.cmd_linear
        twist.angular.z = self.cmd_angular
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardArrowControl()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
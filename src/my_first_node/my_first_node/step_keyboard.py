#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select
import time

class StepKeyboard(Node):
    def __init__(self):
        super().__init__('step_keyboard')
        self.pub = self.create_publisher(String, '/step_cmd', 10)
        
        # 步长参数
        self.linear_step = 0.1      # 米
        self.angular_step = 15.0    # 度
        
        # 防连发间隔（秒）
        self.interval = 0.2
        self.last_trigger_time = 0
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.print_help()
    
    def print_help(self):
        self.get_logger().info("""
步进式键盘控制 (无延迟版，树莓派本地执行)
==========================================
上箭头: 前进 %.2f 米
下箭头: 后退 %.2f 米
左箭头: 左转 %.1f 度
右箭头: 右转 %.1f 度
W: 增加线步长 (+0.05米)  当前:%.2f
S: 减小线步长 (-0.05米)  当前:%.2f
D: 增加角步长 (+5.0度)   当前:%.1f
A: 减小角步长 (-5.0度)   当前:%.1f
按住箭头: 连续步进 (约5Hz)
Ctrl+C: 退出
        """.format(self.linear_step, self.linear_step,
                   self.angular_step, self.angular_step,
                   self.linear_step, self.linear_step,
                   self.angular_step, self.angular_step))
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        if select.select([sys.stdin], [], [], 0.02)[0]:
            key = sys.stdin.read(1)
            if key == '\x1b':
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
            elif key == '\x03':
                raise KeyboardInterrupt
            else:
                key = None
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def send_step(self, action, value):
        msg = String()
        msg.data = f"{action} {value}"
        self.pub.publish(msg)
        self.get_logger().info(f"发送指令: {msg.data}")
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                now = time.time()
                
                # 调节步长
                if key == 'W':
                    self.linear_step = min(0.5, self.linear_step + 0.05)
                    self.get_logger().info(f"线步长 -> {self.linear_step:.2f} 米")
                elif key == 'S':
                    self.linear_step = max(0.05, self.linear_step - 0.05)
                    self.get_logger().info(f"线步长 -> {self.linear_step:.2f} 米")
                elif key == 'D':
                    self.angular_step = min(90.0, self.angular_step + 5.0)
                    self.get_logger().info(f"角步长 -> {self.angular_step:.1f} 度")
                elif key == 'A':
                    self.angular_step = max(5.0, self.angular_step - 5.0)
                    self.get_logger().info(f"角步长 -> {self.angular_step:.1f} 度")
                
                # 运动触发
                if key in ('UP', 'DOWN', 'LEFT', 'RIGHT'):
                    # 防连发间隔
                    if now - self.last_trigger_time >= self.interval:
                        self.last_trigger_time = now
                        if key == 'UP':
                            self.send_step('f', self.linear_step)
                        elif key == 'DOWN':
                            self.send_step('b', self.linear_step)
                        elif key == 'LEFT':
                            self.send_step('l', self.angular_step)
                        elif key == 'RIGHT':
                            self.send_step('r', self.angular_step)
                
                # 处理ROS事件
                rclpy.spin_once(self, timeout_sec=0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.get_logger().info("节点退出")

def main(args=None):
    rclpy.init(args=args)
    node = StepKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
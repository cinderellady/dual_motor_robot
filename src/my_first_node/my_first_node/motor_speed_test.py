#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
M0603A FOC 电机速度模式专用控制程序
====================================
功能：
- 电机使能/失能
- 设置目标转速（-380 ~ 380 RPM）
- 读取实际转速、电流、温度、里程圈数、当前位置角度
- 实时监控（转速、电流、温度）
- 发送原始十六进制指令（调试用）

适用于两轮差速机器人驱动轮测试
"""

import serial
import serial.tools.list_ports
import struct
import time
import sys
import threading

# ================== CRC-8/MAXIM 算法 ==================
def crc8_maxim(data: bytes) -> int:
    """CRC-8/MAXIM 校验算法（多项式 x^8 + x^5 + x^4 + 1）"""
    crc = 0x00
    poly = 0x8C          # 多项式 0x31 的反转
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
    return crc

def build_command(motor_id: int, cmd_byte: int, data7: bytes) -> bytes:
    """构建10字节指令，自动添加CRC"""
    if len(data7) != 7:
        raise ValueError("data7 must be 7 bytes")
    cmd = bytearray([motor_id, cmd_byte])
    cmd.extend(data7)
    crc = crc8_maxim(bytes(cmd))
    cmd.append(crc)
    return bytes(cmd)


class M0603ASpeedMotor:
    """M0603A 电机速度模式控制类"""

    # 命令标识符
    CMD_DRIVE = 0x64          # 驱动指令
    CMD_MODE_SWITCH = 0xA0    # 模式切换指令
    CMD_GET_FEEDBACK = 0x74   # 获取里程/位置反馈
    CMD_GET_VERSION = 0xFD    # 获取版本号

    # 模式值
    MODE_ENABLE = 0x08        # 电机使能
    MODE_DISABLE = 0x09       # 电机失能
    MODE_SPEED = 0x02         # 速度环 

    # 故障码位定义
    FAULT_HALL = 0x01
    FAULT_OVER_CURRENT = 0x02
    FAULT_STALL = 0x08
    FAULT_OVER_TEMP = 0x10
    FAULT_DISCONNECT = 0x20
    FAULT_OVER_UNDER_VOLTAGE = 0x40

    def __init__(self, port: str, motor_id: int = 1, baudrate: int = 38400, timeout: float = 0.5):
        self.port = port
        self.motor_id = motor_id
        self.baudrate = baudrate
        self.serial = None
        self.timeout = timeout
        self.current_target_rpm = 0
        self.monitoring = False
        self.monitor_thread = None

    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=self.timeout
            )
            print(f"[OK] 已连接 {self.port} @ {self.baudrate} bps")
            return True
        except Exception as e:
            print(f"[ERROR] 串口连接失败: {e}")
            return False

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("[OK] 串口已关闭")

    def _send_and_read(self, cmd: bytes, wait_time: float = 0.05) -> bytes:
        if not self.serial:
            return b''
        self.serial.write(cmd)
        time.sleep(wait_time)
        if self.serial.in_waiting:
            return self.serial.read(self.serial.in_waiting)
        return b''

    def send_raw(self, hex_str: str) -> bytes:
        """发送原始十六进制指令"""
        hex_str = hex_str.replace(" ", "")
        try:
            cmd_bytes = bytes.fromhex(hex_str)
        except ValueError:
            print("[ERROR] 无效的十六进制字符串")
            return b''
        print(f"[RAW] 发送: {' '.join(f'{b:02X}' for b in cmd_bytes)}")
        resp = self._send_and_read(cmd_bytes)
        if resp:
            print(f"[RSP] {' '.join(f'{b:02X}' for b in resp)}")
        return resp

    # ------------------ 基础控制命令 ------------------
    def enable(self) -> bool:
        """电机使能"""
        cmd = build_command(self.motor_id, self.CMD_MODE_SWITCH, bytes([self.MODE_ENABLE, 0, 0, 0, 0, 0, 0]))
        print(f"[CMD] 使能: {' '.join(f'{b:02X}' for b in cmd)}")
        resp = self._send_and_read(cmd)
        if resp:
            print(f"[RSP] { ' '.join(f'{b:02X}' for b in resp) }")
        return resp != b''

    def disable(self) -> bool:
        """电机失能"""
        cmd = build_command(self.motor_id, self.CMD_MODE_SWITCH, bytes([self.MODE_DISABLE, 0, 0, 0, 0, 0, 0]))
        print(f"[CMD] 失能: {' '.join(f'{b:02X}' for b in cmd)}")
        resp = self._send_and_read(cmd)
        if resp:
            print(f"[RSP] { ' '.join(f'{b:02X}' for b in resp) }")
        return resp != b''

    def set_speed(self, rpm: int) -> dict:
        """
        设置目标转速（速度环模式）
        :param rpm: -380 ~ 380
        :return: 反馈字典（实际转速、电流、温度等）
        """
        if rpm < -380:
            rpm = -380
        if rpm > 380:
            rpm = 380
        self.current_target_rpm = rpm
        value = rpm * 10                      # 给定值 = 实际转速 × 10
        speed_bytes = struct.pack('>h', value)  # 大端有符号16位
        data = bytes([speed_bytes[0], speed_bytes[1], 0x00, 0x00, 0x00, 0x00, 0x00])
        cmd = build_command(self.motor_id, self.CMD_DRIVE, data)
        print(f"[CMD] 转速 {rpm} RPM: {' '.join(f'{b:02X}' for b in cmd)}")
        resp = self._send_and_read(cmd)
        if resp:
            print(f"[RSP] { ' '.join(f'{b:02X}' for b in resp) }")
            return self._parse_drive_feedback(resp)
        return {}

    def brake(self) -> bool:
        """刹车（速度环有效）"""
        data = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00])
        cmd = build_command(self.motor_id, self.CMD_DRIVE, data)
        print(f"[CMD] 刹车: {' '.join(f'{b:02X}' for b in cmd)}")
        resp = self._send_and_read(cmd)
        if resp:
            print(f"[RSP] { ' '.join(f'{b:02X}' for b in resp) }")
        return resp != b''

    def set_acceleration(self, acc_ms_per_rpm: int) -> bool:
        """设置加速时间（1~255 ms/rpm，0表示默认1ms）"""
        if acc_ms_per_rpm < 0:
            acc_ms_per_rpm = 0
        if acc_ms_per_rpm > 255:
            acc_ms_per_rpm = 255
        data = bytes([0x00, 0x00, 0x00, 0x00, acc_ms_per_rpm, 0x00, 0x00])
        cmd = build_command(self.motor_id, self.CMD_DRIVE, data)
        print(f"[CMD] 加速时间 {acc_ms_per_rpm} ms/rpm: {' '.join(f'{b:02X}' for b in cmd)}")
        resp = self._send_and_read(cmd)
        if resp:
            print(f"[RSP] { ' '.join(f'{b:02X}' for b in resp) }")
        return resp != b''

    # ------------------ 反馈解析 ------------------
    def _parse_drive_feedback(self, resp: bytes) -> dict:
        """解析驱动指令反馈（0x65）"""
        if len(resp) < 9 or resp[1] != 0x65:
            return {}
        speed_raw = struct.unpack('>h', resp[2:4])[0]
        actual_speed_rpm = speed_raw / 10.0
        current_raw = struct.unpack('>h', resp[4:6])[0]
        current_A = current_raw / 32767.0 * 4.0
        acceleration = resp[6]
        temperature = resp[7]
        fault_code = resp[8]
        return {
            'actual_speed_rpm': actual_speed_rpm,
            'current_A': current_A,
            'acceleration_param': acceleration,
            'temperature_c': temperature,
            'fault_code': fault_code
        }

    def get_odom_position(self) -> dict:
        """获取里程圈数和当前位置角度"""
        cmd = build_command(self.motor_id, self.CMD_GET_FEEDBACK, bytes([0, 0, 0, 0, 0, 0, 0]))
        print(f"[CMD] 获取里程/位置: {' '.join(f'{b:02X}' for b in cmd)}")
        resp = self._send_and_read(cmd, wait_time=0.1)
        if len(resp) >= 9 and resp[1] == 0x75:
            # 里程圈数：大端有符号32位（根据实测修改为 '>i'）
            total_ticks = struct.unpack('>i', resp[2:6])[0]
            # 当前位置：大端无符号16位，0~32767对应0~360°
            pos_raw = struct.unpack('>H', resp[6:8])[0]
            position_deg = pos_raw / 32767.0 * 360.0
            fault_code = resp[8]
            return {
                'total_ticks': total_ticks,
                'position_deg': position_deg,
                'fault_code': fault_code
            }
        else:
            print("获取里程/位置失败")
            return {}

    def get_version(self) -> dict:
        """获取固件版本"""
        cmd = build_command(self.motor_id, self.CMD_GET_VERSION, bytes([0, 0, 0, 0, 0, 0, 0]))
        print(f"[CMD] 获取版本: {' '.join(f'{b:02X}' for b in cmd)}")
        resp = self._send_and_read(cmd, wait_time=0.1)
        if len(resp) >= 8 and resp[1] == 0xFE:
            year = resp[2] + 2000
            month = resp[3]
            day = resp[4]
            motor_type = resp[5]
            sw_version = resp[6]
            hw_version = resp[7]
            return {
                'date': f"{year}-{month:02d}-{day:02d}",
                'motor_type': motor_type,
                'sw_version': sw_version,
                'hw_version': hw_version
            }
        return {}

    # ------------------ 状态显示与监控 ------------------
    def show_status(self):
        """显示详细状态（实际转速、电流、温度、里程、位置）"""
        print("\n--- 电机状态 ---")
        # 发送速度为0获取驱动反馈
        fb = self.set_speed(0)
        if fb:
            print(f"实际转速: {fb['actual_speed_rpm']:.1f} RPM")
            print(f"电流: {fb['current_A']:.3f} A")
            print(f"温度: {fb['temperature_c']} °C")
            print(f"加速参数: {fb['acceleration_param']} ms/rpm")
            fault = fb['fault_code']
            if fault != 0:
                print(f"故障码: 0x{fault:02X}")
                if fault & self.FAULT_OVER_CURRENT:
                    print("  - 过流故障")
                if fault & self.FAULT_OVER_TEMP:
                    print("  - 过温故障")
                if fault & self.FAULT_STALL:
                    print("  - 堵转故障")
                if fault & self.FAULT_HALL:
                    print("  - 霍尔故障")
                if fault & self.FAULT_OVER_UNDER_VOLTAGE:
                    print("  - 过欠压故障")
                if fault & self.FAULT_DISCONNECT:
                    print("  - 断联故障")
        else:
            print("获取驱动反馈失败")

        odom = self.get_odom_position()
        if odom:
            print(f"总里程圈数: {odom['total_ticks']}")
            print(f"当前位置: {odom['position_deg']:.1f}°")
        print("----------------\n")

    def _monitor_loop(self, interval: float = 0.2):
        """监控线程：周期性发送当前目标转速并显示实际值"""
        while self.monitoring:
            if self.serial:
                value = self.current_target_rpm * 10
                speed_bytes = struct.pack('>h', value)
                data = bytes([speed_bytes[0], speed_bytes[1], 0x00, 0x00, 0x00, 0x00, 0x00])
                cmd = build_command(self.motor_id, self.CMD_DRIVE, data)
                self.serial.write(cmd)
                time.sleep(0.02)
                if self.serial.in_waiting:
                    resp = self.serial.read(self.serial.in_waiting)
                    if len(resp) >= 9 and resp[1] == 0x65:
                        speed_raw = struct.unpack('>h', resp[2:4])[0]
                        actual_rpm = speed_raw / 10.0
                        current_raw = struct.unpack('>h', resp[4:6])[0]
                        current_A = current_raw / 32767.0 * 4.0
                        temp = resp[7]
                        print(f"\r目标: {self.current_target_rpm:3.0f} RPM | "
                              f"实际: {actual_rpm:5.1f} RPM | "
                              f"电流: {current_A:5.3f} A | "
                              f"温度: {temp:2}°C   ", end='', flush=True)
            time.sleep(interval)

    def start_monitor(self):
        if self.monitoring:
            print("监控已在运行")
            return
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        print("实时监控已启动（按 Ctrl+C 停止）")

    def stop_monitor(self):
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=0.5)
        print("\n监控已停止")

    def init_motor(self):
        """初始化：使能 + 切换到速度环"""
        print("\n=== 初始化电机 ===")
        self.enable()
        time.sleep(0.05)
        # 切换到速度环
        cmd = build_command(self.motor_id, self.CMD_MODE_SWITCH, bytes([self.MODE_SPEED, 0, 0, 0, 0, 0, 0]))
        print(f"[CMD] 切换速度环: {' '.join(f'{b:02X}' for b in cmd)}")
        resp = self._send_and_read(cmd)
        if resp:
            print(f"[RSP] { ' '.join(f'{b:02X}' for b in resp) }")
        print("=== 初始化完成 ===\n")


# ================== 交互式控制台 ==================
def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到任何串口设备")
        return []
    print("可用串口:")
    for i, p in enumerate(ports):
        print(f"  {i}: {p.device} - {p.description}")
    return [p.device for p in ports]

def interactive_control(motor: M0603ASpeedMotor):
    print("\n========== M0603A 速度模式专用控制台 ==========")
    print("命令列表:")
    print("  speed <rpm>      - 设置转速 (-380~380)")
    print("  brake            - 刹车")
    print("  enable/disable   - 使能/失能")
    print("  acc <ms>         - 加速时间 (1~255 ms/rpm)")
    print("  status           - 显示状态（转速、电流、温度、里程、位置）")
    print("  monitor          - 实时监控（每0.2秒刷新）")
    print("  stop             - 停止监控并设置转速为0")
    print("  version          - 获取固件版本")
    print("  init             - 重新初始化（使能+速度环）")
    print("  raw <hex>        - 发送原始十六进制指令")
    print("  quit/exit        - 退出")
    print("=================================================")

    while True:
        try:
            cmd_line = input("\n>>> ").strip().lower()
            if not cmd_line:
                continue
            if cmd_line in ("quit", "exit"):
                break
            elif cmd_line == "enable":
                motor.enable()
            elif cmd_line == "disable":
                motor.disable()
            elif cmd_line.startswith("speed"):
                parts = cmd_line.split()
                if len(parts) != 2:
                    print("用法: speed <转速>")
                else:
                    try:
                        rpm = int(parts[1])
                        motor.set_speed(rpm)
                    except:
                        print("转速必须是整数")
            elif cmd_line == "brake":
                motor.brake()
            elif cmd_line.startswith("acc"):
                parts = cmd_line.split()
                if len(parts) != 2:
                    print("用法: acc <1~255>")
                else:
                    try:
                        acc = int(parts[1])
                        motor.set_acceleration(acc)
                    except:
                        print("参数必须是整数")
            elif cmd_line == "status":
                motor.show_status()
            elif cmd_line == "monitor":
                motor.start_monitor()
                try:
                    while motor.monitoring:
                        time.sleep(0.5)
                except KeyboardInterrupt:
                    motor.stop_monitor()
            elif cmd_line == "stop":
                motor.stop_monitor()
                motor.set_speed(0)
            elif cmd_line == "version":
                ver = motor.get_version()
                if ver:
                    print(f"版本: {ver}")
                else:
                    print("获取版本失败")
            elif cmd_line == "init":
                motor.init_motor()
            elif cmd_line.startswith("raw"):
                parts = cmd_line.split()
                if len(parts) != 2:
                    print("用法: raw <十六进制指令>")
                else:
                    motor.send_raw(parts[1])
            else:
                print("未知命令")
        except KeyboardInterrupt:
            motor.stop_monitor()
            print("\n用户中断")
            break
        except Exception as e:
            print(f"错误: {e}")

def main():
    ports = list_serial_ports()
    if not ports:
        sys.exit(1)
    if len(ports) == 1:
        port = ports[0]
        print(f"自动选择: {port}")
    else:
        try:
            idx = int(input("请输入序号: "))
            port = ports[idx]
        except:
            print("无效选择")
            sys.exit(1)

    try:
        motor_id = int(input("电机ID (1或2, 默认1): ") or "1")
        if motor_id not in (1,2):
            motor_id = 1
    except:
        motor_id = 1

    motor = M0603ASpeedMotor(port, motor_id, baudrate=38400)
    if not motor.connect():
        sys.exit(1)

    print("\n是否执行初始化 (使能 + 切换到速度环)? [Y/n]: ", end="")
    choice = input().strip().lower()
    if choice != 'n':
        motor.init_motor()
    else:
        print("跳过初始化，请确保电机已使能并处于速度环模式")

    interactive_control(motor)
    motor.disconnect()
    print("程序结束")

if __name__ == "__main__":
    main()
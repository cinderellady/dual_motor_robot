#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
M0603A 电机驱动类（共享串口，带响应验证和离线检测）
"""

import serial
import struct
import time
import threading

# ---------- CRC-8/MAXIM ----------
def _crc8_maxim(data: bytes) -> int:
    crc = 0x00
    poly = 0x8C
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
    return crc

def _build_cmd(motor_id: int, cmd_byte: int, data7: bytes) -> bytes:
    if len(data7) != 7:
        raise ValueError("data7 must be 7 bytes")
    cmd = bytearray([motor_id, cmd_byte]) + data7
    cmd.append(_crc8_maxim(cmd))
    return bytes(cmd)


class M0603AMotor:
    """电机控制类（共享串口，带响应验证）"""
    _lock = threading.Lock()
    _serial = None

    CMD_DRIVE = 0x64
    CMD_MODE = 0xA0
    CMD_ODOM = 0x74

    MODE_ENABLE = 0x08
    MODE_DISABLE = 0x09
    MODE_SPEED = 0x02

    @classmethod
    def init_serial(cls, port: str, baudrate: int = 38400) -> bool:
        if cls._serial is None:
            try:
                cls._serial = serial.Serial(port, baudrate, timeout=0.1)
                print(f"[OK] 共享串口已打开: {port}")
                return True
            except Exception as e:
                print(f"[ERROR] 打开串口失败: {e}")
                return False
        return True

    @classmethod
    def close_serial(cls):
        if cls._serial and cls._serial.is_open:
            cls._serial.close()
            print("[OK] 共享串口已关闭")

    def __init__(self, motor_id: int):
        self.id = motor_id
        self._target_rpm = 0
        self.online = False          # 新增：电机是否在线

    def _send_recv(self, cmd: bytes, wait: float = 0.05) -> bytes:
        if not self._lock.acquire(timeout=1.0):
            print(f"[WARN] 电机{self.id} 获取串口锁超时")
            return b''
        try:
            if not self._serial:
                return b''
            self._serial.write(cmd)
            time.sleep(wait)
            return self._serial.read(self._serial.in_waiting)
        finally:
            self._lock.release()

    def _check_response(self, resp: bytes, expected_cmd_byte: int) -> bool:
        """检查响应是否有效（长度>=9且命令字节匹配）"""
        return len(resp) >= 9 and resp[1] == expected_cmd_byte

    def enable(self) -> bool:
        cmd = _build_cmd(self.id, self.CMD_MODE, bytes([self.MODE_ENABLE, 0,0,0,0,0,0]))
        resp = self._send_recv(cmd)
        ok = self._check_response(resp, 0xA1)   # 使能成功响应命令字节为0xA1
        if not ok:
            print(f"[ERROR] 电机{self.id} 使能失败，无响应")
        return ok

    def disable(self) -> bool:
        cmd = _build_cmd(self.id, self.CMD_MODE, bytes([self.MODE_DISABLE, 0,0,0,0,0,0]))
        resp = self._send_recv(cmd)
        return self._check_response(resp, 0xA1)

    def set_speed(self, rpm: int) -> dict:
        if not self.online:
            return {}   # 离线电机不发送指令
        rpm = max(-380, min(380, rpm))
        self._target_rpm = rpm
        val = rpm * 10
        data = struct.pack('>h', val) + bytes(5)
        cmd = _build_cmd(self.id, self.CMD_DRIVE, data)
        resp = self._send_recv(cmd)
        if self._check_response(resp, 0x65):
            speed_raw = struct.unpack('>h', resp[2:4])[0]
            current_raw = struct.unpack('>h', resp[4:6])[0]
            return {
                'actual_rpm': speed_raw / 10.0,
                'current_A': current_raw / 32767.0 * 4.0,
                'temp_c': resp[7],
                'fault': resp[8]
            }
        return {}

    def get_feedback(self) -> dict:
        if not self.online:
            return {}
        return self.set_speed(self._target_rpm)

    def get_odom(self) -> dict:
        if not self.online:
            return {}
        cmd = _build_cmd(self.id, self.CMD_ODOM, bytes(7))
        resp = self._send_recv(cmd, wait=0.1)
        if self._check_response(resp, 0x75):
            ticks = struct.unpack('>i', resp[2:6])[0]
            raw_ang = struct.unpack('>H', resp[6:8])[0]
            return {
                'ticks': ticks,
                'angle_deg': raw_ang / 32767.0 * 360.0,
                'fault': resp[8]
            }
        return {}

    def init(self) -> bool:
        """初始化电机，返回是否成功"""
        print(f"[INFO] 电机{self.id} 正在初始化...")
        if not self.enable():
            self.online = False
            print(f"[ERROR] 电机{self.id} 使能失败，标记为离线")
            return False
        time.sleep(0.05)
        cmd = _build_cmd(self.id, self.CMD_MODE, bytes([self.MODE_SPEED, 0,0,0,0,0,0]))
        resp = self._send_recv(cmd)
        if not self._check_response(resp, 0xA1):
            self.online = False
            print(f"[ERROR] 电机{self.id} 切换速度环失败，标记为离线")
            return False
        self.online = True
        print(f"[INFO] 电机{self.id} 初始化成功，在线")
        return True
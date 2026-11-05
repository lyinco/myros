# -*- coding: utf-8 -*-
"""Hiwonder 10 轴 IMU —— UART 协议定义与帧解析。

依据《10轴 IMU 通信协议》（Hiwonder IMU，V1.0）

设备主动上报帧：
    0x55 0x55 | ID | LEN | DATA[LEN] | SUM
    SUM = (0x55 + 0x55 + ID + LEN + DATA 各字节) & 0xFF     不含 SUM 自身
    多字节数据一律小端（低字节在前）

配置命令帧：
    0x55 0xAF | ID | LEN | DATA[LEN] | SUM
    读命令 ID = 原 ID | 0x80，LEN = 0

数据帧 ID：
    0x01 姿态角            LEN = 6   roll/pitch/yaw  int16
    0x02 四元数            LEN = 8   q0/q1/q2/q3     int16
    0x03 加速度+角速度+温度 LEN = 14  ax/ay/az/gx/gy/gz/gyroTemp int16
    0x04 磁力计            LEN = 6   mx/my/mz        int16
    0x05 气压+高度+气压温度 LEN = 10  pressure u32 / altitude int32 / baroTemp int16
"""

from __future__ import annotations

import struct

# ---------------------------------------------------------------- 常量定义
HEADER = 0x55
CFG_HEADER = 0xAF

FRAME_ATTITUDE = 0x01
FRAME_QUATERNION = 0x02
FRAME_MOTION = 0x03        # 加速度 + 角速度 + 陀螺仪温度
FRAME_MAG = 0x04
FRAME_BARO = 0x05

# 各帧 DATA 长度（用于长度校验与重同步）
FRAME_DATA_LEN = {
    FRAME_ATTITUDE: 6,
    FRAME_QUATERNION: 8,
    FRAME_MOTION: 14,
    FRAME_MAG: 6,
    FRAME_BARO: 10,
}

# ------------------------------------------------------------ 量程换算系数
# ACCFSR 编码 -> LSB/g
ACC_LSB_PER_G = {0x00: 16384.0, 0x01: 8192.0, 0x02: 4096.0, 0x03: 2048.0}
ACC_RANGE_G = {0x00: 2, 0x01: 4, 0x02: 8, 0x03: 16}

# GYROFSR 编码 -> LSB/(deg/s)
GYRO_LSB_PER_DPS = {0x00: 128.0, 0x01: 64.0, 0x02: 32.0, 0x03: 16.0}
GYRO_RANGE_DPS = {0x00: 256, 0x01: 512, 0x02: 1024, 0x03: 2048}

# 出厂默认：加速度 4G（ACCFSR=0x01）、陀螺仪 1024dps（GYROFSR=0x02）
DEFAULT_ACCFSR = 0x01
DEFAULT_GYROFSR = 0x02

# 配置寄存器
REG_SAVE = 0x00
REG_SENCAL = 0x01
REG_SENSTA = 0x02
REG_GYROFSR = 0x03
REG_ACCFSR = 0x04
REG_GYROBW = 0x05
REG_ACCBW = 0x06
REG_BAUD = 0x07
REG_RETURNSET = 0x08
REG_RETURNSET2 = 0x09
REG_RETURNRATE = 0x0A
REG_REPORTCTRL = 0x0F
REG_RESET = 0x7F


# ------------------------------------------------------------ 基础工具函数
def checksum(data: bytes) -> int:
    """逐字节累加取低 8 位。"""
    return sum(data) & 0xFF


def build_cmd(reg_id: int, data: bytes = b"") -> bytes:
    """构造配置命令帧：55 AF ID LEN DATA SUM。"""
    body = bytes([HEADER, CFG_HEADER, reg_id & 0xFF, len(data)]) + bytes(data)
    return body + bytes([checksum(body)])


def build_read_cmd(reg_id: int) -> bytes:
    """构造读寄存器命令（ID | 0x80，无 DATA）。"""
    return build_cmd(reg_id | 0x80, b"")


def _i16(buf: bytes) -> list:
    """按小端解出 n 个 int16。"""
    n = len(buf) // 2
    return list(struct.unpack("<%dh" % n, buf[: n * 2]))


# ---------------------------------------------------------------- 帧解析器
class ImuFrameParser:
    """串口字节流 -> 完整数据帧的流式解析器。

    用法::

        p = ImuFrameParser()
        for fid, data in p.feed(ser.read(256)):
            ...
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames_ok = 0
        self.frames_bad_sum = 0
        self.bytes_dropped = 0

    def feed(self, chunk: bytes):
        """喂入新字节，返回 [(frame_id, data_bytes), ...]。"""
        self._buf.extend(chunk)
        out = []
        while True:
            # 1) 定位帧头 55 55
            idx = self._find_header()
            if idx < 0:
                # 只保留末尾可能构成半个帧头的 1 个字节
                if self._buf and self._buf[-1] == HEADER:
                    self.bytes_dropped += len(self._buf) - 1
                    del self._buf[:-1]
                else:
                    self.bytes_dropped += len(self._buf)
                    self._buf.clear()
                break
            if idx > 0:
                self.bytes_dropped += idx
                del self._buf[:idx]

            # 2) 至少要有 帧头2 + ID + LEN
            if len(self._buf) < 4:
                break
            fid = self._buf[2]
            dlen = self._buf[3]
            total = 4 + dlen + 1

            # 长度合理性校验，非法则丢一个字节重新同步
            expect = FRAME_DATA_LEN.get(fid)
            if expect is not None and dlen != expect:
                self.bytes_dropped += 1
                del self._buf[0]
                continue
            if dlen > 64:
                self.bytes_dropped += 1
                del self._buf[0]
                continue

            # 3) 数据未收全
            if len(self._buf) < total:
                break

            frame = bytes(self._buf[:total])
            if checksum(frame[: total - 1]) != frame[total - 1]:
                self.frames_bad_sum += 1
                self.bytes_dropped += 1
                del self._buf[0]
                continue

            out.append((fid, frame[4: total - 1]))
            self.frames_ok += 1
            del self._buf[:total]
        return out

    def _find_header(self) -> int:
        """返回 buffer 中第一个 55 55 的下标，没有则 -1。"""
        i = 0
        n = len(self._buf)
        while i < n - 1:
            if self._buf[i] == HEADER and self._buf[i + 1] == HEADER:
                return i
            i += 1
        return -1


# ---------------------------------------------------------------- 帧解码器
def decode_attitude(data: bytes) -> dict:
    """0x01 姿态角帧 -> 单位：度。"""
    roll, pitch, yaw = _i16(data[:6])
    return {
        "roll": roll * 180.0 / 32768.0,
        "pitch": pitch * 180.0 / 32768.0,
        "yaw": yaw * 180.0 / 32768.0,
    }


def decode_quaternion(data: bytes) -> dict:
    """0x02 四元数帧（归一化到 ±1）。"""
    q0, q1, q2, q3 = _i16(data[:8])
    return {
        "q0": q0 / 32768.0,
        "q1": q1 / 32768.0,
        "q2": q2 / 32768.0,
        "q3": q3 / 32768.0,
    }


def decode_motion(data: bytes, accfsr: int = DEFAULT_ACCFSR,
                  gyrofsr: int = DEFAULT_GYROFSR) -> dict:
    """0x03 加速度/角速度/陀螺仪温度帧。

    加速度: g   = lsb / LSB_per_g
    角速度: dps = lsb / LSB_per_dps
    温度:   ℃  = int16(gyroTemp) / 256
    """
    ax, ay, az, gx, gy, gz, t = _i16(data[:14])
    acc_k = ACC_LSB_PER_G.get(accfsr, 8192.0)
    gyro_k = GYRO_LSB_PER_DPS.get(gyrofsr, 32.0)
    return {
        "acc_lsb": (ax, ay, az),
        "acc": (ax / acc_k, ay / acc_k, az / acc_k),
        "gyro_lsb": (gx, gy, gz),
        "gyro": (gx / gyro_k, gy / gyro_k, gz / gyro_k),
        "temperature": t / 256.0,
    }


def decode_mag(data: bytes) -> dict:
    """0x04 磁力计帧（原始 LSB，物理单位需磁力计标定后才可得）。"""
    mx, my, mz = _i16(data[:6])
    return {"mag_lsb": (mx, my, mz)}


def decode_baro(data: bytes) -> dict:
    """0x05 气压/高度/气压计温度帧。

    pressure : uint32，单位 Pa（原厂 10 轴固件）
    altitude : int32，单位 cm
    baroTemp : int16，单位 0.01 ℃
    """
    pressure = struct.unpack("<I", data[0:4])[0]
    altitude = struct.unpack("<i", data[4:8])[0]
    baro_t = _i16(data[8:10])[0]
    return {
        "pressure": pressure,
        "altitude": altitude / 100.0,
        "baro_temperature": baro_t / 100.0,
    }


# ------------------------------------------------- 配置命令应答帧（0xAF）
def parse_cfg_reply(raw: bytes):
    """解析配置命令应答：55 AF ID LEN DATA SUM，返回 (id, data) 或 None。"""
    if len(raw) < 5 or raw[0] != HEADER or raw[1] != CFG_HEADER:
        return None
    fid, dlen = raw[2], raw[3]
    total = 4 + dlen + 1
    if len(raw) < total:
        return None
    if checksum(raw[: total - 1]) != raw[total - 1]:
        return None
    return fid, raw[4: total - 1]

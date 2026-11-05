# -*- coding: utf-8 -*-
"""IMU 零偏检测与陀螺仪校准（可选工具）。

用法：
    python imu_calibrate.py -p COM3                    # 仅测量静止零偏，不写设备
    python imu_calibrate.py -p COM3 --send-sencal      # 测量后执行陀螺仪校准并 SAVE
    python imu_calibrate.py -p COM3 --seconds 15

注意：执行 --send-sencal 时设备必须完全静止放在水平面，否则会把运动当成零偏扣掉。
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial，请先执行：pip install pyserial")

import imu_protocol as proto
from imu_reader import autodetect_port

# SENCAL 参数：0x00 陀螺仪校准 / 0x01 磁力计校准 / 0x03 加速度六面校准
SENCAL_GYRO = 0x00


def measure(ser, seconds):
    parser = proto.ImuFrameParser()
    acc_raw, gyro_raw, temps, mags = [], [], [], []
    t_end = time.time() + seconds
    while time.time() < t_end:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        for fid, data in parser.feed(chunk):
            if fid == proto.FRAME_MOTION:
                ax, ay, az, gx, gy, gz, t = proto._i16(data[:14])
                acc_raw.append((ax, ay, az))
                gyro_raw.append((gx, gy, gz))
                temps.append(t / 256.0)
            elif fid == proto.FRAME_MAG:
                mags.append(proto.decode_mag(data)["mag_lsb"])
    return acc_raw, gyro_raw, temps, mags, parser


def mean3(rows):
    if not rows:
        return None
    return tuple(statistics.fmean(r[i] for r in rows) for i in range(3))


def main() -> int:
    ap = argparse.ArgumentParser(description="IMU 零偏检测 / 陀螺仪校准")
    ap.add_argument("-p", "--port", help="串口名（默认自动扫描）")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=10.0, help="静止采样时长，默认 10s")
    ap.add_argument("--accfsr", type=lambda x: int(x, 0), default=proto.DEFAULT_ACCFSR)
    ap.add_argument("--gyrofsr", type=lambda x: int(x, 0), default=proto.DEFAULT_GYROFSR)
    ap.add_argument("--send-sencal", action="store_true",
                    help="对设备下发陀螺仪校准命令（55 AF 01 01 00 xx）并 SAVE")
    args = ap.parse_args()

    port = args.port or autodetect_port()
    if not port:
        print("未找到串口")
        return 1

    ser = serial.Serial(port, args.baud, timeout=0.05)
    print("请在 %.0f 秒内保持设备静止、水平放置…" % args.seconds)
    time.sleep(0.5)
    ser.reset_input_buffer()
    acc_raw, gyro_raw, temps, mags, parser = measure(ser, args.seconds)

    if not gyro_raw:
        print("未采集到 0x03 帧，无法评估零偏")
        ser.close()
        return 2

    acc_k = proto.ACC_LSB_PER_G.get(args.accfsr, 8192.0)
    gyro_k = proto.GYRO_LSB_PER_DPS.get(args.gyrofsr, 32.0)
    acc_m = mean3(acc_raw)
    gyro_m = mean3(gyro_raw)

    print("\n采样 %d 帧，解析有效帧 %d，校验错误 %d"
          % (len(gyro_raw), parser.frames_ok, parser.frames_bad_sum))
    print("加速度零偏 (g)     : X%+.4f Y%+.4f Z%+.4f  | 合模 %+.4f"
          % (acc_m[0] / acc_k, acc_m[1] / acc_k, acc_m[2] / acc_k,
             (sum(v * v for v in acc_m) ** 0.5) / acc_k))
    print("陀螺仪零偏 (deg/s) : X%+.3f Y%+.3f Z%+.3f  | 合模 %.3f"
          % (gyro_m[0] / gyro_k, gyro_m[1] / gyro_k, gyro_m[2] / gyro_k,
             (sum(v * v for v in gyro_m) ** 0.5) / gyro_k))
    print("陀螺仪零偏 (LSB)   : X%+.1f Y%+.1f Z%+.1f" % gyro_m)
    print("陀螺仪温度         : %.2f ℃（范围 %.2f ~ %.2f）"
          % (statistics.fmean(temps), min(temps), max(temps)))
    if mags:
        m = mean3(mags)
        print("磁力计均值 (LSB)   : X%+.1f Y%+.1f Z%+.1f（%d 帧）" % (m[0], m[1], m[2], len(mags)))
    else:
        print("磁力计             : 未收到 0x04 帧")

    if args.send_sencal:
        cmd = proto.build_cmd(proto.REG_SENCAL, bytes([SENCAL_GYRO]))
        print("\n下发陀螺仪校准命令：%s" % cmd.hex())
        ser.reset_input_buffer()
        ser.write(cmd)
        ser.flush()
        time.sleep(2.0)
        raw = ser.read(ser.in_waiting or 1)
        parsed = proto.parse_cfg_reply(raw) if raw else None
        if parsed:
            print("应答：ID=0x%02X DATA=%s%s"
                  % (parsed[0], parsed[1].hex(),
                     "（0x00 成功）" if parsed[1][:1] == b"\x00" else ""))
        else:
            print("未解析到应答（原始：%s）" % (raw.hex() if raw else "空"))
        save = proto.build_cmd(proto.REG_SAVE, b"\x00")
        ser.write(save)
        ser.flush()
        print("已下发 SAVE：%s" % save.hex())

    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

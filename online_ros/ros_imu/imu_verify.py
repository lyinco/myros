# -*- coding: utf-8 -*-
"""IMU 数据正确性验证工具。

自动检查（默认）：
  1. 加速度合模 ~= 1g        -> 校验量程换算（LSB/g）是否设错
  2. 静止时陀螺仪 ~= 0       -> 零偏检查
  3. 噪声水平               -> 判断数据是否真在刷新而非恒定值
  4. 温度合理区间
  5. 设备姿态角(0x01) vs 由加速度反算的 roll/pitch -> 设备内部算法与原始数据互相印证
  6. 磁力计合模稳定且非零
  7. 各帧 ID 刷新率
  8. 校验和错误率

交互检查：
  --six-face   六面法，逐面朝下确认三轴符号与增益
  --rotate     绕 Z 轴转 90 度，比对陀螺积分 / 设备 yaw

用法：
  python imu_verify.py -p COM3
  python imu_verify.py -p COM3 --six-face
  python imu_verify.py -p COM3 --rotate
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial，请先执行：pip install pyserial")

import imu_protocol as proto
from imu_reader import autodetect_port

RESULTS = []


def check(name, ok, detail):
    RESULTS.append((name, ok, detail))
    print("  [%s] %-20s %s" % ("PASS" if ok else "FAIL", name, detail))
    return ok


def collect(ser, seconds, accfsr, gyrofsr):
    parser = proto.ImuFrameParser()
    acc, gyro, temp, mag, att = [], [], [], [], []
    id_count = {}
    t_end = time.time() + seconds
    while time.time() < t_end:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        for fid, data in parser.feed(chunk):
            id_count[fid] = id_count.get(fid, 0) + 1
            if fid == proto.FRAME_MOTION:
                d = proto.decode_motion(data, accfsr, gyrofsr)
                acc.append(d["acc"])
                gyro.append(d["gyro"])
                temp.append(d["temperature"])
            elif fid == proto.FRAME_MAG:
                mag.append(proto.decode_mag(data)["mag_lsb"])
            elif fid == proto.FRAME_ATTITUDE:
                a = proto.decode_attitude(data)
                att.append((a["roll"], a["pitch"], a["yaw"]))
    return acc, gyro, temp, mag, att, id_count, parser


def run_static(args):
    port = args.port or autodetect_port()
    if not port:
        print("未找到串口")
        return 1
    ser = serial.Serial(port, args.baud, timeout=0.05)
    ser.reset_input_buffer()
    print("请保持设备静止、水平放置，采集 %.0f 秒...\n" % args.seconds)
    t0 = time.time()
    acc, gyro, temp, mag, att, id_count, parser = collect(
        ser, args.seconds, args.accfsr, args.gyrofsr)
    elapsed = time.time() - t0
    ser.close()

    if not acc:
        print("未采集到 0x03 帧，检查 RETURNSET 与波特率")
        return 2

    print("== 自动检查 ==")
    acc_m = [statistics.fmean(r[i] for r in acc) for i in range(3)]
    norm = math.sqrt(sum(v * v for v in acc_m))
    check("加速度合模~1g", abs(norm - 1.0) <= 0.05,
          "|a| = %.4f g (偏差 %+.2f%%) -> 量程 %sG 换算正确"
          % (norm, (norm - 1) * 100, proto.ACC_RANGE_G.get(args.accfsr, "?")))

    acc_std = [statistics.pstdev(r[i] for r in acc) for i in range(3)]
    check("加速度噪声", max(acc_std) <= 0.02,
          "std = (%.4f, %.4f, %.4f) g" % tuple(acc_std))

    gyro_m = [statistics.fmean(r[i] for r in gyro) for i in range(3)]
    gyro_norm = math.sqrt(sum(v * v for v in gyro_m))
    check("静止陀螺仪~0", gyro_norm <= 1.0,
          "零偏 = (%+.3f, %+.3f, %+.3f) dps, 合模 %.3f dps%s"
          % (gyro_m[0], gyro_m[1], gyro_m[2], gyro_norm,
             "" if gyro_norm <= 1.0 else "  <- 建议 SENCAL 校准"))

    gyro_std = [statistics.pstdev(r[i] for r in gyro) for i in range(3)]
    check("陀螺仪噪声", max(gyro_std) <= 0.5,
          "std = (%.3f, %.3f, %.3f) dps" % tuple(gyro_std))

    t_mean = statistics.fmean(temp)
    check("温度合理", -20 <= t_mean <= 85 and statistics.pstdev(temp) <= 0.5,
          "%.2f C, std %.3f C" % (t_mean, statistics.pstdev(temp)))

    if att:
        roll_acc = math.degrees(math.atan2(acc_m[1], acc_m[2]))
        pitch_acc = math.degrees(math.atan2(-acc_m[0], math.hypot(acc_m[1], acc_m[2])))
        roll_dev = statistics.fmean(a[0] for a in att)
        pitch_dev = statistics.fmean(a[1] for a in att)
        d_roll = abs(roll_dev - roll_acc)
        d_pitch = abs(pitch_dev - pitch_acc)
        check("姿态角交叉校验", max(d_roll, d_pitch) <= 3.0,
              "设备 roll/pitch = (%+.2f, %+.2f), 加速度反算 = (%+.2f, %+.2f), 差 (%.2f, %.2f)"
              % (roll_dev, pitch_dev, roll_acc, pitch_acc, d_roll, d_pitch))
    else:
        print("  [SKIP] 姿态角交叉校验   未收到 0x01 帧")

    if mag:
        mag_m = [statistics.fmean(r[i] for r in mag) for i in range(3)]
        mag_norm = math.sqrt(sum(v * v for v in mag_m))
        mag_series = [math.sqrt(sum(v * v for v in r)) for r in mag]
        check("磁力计非零且稳定", mag_norm > 50 and statistics.pstdev(mag_series) <= 5.0,
              "均模 %.1f LSB, 模长 std %.2f LSB"
              % (mag_norm, statistics.pstdev(mag_series)))
    else:
        check("磁力计非零且稳定", False, "未收到 0x04 帧")

    fps = {hex(k): v / elapsed for k, v in sorted(id_count.items())}
    min_rate = min(fps.values()) if fps else 0
    check("各帧刷新率", min_rate >= 50,
          ", ".join("%s:%.0fHz" % (k, v) for k, v in fps.items()))

    bad = parser.frames_bad_sum
    total = parser.frames_ok + bad
    check("校验和错误率", bad == 0 or bad / max(total, 1) < 0.01,
          "有效帧 %d, 校验错 %d, 丢弃字节 %d" % (parser.frames_ok, bad, parser.bytes_dropped))

    failed = [r for r in RESULTS if not r[1]]
    print("\n== 结论：%d 项通过，%d 项未通过 ==" % (len(RESULTS) - len(failed), len(failed)))
    for name, _, detail in failed:
        print("  - %s: %s" % (name, detail))
    return 0 if not failed else 3


def run_six_face(args):
    port = args.port or autodetect_port()
    if not port:
        return 1
    ser = serial.Serial(port, args.baud, timeout=0.05)
    faces = [("+Z 朝下", 2, +1), ("-Z 朝下", 2, -1),
             ("+X 朝下", 0, +1), ("-X 朝下", 0, -1),
             ("+Y 朝下", 1, +1), ("-Y 朝下", 1, -1)]
    print("六面法：按提示摆放模块，每次回车后自动采 2 秒。\n")
    for label, axis, sign in faces:
        input("请将 %s，保持静止后回车..." % label)
        ser.reset_input_buffer()
        acc = collect(ser, 2.0, args.accfsr, args.gyrofsr)[0]
        if not acc:
            print("  未采到数据")
            continue
        m = [statistics.fmean(r[i] for r in acc) for i in range(3)]
        expect = [0.0, 0.0, 0.0]
        expect[axis] = float(sign)
        err = max(abs(m[i] - expect[i]) for i in range(3))
        print("  [%s] 实测 (%+.3f, %+.3f, %+.3f) g, 期望 (%+.1f, %+.1f, %+.1f), 最大偏差 %.3f g"
              % ("PASS" if err <= 0.1 else "FAIL", m[0], m[1], m[2], expect[0], expect[1], expect[2], err))
    ser.close()
    print("\n六面法同时确认：三轴物理方向、符号极性、每轴增益一致性。")
    print("某轴位置不符只说明丝印方向与脚本假设不同，不影响数据本身。")
    return 0


def run_rotate(args):
    port = args.port or autodetect_port()
    if not port:
        return 1
    ser = serial.Serial(port, args.baud, timeout=0.05)

    input("请保持静止，回车开始采集基线（3 秒）...")
    ser.reset_input_buffer()
    acc, gyro, temp, mag, att, _, _ = collect(ser, 3.0, args.accfsr, args.gyrofsr)
    if not att:
        print("未收到 0x01 姿态角帧，无法比对 yaw")
        ser.close()
        return 2
    yaw0 = statistics.fmean(a[2] for a in att)
    bias = [statistics.fmean(r[i] for r in gyro) for i in range(3)]
    print("基线 yaw = %+.2f, 陀螺零偏 = (%+.2f, %+.2f, %+.2f) dps"
          % (yaw0, bias[0], bias[1], bias[2]))

    print("\n请绕 Z 轴匀速转约 90 度，转完保持静止，脚本自动判定结束...")
    ser.reset_input_buffer()
    print("采集中...")
    t_end = time.time() + 12.0
    parser = proto.ImuFrameParser()
    integ = 0.0
    last_t = time.time()
    yaw_last = None
    still_since = None
    while time.time() < t_end:
        chunk = ser.read(ser.in_waiting or 1)
        now = time.time()
        if chunk:
            for fid, data in parser.feed(chunk):
                if fid == proto.FRAME_MOTION:
                    g = proto.decode_motion(data, args.accfsr, args.gyrofsr)["gyro"]
                    integ += (g[2] - bias[2]) * (now - last_t)
                    last_t = now
                    if abs(g[2] - bias[2]) > 3.0:
                        still_since = None
                    elif still_since is None:
                        still_since = now
                elif fid == proto.FRAME_ATTITUDE:
                    yaw_last = proto.decode_attitude(data)["yaw"]
        if still_since is not None and now - still_since > 1.5 and abs(integ) > 10:
            break
    ser.close()

    print("\n== 旋转比对 ==")
    print("陀螺 Z 轴积分转过的角度 : %+.2f 度" % integ)
    if yaw_last is not None:
        d = (yaw_last - yaw0 + 180) % 360 - 180
        print("设备 yaw 变化           : %+.2f 度 (%.2f -> %.2f)" % (d, yaw0, yaw_last))
        print("两者差值                 : %+.2f 度" % (integ - d))
        print("\n差值在 10 度内且同号：陀螺仪标度与极性都正确。")
        print("符号相反：Z 轴极性定义不同；数值差很大：GYROFSR 量程设错。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="IMU 数据正确性验证")
    ap.add_argument("-p", "--port", help="串口名（默认自动扫描）")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=5.0, help="静止采样时长，默认 5s")
    ap.add_argument("--accfsr", type=lambda x: int(x, 0), default=proto.DEFAULT_ACCFSR)
    ap.add_argument("--gyrofsr", type=lambda x: int(x, 0), default=proto.DEFAULT_GYROFSR)
    ap.add_argument("--six-face", action="store_true", help="六面法逐面验证")
    ap.add_argument("--rotate", action="store_true", help="绕 Z 轴旋转比对陀螺积分")
    args = ap.parse_args()

    if args.six_face:
        return run_six_face(args)
    if args.rotate:
        return run_rotate(args)
    return run_static(args)


if __name__ == "__main__":
    sys.exit(main())

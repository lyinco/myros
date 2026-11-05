# -*- coding: utf-8 -*-
"""IMU 噪声参数标定（Allan 方差），直接输出 VINS-Fusion 可用的 imu 参数。

VINS-Fusion 的 5 个参数：
  acc_n : 加速度计测量噪声密度        [m/s^2/sqrt(Hz)]
  gyr_n : 陀螺仪测量噪声密度          [rad/s/sqrt(Hz)]
  acc_w : 加速度计零偏随机游走        [m/s^3/sqrt(Hz)]   (与 imu_utils 输出口径一致)
  gyr_w : 陀螺仪零偏随机游走          [rad/s^2/sqrt(Hz)]
  g_norm: 当地重力加速度大小          [m/s^2]

用法：
  1) 采集（设备静止，建议 2 小时，最少 45 分钟）
     python imu_allan.py -p COM3 --record 7200 --csv imu_static.csv
  2) 分析
     python imu_allan.py --analyze imu_static.csv
  3) 一气呵成
     python imu_allan.py -p COM3 --record 7200 --analyze

原理：
  白噪声    Allan 偏差 sigma(tau) = N / sqrt(tau)  ->  N = sigma(tau)*sqrt(tau)，取 tau<=1s 段中位数
  随机游走  sigma(tau) = K * sqrt(tau/3)           ->  K = sigma(tau)*sqrt(3/tau)，取长 tau 段中位数
  tau=1s 时 sigma(1s) 就是噪声密度 N，与 imu_utils 的口径一致。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial，请先执行：pip install pyserial")

import imu_protocol as proto
from imu_reader import autodetect_port

G0 = 9.80665


# ------------------------------------------------------------------ 采集
def record(args) -> str:
    port = args.port or autodetect_port()
    if not port:
        sys.exit("未找到可用串口")
    ser = serial.Serial(port, args.baud, timeout=0.05)
    parser = proto.ImuFrameParser()
    ser.reset_input_buffer()

    path = args.csv or "imu_static.csv"
    total = args.record
    print("开始采集 %.0f 秒（%.1f 分钟）-> %s" % (total, total / 60.0, path))
    print("期间请勿触碰设备、勿让桌面震动。Ctrl+C 可提前结束并保留已采数据。\n")

    t0 = time.time()
    n = 0
    last_report = t0
    fh = open(path, "w", encoding="utf-8")
    fh.write("time_s,acc_x_g,acc_y_g,acc_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,temp_c\n")
    try:
        while True:
            now = time.time()
            elapsed = now - t0
            if elapsed >= total:
                break
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                for fid, data in parser.feed(chunk):
                    if fid != proto.FRAME_MOTION:
                        continue
                    d = proto.decode_motion(data, args.accfsr, args.gyrofsr)
                    a, g = d["acc"], d["gyro"]
                    fh.write("%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.3f\n"
                             % (elapsed, a[0], a[1], a[2], g[0], g[1], g[2],
                                d["temperature"]))
                    n += 1
            if now - last_report >= 5.0:
                last_report = now
                if n:
                    fh.flush()
                print("  %7.1fs / %.0fs  已采 %d 帧 (%.1fHz)  剩余 %.0fs"
                      % (elapsed, total, n, n / max(elapsed, 1e-6),
                         max(total - elapsed, 0)), end="\r")
    except KeyboardInterrupt:
        print("\n提前结束采集")
    finally:
        ser.close()
        fh.close()

    elapsed = max(time.time() - t0, 1e-6)
    print("\n采集完成：%d 帧，%.1f 秒，平均 %.1f Hz -> %s"
          % (n, elapsed, n / elapsed, os.path.abspath(path)))
    if n < 1000:
        print("警告：样本太少，Allan 方差结果不可信")
    return path


# ------------------------------------------------------------- Allan 方差
def overlapping_adev(x: np.ndarray, dt: float, taus: np.ndarray) -> np.ndarray:
    """重叠 Allan 偏差（输入为速率信号）。"""
    theta = np.cumsum(x) * dt
    adev = np.full(len(taus), np.nan)
    for i, tau in enumerate(taus):
        m = int(round(tau / dt))
        if m < 1 or 3 * m > len(x):
            continue
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        avar = float(np.sum(d * d)) / (2.0 * len(d)) / (m * dt) ** 2
        adev[i] = math.sqrt(max(avar, 0.0))
    return adev


def estimate_noise(x: np.ndarray, dt: float):
    """返回 (白噪声密度 N, 随机游走 K, taus, adev)。"""
    n = len(x)
    max_m = max(n // 9, 1)
    taus = np.unique(np.logspace(0, math.log10(max_m), 60).astype(int)) * dt
    adev = overlapping_adev(x, dt, taus)
    ok = ~np.isnan(adev)
    taus, adev = taus[ok], adev[ok]
    if len(taus) == 0:
        return float("nan"), float("nan"), taus, adev

    short = taus <= 1.0
    if short.sum() >= 2:
        n_est = float(np.median(adev[short] * np.sqrt(taus[short])))
    else:
        n_est = float(adev[0] * math.sqrt(taus[0]))

    long_sel = taus >= max(50.0, taus.max() / 5.0)
    if long_sel.sum() >= 2:
        k_est = float(np.median(adev[long_sel] * np.sqrt(3.0 / taus[long_sel])))
    else:
        k_est = float(adev[-1] * math.sqrt(3.0 / taus[-1]))
    return n_est, k_est, taus, adev


def analyze(path: str) -> int:
    if not os.path.exists(path):
        sys.exit("找不到数据文件：%s" % path)
    raw = np.genfromtxt(path, delimiter=",", names=True)
    if raw.ndim == 0:
        sys.exit("数据文件为空或格式不对")
    t = np.atleast_1d(raw["time_s"])
    if len(t) < 1000:
        print("警告：样本少于 1000，结果仅供参考")
    good = np.concatenate(([True], np.diff(t) > 0))
    raw = raw[good]
    t = t[good]

    dt = float(np.median(np.diff(t)))
    fs = 1.0 / dt
    n = len(t)
    dur = float(t[-1] - t[0])
    print("数据：%d 帧，%.1f 秒（%.1f 分钟），采样率 %.1f Hz\n"
          % (n, dur, dur / 60.0, fs))
    if dur < 600:
        print("提示：数据不足 10 分钟，随机游走项 acc_w/gyr_w 误差较大，"
              "理想时长 1~2 小时。\n")

    acc = np.column_stack([raw["acc_x_g"], raw["acc_y_g"], raw["acc_z_g"]]) * G0
    gyr = np.column_stack([raw["gyro_x_dps"], raw["gyro_y_dps"],
                           raw["gyro_z_dps"]]) * math.pi / 180.0

    print("%-9s %14s %16s" % ("通道", "白噪声 N", "随机游走 K"))
    print("%-9s %14s %16s" % ("", "unit/sqrt(Hz)", "unit/s^1.5"))
    acc_n, acc_w, gyr_n, gyr_w = [], [], [], []
    for i, ax in enumerate("XYZ"):
        n_a, k_a, _, _ = estimate_noise(acc[:, i], dt)
        n_g, k_g, _, _ = estimate_noise(gyr[:, i], dt)
        acc_n.append(n_a)
        acc_w.append(k_a)
        gyr_n.append(n_g)
        gyr_w.append(k_g)
        print("acc_%s    %14.6e %16.6e" % (ax, n_a, k_a))
        print("gyr_%s    %14.6e %16.6e" % (ax, n_g, k_g))

    a_n = float(np.nanmean(acc_n))
    a_w = float(np.nanmean(acc_w))
    g_n = float(np.nanmean(gyr_n))
    g_w = float(np.nanmean(gyr_w))
    g_meas = float(np.linalg.norm(acc.mean(axis=0)))
    bias = gyr.mean(axis=0) * 180.0 / math.pi

    print("\n静止均值：acc = (%.4f, %.4f, %.4f) m/s^2，合模 %.4f m/s^2"
          % (acc[:, 0].mean(), acc[:, 1].mean(), acc[:, 2].mean(), g_meas))
    print("          gyro 零偏 = (%+.3f, %+.3f, %+.3f) deg/s"
          % (bias[0], bias[1], bias[2]))
    print("          各轴噪声：acc = (%.4f, %.4f, %.4f) m/s^2, "
          "gyro = (%.4f, %.4f, %.4f) deg/s"
          % (acc.std(axis=0)[0], acc.std(axis=0)[1], acc.std(axis=0)[2],
             gyr.std(axis=0)[0] * 180 / math.pi, gyr.std(axis=0)[1] * 180 / math.pi,
             gyr.std(axis=0)[2] * 180 / math.pi))

    print("\n================ VINS-Fusion 配置 ================")
    print("#imu parameters       The more accurate parameters you provide, the better performance")
    print("acc_n: %.6g          # accelerometer measurement noise standard deviation." % a_n)
    print("gyr_n: %.6g          # gyroscope measurement noise standard deviation." % g_n)
    print("acc_w: %.6g          # accelerometer bias random work noise standard deviation." % a_w)
    print("gyr_w: %.6g          # gyroscope bias random work noise standard deviation." % g_w)
    print("g_norm: %.5f     # gravity magnitude" % g_meas)
    print("==================================================")
    print("g_norm 建议用当地重力标准值（见 README 公式）；上面是本次静止实测合模，")
    print("偏差小于 0.05 时可直接采用。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="IMU Allan 方差标定，输出 VINS-Fusion imu 参数")
    ap.add_argument("-p", "--port", help="串口名（默认自动扫描）")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--accfsr", type=lambda x: int(x, 0), default=proto.DEFAULT_ACCFSR)
    ap.add_argument("--gyrofsr", type=lambda x: int(x, 0), default=proto.DEFAULT_GYROFSR)
    ap.add_argument("--record", type=float, default=0.0,
                    help="采集时长（秒），0 表示不采集")
    ap.add_argument("--csv", default="imu_static.csv", help="数据文件路径")
    ap.add_argument("--analyze", nargs="?", const="@csv", default=None,
                    help="分析已有 CSV（不带参数则分析 --csv 指定文件）")
    args = ap.parse_args()

    path = args.csv
    if args.record > 0:
        path = record(args)
        if args.analyze is None:
            args.analyze = "@csv"
    if args.analyze is not None:
        target = path if args.analyze == "@csv" else args.analyze
        return analyze(target)
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())

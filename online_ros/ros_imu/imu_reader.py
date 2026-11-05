# -*- coding: utf-8 -*-
"""从 USB 串口读取 Hiwonder 10 轴 IMU 数据。

输出四类数据：
  1) 加速度计  acc  (g)      <- 0x03 帧
  2) 陀螺仪    gyro (deg/s)  <- 0x03 帧
  3) 磁力计    mag  (原始 LSB) <- 0x04 帧
  4) 温度计    temp (degC)   <- 0x03 帧的陀螺仪温度（0x05 帧的气压计温度可选）

用法示例：
    python imu_reader.py                          # 自动扫描 USB 串口，115200
    python imu_reader.py -p COM3                  # 指定串口
    python imu_reader.py -p /dev/ttyUSB0 -b 460800
    python imu_reader.py --csv imu_log.csv -t 30  # 录 30 秒到 CSV
    python imu_reader.py --query                  # 先回读设备量程再换算
    python imu_reader.py --json                   # 每行输出一条 JSON
    python imu_reader.py --no-barometer           # 不关心气压帧

依赖：pip install pyserial
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    sys.exit("缺少 pyserial，请先执行：pip install pyserial")

import imu_protocol as proto


# ------------------------------------------------------------ 串口自动发现
USB_SERIAL_HINTS = ("CH340", "CH341", "CP210", "FT232", "USB-SERIAL",
                    "USB Serial", "Silicon Labs", "UART", "ACM")


def autodetect_port() -> str | None:
    """按关键字挑选 USB 转串口设备，找不到就返回第一个可用串口。"""
    ports = list(list_ports.comports())
    if not ports:
        return None
    for p in ports:
        text = " ".join(filter(None, (p.device, p.description, p.manufacturer)))
        if any(h.lower() in text.lower() for h in USB_SERIAL_HINTS):
            return p.device
    return ports[0].device


def print_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("未发现任何串口设备")
        return
    print("可用串口：")
    for p in ports:
        print("  %-10s %s" % (p.device, p.description))


# -------------------------------------------------------------- 配置回读
def query_config(ser: "serial.Serial") -> tuple[int, int]:
    """回读 ACCFSR / GYROFSR，失败则退回出厂默认值。"""
    accfsr, gyrofsr = proto.DEFAULT_ACCFSR, proto.DEFAULT_GYROFSR
    for reg, name in ((proto.REG_ACCFSR, "ACCFSR"),
                      (proto.REG_GYROFSR, "GYROFSR")):
        try:
            ser.reset_input_buffer()
            ser.write(proto.build_read_cmd(reg))
            ser.flush()
            deadline = time.time() + 0.3
            buf = bytearray()
            reply = None
            while time.time() < deadline:
                buf.extend(ser.read(ser.in_waiting or 1))
                # 在缓冲里找 55 AF 应答
                i = buf.find(bytes([proto.HEADER, proto.CFG_HEADER]))
                if i >= 0:
                    reply = proto.parse_cfg_reply(bytes(buf[i:]))
                    if reply:
                        break
            if reply and reply[1]:
                val = reply[1][0]
                if name == "ACCFSR":
                    accfsr = val
                else:
                    gyrofsr = val
        except Exception as exc:  # 设备忙/无应答不影响主流程
            print("[warn] 读取 %s 失败：%s" % (name, exc))
    return accfsr, gyrofsr


# ------------------------------------------------------------------ 输出
def format_line(state: dict, elapsed: float, hz: float) -> str:
    acc = state.get("acc")
    gyro = state.get("gyro")
    mag = state.get("mag_lsb")
    temp = state.get("temperature")

    acc_s = ("X%+7.3f Y%+7.3f Z%+7.3f" % acc) if acc else "     (无 0x03 帧)     "
    gyro_s = ("X%+8.2f Y%+8.2f Z%+8.2f" % gyro) if gyro else "      (无 0x03 帧)      "
    mag_s = ("X%+7d Y%+7d Z%+7d" % mag) if mag else "   (无 0x04 帧，6 轴?)   "
    temp_s = ("%6.2f" % temp) if temp is not None else "  --  "

    return ("[%7.2fs %5.1fHz] 加速度(%s)g | 陀螺仪(%s)dps | 磁力计(%s)LSB | 温度(%s)℃"
            % (elapsed, hz, acc_s, gyro_s, mag_s, temp_s))


def main() -> int:
    ap = argparse.ArgumentParser(
        description="读取 USB 串口上的 Hiwonder 10 轴 IMU 数据（加速度/陀螺仪/磁力计/温度）")
    ap.add_argument("-p", "--port", help="串口名，如 COM3 或 /dev/ttyUSB0（默认自动扫描）")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--accfsr", type=lambda x: int(x, 0), default=None,
                    help="加速度量程编码：0=2G 1=4G 2=8G 3=16G（默认按设备配置/4G）")
    ap.add_argument("--gyrofsr", type=lambda x: int(x, 0), default=None,
                    help="陀螺仪量程编码：0=256 1=512 2=1024 3=2048 dps（默认按设备配置/1024）")
    ap.add_argument("--query", action="store_true", help="打开串口后回读 ACCFSR/GYROFSR")
    ap.add_argument("-t", "--duration", type=float, default=0.0,
                    help="采集时长（秒），0 表示一直采集直到 Ctrl+C")
    ap.add_argument("--print-hz", type=float, default=5.0, help="终端刷新频率，默认 5Hz")
    ap.add_argument("--csv", help="将数据写入该 CSV 文件")
    ap.add_argument("--json", action="store_true", help="以 JSON 行格式输出")
    ap.add_argument("--raw", action="store_true", help="额外打印每帧原始十六进制")
    ap.add_argument("--list-ports", action="store_true", help="列出所有串口后退出")
    args = ap.parse_args()

    if args.list_ports:
        print_ports()
        return 0

    port = args.port or autodetect_port()
    if not port:
        print("未找到可用串口，请检查 USB 连接；可用 --list-ports 查看")
        return 1

    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
    except Exception as exc:
        print("打开串口 %s 失败：%s" % (port, exc))
        return 1

    accfsr = args.accfsr if args.accfsr is not None else proto.DEFAULT_ACCFSR
    gyrofsr = args.gyrofsr if args.gyrofsr is not None else proto.DEFAULT_GYROFSR
    if args.query:
        accfsr, gyrofsr = query_config(ser)
        ser.reset_input_buffer()

    print("已打开 %s @ %d，加速度量程 %sG（LSB/g=%.0f），陀螺仪量程 %sdps（LSB/dps=%.0f）"
          % (port, args.baud,
             proto.ACC_RANGE_G.get(accfsr, "?"), proto.ACC_LSB_PER_G.get(accfsr, 0),
             proto.GYRO_RANGE_DPS.get(gyrofsr, "?"), proto.GYRO_LSB_PER_DPS.get(gyrofsr, 0)))
    print("按 Ctrl+C 结束。\n")

    parser = proto.ImuFrameParser()
    state: dict = {}
    csv_fh = csv_writer = None
    if args.csv:
        csv_fh = open(args.csv, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_fh)
        csv_writer.writerow(["time_s", "acc_x_g", "acc_y_g", "acc_z_g",
                             "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
                             "mag_x_lsb", "mag_y_lsb", "mag_z_lsb",
                             "temp_c", "baro_temp_c", "baro_alt_m"])

    t0 = time.time()
    last_print = 0.0
    last_count = 0
    rate = 0.0
    motion_frames = 0
    mag_frames = 0
    new_motion = False
    try:
        while True:
            now = time.time()
            elapsed = now - t0
            if args.duration and elapsed >= args.duration:
                break

            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                for fid, data in parser.feed(chunk):
                    if args.raw:
                        print("RAW %02X %s" % (fid, data.hex()))
                    if fid == proto.FRAME_MOTION:
                        d = proto.decode_motion(data, accfsr, gyrofsr)
                        state.update(d)
                        motion_frames += 1
                        new_motion = True
                    elif fid == proto.FRAME_MAG:
                        state.update(proto.decode_mag(data))
                        mag_frames += 1
                    elif fid == proto.FRAME_ATTITUDE:
                        state.update(proto.decode_attitude(data))
                    elif fid == proto.FRAME_QUATERNION:
                        state.update(proto.decode_quaternion(data))
                    elif fid == proto.FRAME_BARO:
                        state.update(proto.decode_baro(data))

            # CSV 按帧写入（每个 0x03 帧一行）
            if csv_writer and new_motion:
                acc = state["acc"]
                gyro = state.get("gyro", (0.0, 0.0, 0.0))
                mag = state.get("mag_lsb", ("", "", ""))
                csv_writer.writerow([round(elapsed, 4),
                                     "%.4f" % acc[0], "%.4f" % acc[1], "%.4f" % acc[2],
                                     "%.3f" % gyro[0], "%.3f" % gyro[1], "%.3f" % gyro[2],
                                     mag[0], mag[1], mag[2],
                                     "%.2f" % state.get("temperature", 0.0),
                                     state.get("baro_temperature", ""),
                                     state.get("altitude", "")])
                new_motion = False

            # 终端刷新
            if args.print_hz > 0 and now - last_print >= 1.0 / args.print_hz:
                if parser.frames_ok != last_count:
                    rate = (parser.frames_ok - last_count) / max(now - last_print, 1e-6)
                    last_count = parser.frames_ok
                last_print = now
                if args.json:
                    out = {"t": round(elapsed, 4),
                           "acc": state.get("acc"),
                           "gyro": state.get("gyro"),
                           "mag": state.get("mag_lsb"),
                           "temp": state.get("temperature")}
                    print(json.dumps(out, ensure_ascii=False))
                else:
                    print(format_line(state, elapsed, rate))

    except KeyboardInterrupt:
        print("\n用户中断。")
    finally:
        ser.close()
        if csv_fh:
            csv_fh.close()
            print("CSV 已保存：%s" % args.csv)

    elapsed = max(time.time() - t0, 1e-6)
    print("\n统计：有效帧 %d（0x03 加速度/角速度 %d 帧，0x04 磁力计 %d 帧），"
          "校验错误 %d，丢弃字节 %d，平均 %.1f 帧/秒"
          % (parser.frames_ok, motion_frames, mag_frames,
             parser.frames_bad_sum, parser.bytes_dropped, parser.frames_ok / elapsed))
    if parser.frames_ok == 0:
        print("未收到任何有效帧。请确认：串口是否正确、波特率是否为 115200、"
              "RETURNCTRL/REPORTCTRL 是否已开启主动上报、接线 TX/RX 是否交叉。")
        return 2
    if motion_frames == 0:
        print("提示：未收到 0x03 帧，无法得到加速度/陀螺仪/温度；"
              "请检查 RETURNSET 的 bit2(0x04) 是否打开。")
    if mag_frames == 0:
        print("提示：未收到 0x04 帧，磁力计数据缺失。6 轴模块不输出磁力计；"
              "9/10 轴需确认 RETURNSET 的 bit3(0x08) 已打开。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

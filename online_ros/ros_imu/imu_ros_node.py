# -*- coding: utf-8 -*-
"""Hiwonder 10 轴 IMU -> ROS sensor_msgs/Imu，默认发布话题 /imu0（VINS-Fusion 直接可用）。

运行环境：Ubuntu（VirtualBox 客机），USB-TTL 直通后设备为 /dev/ttyUSB*。
依赖：ROS Noetic（rospy）、pyserial。本脚本与 imu_protocol.py 放在同一目录。

退出行为：
  * 自己接管 SIGINT/SIGTERM（rospy 用 disable_signals=True，避免它吞掉 Ctrl+C）；
  * 第一次 Ctrl+C：置停止标志，<100ms 内收尾退出；
  * 卡住时再按一次 Ctrl+C：直接 os._exit，无条件退出。

用法：
    python3 imu_ros_node.py                        # 自动找串口，发布 /imu0
    python3 imu_ros_node.py -p /dev/ttyUSB0
    python3 imu_ros_node.py --frame-id imu_link --bias-seconds 3
    python3 imu_ros_node.py --query                # 先回读设备量程再换算
    python3 imu_ros_node.py --list-ports           # 只列串口，不启动节点
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import threading
import time

try:
    import rospy
    from sensor_msgs.msg import Imu
except ImportError:
    sys.exit("未找到 rospy / sensor_msgs，请先 source /opt/ros/noetic/setup.bash")

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial，请先执行：pip3 install pyserial")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_protocol as proto  # noqa: E402

G_TO_MS2 = 9.80665
DPS_TO_RADPS = math.pi / 180.0
READ_TIMEOUT = 0.05          # 串口阻塞上限，决定 Ctrl+C 响应速度
EXIT_GRACE = 1.0             # 第一次 Ctrl+C 后留给正常清理的时间，超时无条件 os._exit
READ_CHUNK_MAX = 64          # 单次最多读取字节数，避免大缓冲把主循环拖住

USB_SERIAL_HINTS = ("ch340", "ch341", "cp210", "ft232", "usb-serial", "usb serial",
                    "silicon labs", "uart", "acm", "ttyusb")


def _hard_exit(code):
    """跳过 rospy 的 atexit 清理，保证进程立刻结束（测试可替换）。"""
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:
        pass
    os._exit(code)


def list_ports_verbose():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        print("未发现任何串口设备（确认 VirtualBox 已将 USB-TTL 直通给客机）")
    for p in ports:
        print("  %-16s %s" % (p.device, p.description))


def autodetect_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    for p in ports:
        text = " ".join(filter(None, (p.device, p.description, p.manufacturer))).lower()
        if any(h in text for h in USB_SERIAL_HINTS):
            return p.device
    return None


def query_config(ser, stop_event=None):
    """回读 ACCFSR / GYROFSR，失败则用默认值。"""
    accfsr, gyrofsr = proto.DEFAULT_ACCFSR, proto.DEFAULT_GYROFSR
    for reg, key in ((proto.REG_ACCFSR, "acc"), (proto.REG_GYROFSR, "gyro")):
        if stop_event is not None and stop_event.is_set():
            break
        try:
            ser.reset_input_buffer()
            ser.write(proto.build_read_cmd(reg))
            ser.flush()
            deadline = time.time() + 0.3
            buf = bytearray()
            while time.time() < deadline:
                if stop_event is not None and stop_event.is_set():
                    break
                buf.extend(ser.read(max(1, min(ser.in_waiting, READ_CHUNK_MAX))))
                i = buf.find(bytes([proto.HEADER, proto.CFG_HEADER]))
                if i >= 0:
                    reply = proto.parse_cfg_reply(bytes(buf[i:]))
                    if reply and reply[1]:
                        if key == "acc":
                            accfsr = reply[1][0]
                        else:
                            gyrofsr = reply[1][0]
                        break
        except Exception as exc:  # 部分固件不支持回读，退回默认值
            _log_warn("回读 %s 量程失败（%s），改用默认值" % (key, exc))
    return accfsr, gyrofsr


def _log_info(fmt, *args):
    try:
        rospy.loginfo(fmt, *args)
    except Exception:
        print(fmt % args if args else fmt)


def _log_warn(fmt, *args):
    try:
        rospy.logwarn(fmt, *args)
    except Exception:
        print("[WARN] " + (fmt % args if args else fmt))


class ImuNode:
    def __init__(self, args):
        self.args = args
        self._stop_event = threading.Event()
        self._count = 0
        self._t0 = time.time()
        self._last_report = self._t0
        self._last_stamp = None
        self.ser = None

        # 信号处理与退出看门狗必须在任何可能阻塞的动作（开串口、回读量程、等 master）之前就位
        self._install_signal_handlers()
        self._start_exit_watchdog()

        port = args.port or autodetect_port()
        if not port:
            list_ports_verbose()
            sys.exit("找不到串口设备，请用 -p /dev/ttyUSB0 指定")

        self.ser = serial.Serial(port, args.baud, timeout=READ_TIMEOUT,
                                 write_timeout=READ_TIMEOUT)
        self.parser = proto.ImuFrameParser()
        self.ser.reset_input_buffer()

        accfsr = args.accfsr if args.accfsr is not None else proto.DEFAULT_ACCFSR
        gyrofsr = args.gyrofsr if args.gyrofsr is not None else proto.DEFAULT_GYROFSR
        if args.query:
            accfsr, gyrofsr = query_config(self.ser, self._stop_event)
            self.ser.reset_input_buffer()
        self.accfsr, self.gyrofsr = accfsr, gyrofsr

        # 关键：disable_signals=True，否则 rospy 会接管 SIGINT 且吞掉 KeyboardInterrupt
        rospy.init_node("imu_publisher", anonymous=False, disable_signals=True)
        self.pub = rospy.Publisher(args.topic, Imu, queue_size=200)
        self.frame_id = args.frame_id

        self.bias = [0.0, 0.0, 0.0]
        if args.gyro_bias:
            vals = [float(x) for x in args.gyro_bias.split(",")]
            if len(vals) != 3:
                sys.exit("--gyro-bias 需要三个数：x,y,z（rad/s）")
            self.bias = vals
            _log_info("使用手动陀螺零偏 %.5f %.5f %.5f rad/s", *self.bias)

        _log_info("串口 %s @%d | 加速度 %sG (%.0f LSB/g) | 陀螺 %sdps (%.0f LSB/dps)",
                  port, args.baud,
                  proto.ACC_RANGE_G.get(accfsr, "?"), proto.ACC_LSB_PER_G.get(accfsr, 8192.0),
                  proto.GYRO_RANGE_DPS.get(gyrofsr, "?"),
                  proto.GYRO_LSB_PER_DPS.get(gyrofsr, 32.0))
        _log_info("发布 %s，frame_id=%s", args.topic, self.frame_id)

        self.orient_cov = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.acc_cov = [0.0] * 9
        self.gyr_cov = [0.0] * 9
        if args.acc_n > 0 or args.gyr_n > 0:
            fs = max(args.update_rate, 1.0)
            av = args.acc_n ** 2 * fs
            gv = args.gyr_n ** 2 * fs
            self.acc_cov = [av, 0.0, 0.0, 0.0, av, 0.0, 0.0, 0.0, av]
            self.gyr_cov = [gv, 0.0, 0.0, 0.0, gv, 0.0, 0.0, 0.0, gv]
            _log_info("协方差按 acc_n=%.3e gyr_n=%.3e 填充（VINS 不读取协方差）",
                      args.acc_n, args.gyr_n)

    # -------------------------------------------------------------- 退出控制
    # -------------------------------------------------------------- 退出控制
    def _install_signal_handlers(self):
        """接管 SIGINT/SIGTERM：第一次置停止标志，第二次立即硬退出。

        信号上下文里只做「置标志」这一件事。写 stderr 属于可阻塞 IO，一旦输出被
        重定向到满管道或由 roslaunch 接管，信号处理本身就会被卡住，所以提示信息
        统一交给看门狗线程打印。
        """
        def handler(signum, frame):
            if self._stop_event.is_set():
                _hard_exit(130)       # 第二次中断：不等清理，直接结束进程
            self._stop_event.set()    # 第一次中断：唤醒看门狗 + 让主循环退出

        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, handler)
            except (ValueError, OSError) as exc:
                _log_warn("注册 %s 处理器失败（%s），退回默认行为",
                          getattr(sig, "name", sig), exc)

    def _start_exit_watchdog(self):
        """后台守护线程：收到中断后给主线程 EXIT_GRACE 秒收尾，超时无条件退出。

        这是 Ctrl+C 能否真正退出的兜底。主线程可能卡在：
          * rospy.Publisher.publish() 的同步 socket 写（订阅端不消费、队列满即阻塞）；
          * USB 串口 read（USB 直通抖动、驱动层 timeout 失效）；
          * rospy.signal_shutdown() 内部线程 join。
        这些位置都不会回头检查 _stop_event，主循环里的判断永远等不到，只能强制结束。
        """
        threading.Thread(target=self._exit_watchdog,
                         name="imu-exit-watchdog", daemon=True).start()

    def _exit_watchdog(self):
        self._stop_event.wait()
        try:
            sys.stderr.write("\n[imu_ros_node] 收到中断，正在退出（再按一次 Ctrl+C 立即结束）...\n")
            sys.stderr.flush()
        except Exception:
            pass
        time.sleep(EXIT_GRACE)
        try:
            sys.stderr.write("[imu_ros_node] 清理超过 %.1fs，强制退出\n" % EXIT_GRACE)
            sys.stderr.flush()
        except Exception:
            pass
        _hard_exit(0)

    def _stopping(self):
        if self._stop_event.is_set():
            return True
        try:
            return rospy.is_shutdown()
        except Exception:
            # rospy 尚未就绪或已销毁，不能当作退出信号，否则会莫名退出
            return False

    def _read_chunk(self):
        """单次最多读 READ_CHUNK_MAX 字节，阻塞时间不超过 READ_TIMEOUT。

        原写法 ser.read(ser.in_waiting or 1)：in_waiting 可能是几千字节，read 会为了
        凑够长度一直等在内核里，主循环迟迟回不到 _stopping() 判断，Ctrl+C 响应被放大。
        """
        try:
            n = self.ser.in_waiting
        except Exception:
            n = 0
        return self.ser.read(max(1, min(n, READ_CHUNK_MAX)))

    # -------------------------------------------------------------- 发布逻辑
    def _stamp(self):
        now = rospy.Time.now()
        if self._last_stamp is not None and now <= self._last_stamp:
            now = self._last_stamp + rospy.Duration(0, 1000)
        self._last_stamp = now
        return now

    def estimate_bias(self, seconds):
        _log_info("估计陀螺零偏：请保持设备静止 %.1f 秒（Ctrl+C 可随时中止）", seconds)
        sums = [0.0, 0.0, 0.0]
        n = 0
        deadline = time.time() + seconds
        while time.time() < deadline and not self._stopping():
            chunk = self._read_chunk()
            if not chunk:
                continue
            for fid, data in self.parser.feed(chunk):
                if fid != proto.FRAME_MOTION:
                    continue
                d = proto.decode_motion(data, self.accfsr, self.gyrofsr)
                for i in range(3):
                    sums[i] += d["gyro"][i]
                n += 1
        if n == 0:
            _log_warn("零偏估计期间未收到数据，跳过")
            return
        self.bias = [s / n for s in sums]
        _log_info("陀螺零偏（%d 帧）：%.5f %.5f %.5f rad/s = %.3f %.3f %.3f deg/s",
                  n, self.bias[0], self.bias[1], self.bias[2],
                  self.bias[0] / DPS_TO_RADPS, self.bias[1] / DPS_TO_RADPS,
                  self.bias[2] / DPS_TO_RADPS)

    def publish_motion(self, d):
        msg = Imu()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.frame_id
        ax, ay, az = d["acc"]
        gx, gy, gz = d["gyro"]
        msg.linear_acceleration.x = ax * G_TO_MS2
        msg.linear_acceleration.y = ay * G_TO_MS2
        msg.linear_acceleration.z = az * G_TO_MS2
        msg.angular_velocity.x = (gx - self.bias[0]) * DPS_TO_RADPS
        msg.angular_velocity.y = (gy - self.bias[1]) * DPS_TO_RADPS
        msg.angular_velocity.z = (gz - self.bias[2]) * DPS_TO_RADPS
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.orientation_covariance = list(self.orient_cov)
        msg.linear_acceleration_covariance = list(self.acc_cov)
        msg.angular_velocity_covariance = list(self.gyr_cov)
        self.pub.publish(msg)
        self._count += 1

    def run(self):
        try:
            if self.args.bias_seconds > 0:
                self.estimate_bias(self.args.bias_seconds)
                self.parser = proto.ImuFrameParser()
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass

            if not self._stopping():
                _log_info("开始发布，Ctrl+C 结束")
            while not self._stopping():
                chunk = self._read_chunk()
                if not chunk:
                    continue
                for fid, data in self.parser.feed(chunk):
                    if self._stopping():     # 一个 chunk 可能含多帧，逐帧检查退出标志
                        break
                    if fid == proto.FRAME_MOTION:
                        self.publish_motion(
                            proto.decode_motion(data, self.accfsr, self.gyrofsr))
                self._report()
        except (KeyboardInterrupt, SystemExit):
            pass
        except Exception as exc:
            _log_warn("运行异常：%s", exc)
        finally:
            self._cleanup()

    def _report(self):
        now = time.time()
        if now - self._last_report < self.args.report_period:
            return
        hz = self._count / max(now - self._t0, 1e-6)
        _log_info("已发布 %d 帧 | 平均 %.1f Hz | 校验错 %d | 丢弃字节 %d",
                  self._count, hz, self.parser.frames_bad_sum, self.parser.bytes_dropped)
        if hz < self.args.update_rate * 0.5:
            _log_warn("发布频率 %.1f Hz 明显低于期望 %.0f Hz，"
                      "检查波特率、USB 直通带宽或设备回传率设置" % (hz, self.args.update_rate))
        self._last_report = now

    def _cleanup(self):
        """释放串口 -> 打印统计 -> 兜底强制退出，任何一步失败都不阻塞退出。"""
        try:
            if self.ser is not None and getattr(self.ser, "is_open", True):
                self.ser.close()
        except Exception:
            pass

        dt = max(time.time() - self._t0, 1e-6)
        try:
            sys.stderr.write("[imu_ros_node] 结束：共发布 %d 帧，平均 %.1f Hz，校验错 %d\n"
                             % (self._count, self._count / dt,
                                getattr(self.parser, "frames_bad_sum", 0)))
            sys.stderr.flush()
        except Exception:
            pass

        try:
            rospy.signal_shutdown("imu_ros_node finished")
        except Exception:
            pass

        _hard_exit(0)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="Hiwonder IMU -> ROS /imu0")
    ap.add_argument("-p", "--port", help="串口，如 /dev/ttyUSB0（默认自动扫描）")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--topic", default="/imu0", help="发布话题，默认 /imu0")
    ap.add_argument("--frame-id", default="imu_link")
    ap.add_argument("--accfsr", type=lambda x: int(x, 0), default=None,
                    help="0=2G 1=4G 2=8G 3=16G")
    ap.add_argument("--gyrofsr", type=lambda x: int(x, 0), default=None,
                    help="0=256 1=512 2=1024 3=2048 dps")
    ap.add_argument("--query", action="store_true", help="回读设备量程后再换算")
    ap.add_argument("--bias-seconds", type=float, default=0.0,
                    help="启动时静止估计陀螺零偏的秒数，默认 0（不估计）")
    ap.add_argument("--gyro-bias", help="手动零偏 rad/s，形如 x,y,z")
    ap.add_argument("--update-rate", type=float, default=100.0,
                    help="期望频率，用于协方差换算与频率告警")
    ap.add_argument("--acc-n", type=float, default=0.0, help="加速度噪声密度（用于协方差）")
    ap.add_argument("--gyr-n", type=float, default=0.0, help="陀螺噪声密度（用于协方差）")
    ap.add_argument("--report-period", type=float, default=10.0)
    ap.add_argument("--list-ports", action="store_true", help="列出串口后退出")
    return ap.parse_args(argv)


def main():
    args = parse_args()
    if args.list_ports:
        list_ports_verbose()
        return 0
    try:
        ImuNode(args).run()
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as exc:
        _log_warn("启动失败：%s", exc)
        return 1
    return 0

if __name__ == "__main__":
    sys.exit(main())

# -*- coding: utf-8 -*-
"""Windows 端：双目相机（单 UVC 左右拼接）+ Hiwonder 10 轴 IMU，直接录制 ROS1 bag。

不依赖 ROS，只靠 rosbags 写 bag 文件；相机用 OpenCV(DSHOW)，IMU 用 pyserial。

运行环境（Windows 11）：
    conda activate full
    python grab_bag.py

常用示例：
    # 默认：相机索引 1 / 2560x720 / 20fps，IMU 自动找串口 115200，录到 bags/MMDD_HHMMSS.bag
    python grab_bag.py

    # 指定串口与时长（秒），带实时预览窗口（默认已开启）
    python grab_bag.py --port COM3 --duration 60

    # 预览只显示左目 / 缩小窗口 / 关闭预览
    python grab_bag.py --preview-mode left --preview-width 960
    python grab_bag.py --no-preview

    # 相机当前只给低帧率时，改走 msmf 半分辨率（每目 640x720）
    python grab_bag.py --backend msmf --width 1280 --height 720 --fps 30

    # 只录相机（IMU 没插时）
    python grab_bag.py --no-imu

    # 只录 IMU
    python grab_bag.py --no-cam --out bags/imu_only.bag

    # 列出串口 / 可用相机后退出
    python grab_bag.py --list-ports
    python grab_bag.py --list-cams

输出话题（与 Linux 侧 online_cam.py / imu_ros_node.py 保持一致，VINS-Fusion 直接可用）：
    /image/raw/cam0   sensor_msgs/Image   bgr8（或 --to-mono8 时 mono8）
    /image/raw/cam1   sensor_msgs/Image   同上
    /imu0             sensor_msgs/Imu     6 轴（加速度 + 角速度），orientation_covariance[0] = -1

设计要点：
  * 相机线程 / IMU 线程 / 写盘线程三线分离，内存队列 + 背压，绝不让写盘阻塞采集；
  * 相机左右两帧共用同一时间戳，作为一个整体入队，保证 bag 里左右严格成对；
  * 写盘前做 reorder-window 时间戳重排，避免线程调度抖动导致 bag 内时间倒退；
  * Ctrl+C：第一次置停止标志，把队列和缓冲刷完再退出；卡住时再按一次立即结束。
"""

from __future__ import annotations

import argparse
import os
import signal
import struct
import sys
import threading
import time

import numpy as np

# ---------------------------------------------------------------------------
# Hiwonder 10 轴 IMU —— UART 协议（《10轴 IMU 通信协议》V1.0）
#
#   设备上报帧： 0x55 0x55 | ID | LEN | DATA[LEN] | SUM
#   SUM = (0x55 + 0x55 + ID + LEN + DATA 各字节) & 0xFF，不含 SUM 自身
#   多字节数据一律小端
#
#   0x03 加速度 + 角速度 + 陀螺温度   LEN = 14
#        ax ay az gx gy gz gyroTemp，均为 int16
#   加速度 g   = lsb / LSB_per_g        （ACCFSR: 0=2G 1=4G 2=8G 3=16G）
#   角速度 dps = lsb / LSB_per_dps      （GYROFSR: 0=256 1=512 2=1024 3=2048）
#   温度  ℃    = gyroTemp / 256
# ---------------------------------------------------------------------------
IMU_HEADER = 0x55
FRAME_MOTION = 0x03
FRAME_DATA_LEN = {0x01: 6, 0x02: 8, 0x03: 14, 0x04: 6, 0x05: 10}

ACC_LSB_PER_G = {0x00: 16384.0, 0x01: 8192.0, 0x02: 4096.0, 0x03: 2048.0}
GYRO_LSB_PER_DPS = {0x00: 128.0, 0x01: 64.0, 0x02: 32.0, 0x03: 16.0}
DEFAULT_ACCFSR = 0x01        # 出厂默认 4G
DEFAULT_GYROFSR = 0x02       # 出厂默认 1024dps

G_TO_MS2 = 9.80665
DPS_TO_RADPS = np.pi / 180.0

USB_SERIAL_HINTS = ("ch340", "ch341", "cp210", "ft232", "usb-serial", "usb serial",
                    "silicon labs", "uart", "acm")


class ImuFrameParser:
    """串口字节流 -> [(frame_id, data_bytes), ...]。"""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames_ok = 0
        self.frames_bad_sum = 0
        self.bytes_dropped = 0

    def feed(self, chunk: bytes):
        if chunk:
            self._buf.extend(chunk)
        out = []
        while True:
            idx = self._find_header()
            if idx < 0:
                if self._buf and self._buf[-1] == IMU_HEADER:
                    self.bytes_dropped += len(self._buf) - 1
                    del self._buf[:-1]
                else:
                    self.bytes_dropped += len(self._buf)
                    self._buf.clear()
                break
            if idx > 0:
                self.bytes_dropped += idx
                del self._buf[:idx]

            if len(self._buf) < 4:
                break
            fid = self._buf[2]
            dlen = self._buf[3]
            total = 4 + dlen + 1

            expect = FRAME_DATA_LEN.get(fid)
            if (expect is not None and dlen != expect) or dlen > 64:
                self.bytes_dropped += 1
                del self._buf[0]
                continue

            if len(self._buf) < total:
                break

            if (sum(self._buf[:total - 1]) & 0xFF) != self._buf[total - 1]:
                self.frames_bad_sum += 1
                self.bytes_dropped += 1
                del self._buf[0]
                continue

            out.append((fid, bytes(self._buf[4:total - 1])))
            self.frames_ok += 1
            del self._buf[:total]
        return out

    def _find_header(self) -> int:
        buf, n = self._buf, len(self._buf)
        for i in range(n - 1):
            if buf[i] == IMU_HEADER and buf[i + 1] == IMU_HEADER:
                return i
        return -1


def decode_motion(data: bytes, accfsr: int, gyrofsr: int):
    """0x03 帧 -> (acc[3] 单位 g, gyro[3] 单位 dps, temp 摄氏度)。"""
    ax, ay, az, gx, gy, gz, t = struct.unpack("<7h", data[:14])
    acc_k = ACC_LSB_PER_G.get(accfsr, 8192.0)
    gyro_k = GYRO_LSB_PER_DPS.get(gyrofsr, 32.0)
    return ((ax / acc_k, ay / acc_k, az / acc_k),
            (gx / gyro_k, gy / gyro_k, gz / gyro_k),
            t / 256.0)


def autodetect_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    for p in ports:
        text = " ".join(filter(None, (p.device, p.description,
                                      getattr(p, "manufacturer", None)))).lower()
        if any(h in text for h in USB_SERIAL_HINTS):
            return p.device
    return ports[0].device if len(ports) == 1 else None


# ---------------------------------------------------------------------------
# 消息构造 / 序列化（兼容 rosbags 0.9.x 与 0.10+）
# ---------------------------------------------------------------------------
def build_msg_kit():
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS1_NOETIC)
    kit = {
        "typestore": ts,
        "Image": ts.types["sensor_msgs/msg/Image"],
        "Imu": ts.types["sensor_msgs/msg/Imu"],
        "Header": ts.types["std_msgs/msg/Header"],
        "Time": ts.types["builtin_interfaces/msg/Time"],
        "Vector3": ts.types["geometry_msgs/msg/Vector3"],
        "Quaternion": ts.types["geometry_msgs/msg/Quaternion"],
    }
    if hasattr(ts, "serialize_ros1"):
        def _ser(msg, typename):
            return bytes(ts.serialize_ros1(msg, typename))
    else:                                  # 老版本 rosbags：模块级函数
        from rosbags.serde import serialize_ros1

        def _ser(msg, typename):
            try:
                return bytes(serialize_ros1(msg, typename, ts))
            except TypeError:
                return bytes(serialize_ros1(msg, typename))

    kit["serialize"] = _ser
    return kit


_SEQ_LOCK = threading.Lock()
_SEQ = {}


def _next_seq(frame_id: str) -> int:
    """ROS1 Header.seq 自增（按 frame_id 独立计数）。"""
    with _SEQ_LOCK:
        v = _SEQ.get(frame_id, 0)
        _SEQ[frame_id] = v + 1
        return v


def make_header(kit, stamp_sec: float, frame_id: str):
    sec = int(stamp_sec)
    nsec = int(round((stamp_sec - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    return kit["Header"](seq=_next_seq(frame_id),
                         stamp=kit["Time"](sec=sec, nanosec=nsec), frame_id=frame_id)


def make_image(kit, stamp_sec: float, frame_id: str, img: np.ndarray, encoding: str):
    h, w = img.shape[:2]
    step = w * (1 if encoding == "mono8" else 3)
    return kit["Image"](
        header=make_header(kit, stamp_sec, frame_id),
        height=h, width=w, encoding=encoding, is_bigendian=0, step=step,
        data=np.ascontiguousarray(img).reshape(-1),
    )


def make_imu(kit, stamp_sec: float, frame_id: str, acc_ms2, gyro_radps):
    zero9 = np.zeros(9, dtype=np.float64)
    orient_cov = np.array([-1.0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
    return kit["Imu"](
        header=make_header(kit, stamp_sec, frame_id),
        orientation=kit["Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=orient_cov,
        angular_velocity=kit["Vector3"](x=float(gyro_radps[0]),
                                        y=float(gyro_radps[1]),
                                        z=float(gyro_radps[2])),
        angular_velocity_covariance=zero9.copy(),
        linear_acceleration=kit["Vector3"](x=float(acc_ms2[0]),
                                           y=float(acc_ms2[1]),
                                           z=float(acc_ms2[2])),
        linear_acceleration_covariance=zero9.copy(),
    )


# ---------------------------------------------------------------------------
# 线程：相机采集
# ---------------------------------------------------------------------------
class CameraGrabber(threading.Thread):
    """单 UVC 双目（左右拼接）采集线程，左右半帧作为一个整体入队。"""

    def __init__(self, cfg, kit, queue, stop_event, stats):
        super().__init__(name="cam-grabber", daemon=True)
        self.cfg, self.kit, self.q, self.stop_event, self.stats = cfg, kit, queue, stop_event, stats
        self.latest_preview = None
        self.latest_full = None      # 供主线程预览的整幅拼接帧（引用赋值，无需加锁）
        self.latest_ts = 0.0
        self.cap = None

    CAND_FOURCC = ("NV12", "MJPG", "YUY2")   # 实际能跑满带宽的优先；顺序可调

    def _open_once(self, cv2, backend, cand):
        """按指定像素格式开一次相机，返回 (cap, w, h, fourcc)；失败返回 None。"""
        cfg = self.cfg
        cap = cv2.VideoCapture(cfg.cam, backend)
        if not cap.isOpened():
            cap.release()
            return None
        if cand:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cand))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
        cap.set(cv2.CAP_PROP_FPS, cfg.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cc = "".join(chr((int(cap.get(cv2.CAP_PROP_FOURCC)) >> (8 * i)) & 0xFF)
                     for i in range(4)).strip("\x00 ")
        return cap, w, h, cc

    def _probe_fps(self, cap, frames=6):
        """实测驱动能给出的帧率（先丢 2 帧预热）；明显偏慢时提前结束，避免启动卡太久。"""
        for _ in range(10):
            cap.read()
        t = time.time()
        n = 0
        budget = frames / max(self.cfg.fps, 1.0) * 2.5
        for _ in range(frames):
            ok, _f = cap.read()
            n += 1 if ok else 0
            if (time.time() - t) > budget:
                break
        return n / max(time.time() - t, 1e-6)

    def open(self):
        import cv2
        cfg = self.cfg
        backend = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF,
                   "any": cv2.CAP_ANY}.get((cfg.backend or "dshow").lower(), cv2.CAP_DSHOW)
        cands = []
        for c in (cfg.fourcc, "") + self.CAND_FOURCC:
            c = (c or "").strip().upper()
            if c not in cands:
                cands.append(c)

        best = None      # (fps, cand, cc)
        seen = set()     # 已经实测过的实际格式，避免重复探测同一格式
        for cand in cands:
            r = self._open_once(cv2, backend, cand)
            if r is None:
                continue
            cap, w, h, cc = r
            if cand and cc != cand:
                log("像素格式 %s 不被支持（驱动返回 %s），跳过" % (cand, cc))
                cap.release()
                continue
            if (w, h) != (cfg.width, cfg.height):
                log("格式 %s 下分辨率为 %dx%d（请求 %dx%d），跳过"
                    % (cand or "默认", w, h, cfg.width, cfg.height))
                cap.release()
                continue
            if cc in seen:
                cap.release()
                continue
            seen.add(cc)
            fps_meas = self._probe_fps(cap)
            cap.release()          # 探测完立即释放，避免多次打开同一设备互相抢占
            log("格式探测 %-4s -> %dx%d %s 实测 %.1f fps"
                % (cand or "默认", w, h, cc, fps_meas))
            if best is None or fps_meas > best[0]:
                best = (fps_meas, cand, cc)
            if fps_meas >= cfg.fps * 0.9:
                break

        if best is None:
            raise RuntimeError("相机索引 %d 在工作分辨率 %dx%d 下打不开（--list-cams 查看可用索引）"
                               % (cfg.cam, cfg.width, cfg.height))
        fps_meas, cand, cc = best
        r = self._open_once(cv2, backend, cand)
        if r is None:
            raise RuntimeError("相机索引 %d 二次打开失败" % cfg.cam)
        cap, w, h, cc2 = r
        self.cap, self.fourcc_used = cap, cc2
        self.frame_size = (w // 2, h)
        if fps_meas < cfg.fps * 0.9:
            log("警告：驱动只给了 %s，实测 %.1f fps（目标 %.1f）。可尝试：① 相机换到 USB 3.0 口/换线；"
                "② 用 --backend msmf --width 1280 --height 720 走半分辨率（每目 640x720）；"
                "③ 降低 --width/--height 或 --fps"
                % (cc2 or "未知格式", fps_meas, cfg.fps))
        log("相机已打开 idx=%d %dx%d fourcc=%s backend=%s 实测 %.1f fps"
            % (cfg.cam, w, h, cc2, cfg.backend, fps_meas))
        # 丢掉刚上电的几帧自动曝光/白平衡未稳定的画面
        for _ in range(50):
            cap.read()

    def run(self):
        import cv2
        min_interval = 1.0 / self.cfg.fps if self.cfg.fps > 0 else 0.0
        last_kept = 0.0
        try:
            while not self.stop_event.is_set():
                ok, frame = self.cap.read()
                ts = time.time()
                if not ok or frame is None:
                    self.stats["cam_read_fail"] += 1
                    time.sleep(0.005)
                    continue
                if min_interval and (ts - last_kept) < min_interval * 0.9:
                    continue           # 驱动按 60fps 出帧时，软件节流到目标帧率
                last_kept = ts
                h, w = frame.shape[:2]
                half = w // 2
                left = np.ascontiguousarray(frame[:, :half])
                right = np.ascontiguousarray(frame[:, half:half * 2])
                if self.cfg.to_mono8:
                    left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
                    right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
                enc = "mono8" if self.cfg.to_mono8 else "bgr8"
                msgs = (
                    ("/image/raw/cam0", make_image(self.kit, ts, self.cfg.frame_id_cam0, left, enc)),
                    ("/image/raw/cam1", make_image(self.kit, ts, self.cfg.frame_id_cam1, right, enc)),
                )
                if self.cfg.preview:
                    self.latest_full = frame
                    self.latest_preview = left
                    self.latest_ts = ts
                try:
                    self.q.put_nowait((ts, msgs))
                    self.stats["cam_pairs"] += 1
                except Exception:
                    self.stats["cam_dropped"] += 1     # 队列满：整对丢弃，绝不阻塞采集
        except Exception as exc:
            log("相机线程异常：%s" % exc)
        finally:
            try:
                self.cap.release()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# 线程：IMU 采集
# ---------------------------------------------------------------------------
class ImuGrabber(threading.Thread):
    def __init__(self, cfg, kit, queue, stop_event, stats):
        super().__init__(name="imu-grabber", daemon=True)
        self.cfg, self.kit, self.q, self.stop_event, self.stats = cfg, kit, queue, stop_event, stats
        self.ser = None
        self.port = None
        self.last_rx = 0.0
        self.last_acc = (0.0, 0.0, 0.0)
        self.last_gyro = (0.0, 0.0, 0.0)

    def open(self):
        import serial
        port = self.cfg.port or autodetect_port()
        if not port:
            raise RuntimeError("找不到串口（确认 IMU 的 USB-TTL 已插好、驱动装好，或用 --port COMx 指定）")
        self.ser = serial.Serial(port, self.cfg.baud, timeout=0.05, write_timeout=0.05)
        self.ser.reset_input_buffer()
        self.port = port
        log("IMU 串口已打开 %s @%d" % (port, self.cfg.baud))

    def run(self):
        parser = ImuFrameParser()
        bias = self.cfg.gyro_bias
        acc_k = self.cfg.accfsr if self.cfg.accfsr is not None else DEFAULT_ACCFSR
        gyro_k = self.cfg.gyrofsr if self.cfg.gyrofsr is not None else DEFAULT_GYROFSR
        try:
            while not self.stop_event.is_set():
                try:
                    n = self.ser.in_waiting
                except Exception:
                    n = 0
                chunk = self.ser.read(max(1, min(n, 64)))
                if not chunk:
                    continue
                for fid, data in parser.feed(chunk):
                    if fid != FRAME_MOTION:
                        continue
                    ts = time.time()
                    acc_g, gyro_dps, _temp = decode_motion(data, acc_k, gyro_k)
                    acc = (acc_g[0] * G_TO_MS2, acc_g[1] * G_TO_MS2, acc_g[2] * G_TO_MS2)
                    gyr = ((gyro_dps[0] - bias[0]) * DPS_TO_RADPS,
                           (gyro_dps[1] - bias[1]) * DPS_TO_RADPS,
                           (gyro_dps[2] - bias[2]) * DPS_TO_RADPS)
                    msg = make_imu(self.kit, ts, self.cfg.frame_id_imu, acc, gyr)
                    self.last_acc = acc
                    self.last_gyro = gyr
                    self.last_rx = ts
                    try:
                        self.q.put_nowait((ts, (("/imu0", msg),)))
                        self.stats["imu_frames"] += 1
                    except Exception:
                        self.stats["imu_dropped"] += 1
                self.stats["imu_bad_sum"] = parser.frames_bad_sum
        except Exception as exc:
            log("IMU 线程异常：%s" % exc)
        finally:
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# 线程：写 bag（时间戳重排 + 批量写盘）
# ---------------------------------------------------------------------------
class BagWriter(threading.Thread):
    def __init__(self, cfg, kit, topics, queue, stop_event, stats):
        super().__init__(name="bag-writer", daemon=True)
        self.cfg, self.kit, self.topics = cfg, kit, topics
        self.q, self.stop_event, self.stats = queue, stop_event, stats
        self.written = 0
        self.error = None

    def _flush(self, writer, conns, buf, max_ts, force):
        if not buf:
            return
        horizon = None if force else max_ts - self.cfg.reorder_window
        keep = []
        due = []
        for item in buf:
            if horizon is None or item[0] <= horizon:
                due.append(item)
            else:
                keep.append(item)
        if not due:
            return
        due.sort(key=lambda x: x[0])
        ser = self.kit["serialize"]
        for ts, payloads in due:
            stamp = int(round(ts * 1e9))
            for topic, msg in payloads:
                writer.write(conns[topic], stamp, ser(msg, self.topics[topic]))
                self.written += 1
        buf[:] = keep

    def run(self):
        from rosbags.rosbag1 import Writer
        out = self.cfg.out
        os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
        buf = []
        max_ts = 0.0
        try:
            with Writer(out) as writer:
                conns = {t: writer.add_connection(t, mt, typestore=self.kit["typestore"])
                         for t, mt in self.topics.items()}
                while True:
                    try:
                        item = self.q.get(timeout=0.1)
                        buf.append(item)
                        max_ts = max(max_ts, item[0])
                    except Exception:
                        pass
                    self._flush(writer, conns, buf, max_ts, force=False)
                    if self.stop_event.is_set() and self.q.empty():
                        break
                while not self.q.empty():
                    try:
                        item = self.q.get_nowait()
                        buf.append(item)
                        max_ts = max(max_ts, item[0])
                    except Exception:
                        break
                self._flush(writer, conns, buf, max_ts, force=True)
        except Exception as exc:
            self.error = exc
            log("写盘线程异常：%s" % exc)


# ---------------------------------------------------------------------------
# 实时预览窗口（必须在主线程调用 imshow）
# ---------------------------------------------------------------------------
class PreviewWindow:
    """实时显示采集画面：支持整幅双目/单目，叠加 HUD，q/ESC 结束、s 截图。"""

    WIN_NAME = "grab_bag preview  [q/ESC] quit   [s] snapshot"

    def __init__(self, cfg):
        import cv2
        self.cv2 = cv2
        self.cfg = cfg
        self.last_draw = 0.0
        cv2.namedWindow(self.WIN_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        self.drawn = 0

    # -- 帧选取 -------------------------------------------------------------
    def _pick(self, frame):
        img = frame
        if img is None:
            return None
        mode = getattr(self.cfg, "preview_mode", "stereo")
        if mode in ("left", "right"):
            half = img.shape[1] // 2
            img = img[:, :half] if mode == "left" else img[:, half:half * 2]
        return np.ascontiguousarray(img)

    def _fit(self, img):
        cv2 = self.cv2
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        limit = getattr(self.cfg, "preview_width", 0) or 0
        if limit and img.shape[1] > limit:
            s = limit / float(img.shape[1])
            img = cv2.resize(img, (max(1, int(round(img.shape[1] * s))),
                                   max(1, int(round(img.shape[0] * s)))),
                             interpolation=cv2.INTER_AREA)
        return img

    # -- HUD ----------------------------------------------------------------
    def _hud(self, img, stats, t0, imu, cam_size):
        cv2 = self.cv2
        h, w = img.shape[:2]
        dt = max(time.time() - t0, 1e-6)
        cam_fps = stats["cam_pairs"] / dt
        imu_hz = stats["imu_frames"] / dt
        now = time.time()
        lines = []
        lines.append(("REC %.1fs" % dt, (60, 60, 255)))
        lines.append(("cam %dx%d (x2)  %.1f fps  pairs %d"
                      % (cam_size[0], cam_size[1], cam_fps, stats["cam_pairs"]),
                      (90, 235, 90)))
        if imu is None:
            lines.append(("imu disabled", (170, 170, 170)))
        else:
            fresh = imu.last_rx > 0 and (now - imu.last_rx) < 1.0
            acc, gyr = imu.last_acc, imu.last_gyro
            lines.append(("imu %s  %.1f Hz  n=%d" % (imu.port or "-", imu_hz,
                                                    stats["imu_frames"]),
                          (90, 235, 90) if fresh else (60, 60, 240)))
            lines.append(("a %.2f %.2f %.2f m/s2" % acc, (215, 215, 215)))
            lines.append(("w %.3f %.3f %.3f rad/s" % gyr, (215, 215, 215)))
        lines.append(("drop cam %d  imu %d  bad_sum %d"
                      % (stats["cam_dropped"], stats["imu_dropped"],
                         stats.get("imu_bad_sum", 0)), (185, 185, 185)))

        pad, lh = 6, 18
        box_h = min(h, pad * 2 + lh * len(lines))
        box_w = min(w, 300)
        if box_h > 0 and box_w > 0:
            roi = img[0:box_h, 0:box_w]
            img[0:box_h, 0:box_w] = (roi.astype(np.uint16) * 35 // 100).astype(np.uint8)
        y = pad + 13
        for text, color in lines:
            if y > box_h:
                break
            cv2.putText(img, text, (pad + 2, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                        color, 1, cv2.LINE_AA)
            y += lh

    # -- 对外接口 -----------------------------------------------------------
    def show(self, frame, stats, t0, imu=None, cam_size=(0, 0)):
        """返回按键码（q/ESC=27 表示结束）；frame 为 None 时只维持窗口响应。"""
        cv2 = self.cv2
        img = self._pick(frame)
        if img is None:
            return cv2.waitKey(1) & 0xFF
        img = self._fit(img)
        self._hud(img, stats, t0, imu, cam_size)
        cv2.imshow(self.WIN_NAME, img)
        self.drawn += 1
        return cv2.waitKey(1) & 0xFF

    def snapshot(self, frame, stats, t0, imu=None, cam_size=(0, 0)):
        """把当前帧（含 HUD）存成 jpg，返回路径。"""
        cv2 = self.cv2
        img = self._pick(frame)
        if img is None:
            return None
        img = self._fit(img)
        self._hud(img, stats, t0, imu, cam_size)
        outdir = self.cfg.snapshot_dir or os.path.join(
            os.path.dirname(os.path.abspath(self.cfg.out)), "snapshots")
        os.makedirs(outdir, exist_ok=True)
        path = os.path.join(outdir, time.strftime("snap_%m%d_%H%M%S.jpg"))
        cv2.imwrite(path, img)
        return path

    def close(self):
        try:
            self.cv2.destroyAllWindows()
            self.cv2.waitKey(1)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------
_LOG_LOCK = threading.Lock()


def log(msg: str) -> None:
    with _LOG_LOCK:
        sys.stdout.write("[%s] %s\n" % (time.strftime("%H:%M:%S"), msg))
        sys.stdout.flush()


def list_cams(max_index=6):
    import cv2
    found = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if not cap.isOpened():
            continue
        ok, _ = cap.read()
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        found.append((i, ok, w, h))
        cap.release()
    if not found:
        print("未发现可用相机")
    for i, ok, w, h in found:
        print("  idx=%d  可读帧=%s  当前分辨率 %dx%d" % (i, ok, w, h))


def list_ports_verbose():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        print("未发现任何串口设备（确认 USB-TTL 已插好）")
    for p in ports:
        print("  %-10s %s" % (p.device, p.description))


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Windows 端 双目相机 + IMU 直接录 ROS1 bag（无需 ROS）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # 相机
    ap.add_argument("--cam", type=int, default=1, help="相机索引（默认 1；--list-cams 查看）")
    ap.add_argument("--width", type=int, default=2560, help="拼接图宽度")
    ap.add_argument("--height", type=int, default=720, help="拼接图高度")
    ap.add_argument("--fps", type=float, default=20.0, help="目标录制帧率（驱动不支持时按软件节流）")
    ap.add_argument("--fourcc", default="MJPG", help="像素格式，MJPG 才能满带宽；留空则不设置")
    ap.add_argument("--backend", default="dshow", choices=["dshow", "msmf", "any"],
                    help="OpenCV 采集后端；MJPG 不生效时可试 msmf")
    ap.add_argument("--no-cam", action="store_true", help="不录相机")
    ap.add_argument("--to-mono8", action="store_true", help="转灰度 mono8（省一半体积，VINS 常用）")
    ap.add_argument("--frame-id-cam0", default="cam0", help="cam0 的 frame_id")
    ap.add_argument("--frame-id-cam1", default="cam1", help="cam1 的 frame_id")
    # IMU
    ap.add_argument("-p", "--port", help="串口，如 COM3（默认自动扫描 CH340/CP210x）")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--no-imu", action="store_true", help="不录 IMU")
    ap.add_argument("--accfsr", type=lambda x: int(x, 0), default=None, help="0=2G 1=4G 2=8G 3=16G")
    ap.add_argument("--gyrofsr", type=lambda x: int(x, 0), default=None, help="0=256 1=512 2=1024 3=2048dps")
    ap.add_argument("--gyro-bias", default="0,0,0",
                    help="手动陀螺零偏（dps），形如 x,y,z，会从角速度里减掉")
    ap.add_argument("--frame-id-imu", default="imu_link", help="IMU 的 frame_id")
    # 输出与运行
    ap.add_argument("--out", help="输出 bag 路径（默认 bags/MMDD_HHMMSS.bag）")
    ap.add_argument("--duration", type=float, default=0.0, help="录制时长（秒），0 表示手动 Ctrl+C")
    ap.add_argument("--reorder-window", type=float, default=0.1, help="时间戳重排窗口（秒）")
    ap.add_argument("--queue-cam", type=int, default=60, help="相机队列深度（对）")
    ap.add_argument("--queue-imu", type=int, default=4000, help="IMU 队列深度（帧）")
    ap.add_argument("--preview", dest="preview", action="store_true", default=True,
                    help="实时显示采集画面（默认开启）")
    ap.add_argument("--no-preview", dest="preview", action="store_false",
                    help="关闭实时画面")
    ap.add_argument("--preview-mode", default="stereo", choices=["stereo", "left", "right"],
                    help="预览内容：stereo=整幅双目拼接，left/right=只看单目")
    ap.add_argument("--preview-width", type=int, default=1280,
                    help="预览画面宽度上限（像素），0 表示不缩放")
    ap.add_argument("--snapshot-dir", default=None,
                    help="按 s 截图保存目录（默认 bag 同级的 snapshots/）")
    ap.add_argument("--list-ports", action="store_true", help="列出串口后退出")
    ap.add_argument("--list-cams", action="store_true", help="列出相机后退出")
    return ap.parse_args(argv)


def install_signals(stop_event):
    def handler(signum, frame):
        if stop_event.is_set():
            log("再次中断，立即退出")
            os._exit(130)
        stop_event.set()
        log("收到中断，正在收尾（再按一次 Ctrl+C 立即结束）...")
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):
            pass


def main(argv=None):
    args = parse_args(argv)

    if args.list_ports:
        list_ports_verbose()
        return 0
    if args.list_cams:
        list_cams()
        return 0
    if args.no_cam and args.no_imu:
        log("--no-cam 与 --no-imu 不能同时指定")
        return 2

    bias = [float(x) for x in args.gyro_bias.split(",")]
    if len(bias) != 3:
        log("--gyro-bias 需要三个数，形如 0,0,0")
        return 2
    args.gyro_bias = bias

    if not args.out:
        stamp = time.strftime("%m%d_%H%M%S")
        args.out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bags", stamp + ".bag")

    kit = build_msg_kit()
    topics = {}
    if not args.no_cam:
        topics["/image/raw/cam0"] = "sensor_msgs/msg/Image"
        topics["/image/raw/cam1"] = "sensor_msgs/msg/Image"
    if not args.no_imu:
        topics["/imu0"] = "sensor_msgs/msg/Imu"

    stop_event = threading.Event()
    install_signals(stop_event)

    stats = {"cam_pairs": 0, "imu_frames": 0, "cam_dropped": 0, "imu_dropped": 0,
             "cam_read_fail": 0, "imu_bad_sum": 0}

    cam_q = None
    cam_thread = imu_thread = writer_thread = None

    # 统一走一个写盘队列，保证跨话题全局有序
    from queue import Queue
    writer_q = Queue(maxsize=args.queue_cam * 2 + args.queue_imu)

    try:
        if not args.no_cam:
            cam_q = writer_q
            cam_thread = CameraGrabber(args, kit, cam_q, stop_event, stats)
            cam_thread.open()
        if not args.no_imu:
            imu_thread = ImuGrabber(args, kit, writer_q, stop_event, stats)
            imu_thread.open()
    except Exception as exc:
        log("启动失败：%s" % exc)
        for t in (cam_thread, imu_thread):
            try:
                if t is not None:
                    t.stop_event.set()
            except Exception:
                pass
        return 1

    writer_thread = BagWriter(args, kit, topics, writer_q, stop_event, stats)
    writer_thread.start()
    if cam_thread is not None:
        cam_thread.start()
    if imu_thread is not None:
        imu_thread.start()

    log("开始录制 -> %s" % os.path.abspath(args.out))
    log("话题：%s" % ", ".join(topics))
    if args.duration > 0:
        log("时长 %.1f s，或随时 Ctrl+C 结束" % args.duration)
    else:
        log("Ctrl+C 结束录制")

    t0 = time.time()
    last_report = t0

    preview = None
    if args.preview and cam_thread is not None:
        try:
            preview = PreviewWindow(args)
            log("预览窗口已打开：q/ESC 结束录制，s 截图（--no-preview 可关闭）")
        except Exception as exc:
            log("预览窗口打开失败（继续录制）：%s" % exc)

    snap_saved = 0
    try:
        while not stop_event.is_set():
            if preview is not None:
                key = preview.show(cam_thread.latest_full, stats, t0, imu_thread,
                                   getattr(cam_thread, "frame_size", (0, 0)))
                if key in (27, ord("q"), ord("Q")):
                    log("收到退出按键，结束录制")
                    stop_event.set()
                    break
                if key in (ord("s"), ord("S")):
                    path = preview.snapshot(cam_thread.latest_full, stats, t0, imu_thread,
                                            getattr(cam_thread, "frame_size", (0, 0)))
                    if path:
                        snap_saved += 1
                        log("已保存截图 %s" % path)
            now = time.time()
            if args.duration > 0 and (now - t0) >= args.duration:
                stop_event.set()
                break
            if now - last_report >= 2.0:
                dt = now - t0
                log("已录 %.1fs | 相机 %d 对 (%.1f fps) | IMU %d 帧 (%.1f Hz) | 丢帧 cam=%d imu=%d"
                    % (dt, stats["cam_pairs"], stats["cam_pairs"] / dt,
                       stats["imu_frames"], stats["imu_frames"] / dt,
                       stats["cam_dropped"], stats["imu_dropped"]))
                last_report = now
            time.sleep(0.02)
    except KeyboardInterrupt:
        stop_event.set()

    # 收尾：先停采集，再让写盘线程把队列和缓冲刷完
    stop_event.set()
    for t in (cam_thread, imu_thread):
        if t is not None:
            t.join(timeout=2.0)
    writer_thread.join(timeout=30.0)

    if preview is not None:
        preview.close()

    dt = max(time.time() - t0, 1e-6)
    log("录制结束：%.1fs，写入消息 %d 条 | 相机 %d 对 (%.1f fps) | IMU %d 帧 (%.1f Hz)"
        % (dt, writer_thread.written, stats["cam_pairs"], stats["cam_pairs"] / dt,
           stats["imu_frames"], stats["imu_frames"] / dt))
    if stats["cam_dropped"] or stats["imu_dropped"]:
        log("提示：队列溢出丢帧 cam=%d imu=%d，可加大 --queue-cam / --queue-imu 或降低 --fps"
            % (stats["cam_dropped"], stats["imu_dropped"]))
    if writer_thread.error is not None:
        log("写盘失败：%s" % writer_thread.error)
        return 1
    if os.path.exists(args.out):
        log("bag 已保存：%s（%.1f MB）" % (os.path.abspath(args.out),
                                          os.path.getsize(args.out) / 1024 / 1024))
    else:
        log("未生成 bag 文件")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

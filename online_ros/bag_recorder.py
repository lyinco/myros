#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双目图像 + IMU 的稳健 rosbag 录制器（替代 rosbag record，专治丢帧/卡盘）。

订阅（可改）：
    /image/raw/cam0  /image/raw/cam1   （online_cam.py 发布）
    /imu0                               （ros_imu/imu_ros_node.py 发布）

设计要点：
  1. 回调只做入队，绝不阻塞；写盘由独立线程顺序完成，chunk 大、不压缩。
  2. 图像队列满时丢弃「最新到达」的图像并计数，绝不阻塞发布端；IMU 队列更大。
  3. 双目成对：cam0/cam1 来自同一物理帧，若两帧 stamp 差在阈值内，强行统一为 cam0 的 stamp。
  4. bag 内时间 t 直接用 header.stamp，标定器按采集时刻对齐。
  5. 退出：自管 SIGINT/SIGTERM（rospy disable_signals），先收尾关包再退出，再按一次则强制退出。
  6. 默认写到本地 ext4 盘（~/bags），检测到 /mnt/shares、/media/sf_ 等共享目录会告警。
  7. 结束打印统计：各话题帧数/频率、最大间隔、丢帧数、成对修正数、bag 大小。

用法：
    python3 bag_recorder.py                              # 默认 ~/bags/rec_<时间>.bag
    python3 bag_recorder.py -o ~/bags/calib.bag
    python3 bag_recorder.py --duration 120               # 录 120 秒自动停
    python3 bag_recorder.py --to-mono8                   # 图像转 mono8 后写入（体积 1/3，标定足够）
    python3 bag_recorder.py --imag-only ...              # 不录 IMU
    python3 bag_recorder.py --compression lz4            # 需要省空间时才用，kalibr 读会慢

录完自检：
    rosbag info <bag>
    rosbag play --pause <bag>            # 配 rqt_image_view 看左右是否成对
"""

from __future__ import annotations

import argparse
import heapq
import os
import queue
import signal
import sys
import threading
import time

import rospy
import rosbag
from sensor_msgs.msg import Image, Imu

try:
    import cv2
except ImportError:      # 仅在 --to-mono8 时需要
    cv2 = None

try:
    from cv_bridge import CvBridge
except ImportError:      # 仅在 --to-mono8 时需要
    CvBridge = None


def _hard_exit(code):
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:
        pass
    os._exit(code)


class BagRecorder:
    def __init__(self, args):
        self.args = args
        self.stop_event = threading.Event()
        self.bag = None
        self.dropped = 0
        self.pair_fixed = 0
        self.pair_mismatch = 0
        self.counts = {}
        self.sizes = {}
        self.last_stamp = {}
        self.max_gap = {}
        self.min_stamp = {}
        self.first_stamp = None
        self.last_stamp_any = None
        self.written = 0
        self.late = 0
        self._last_written_t = float("-inf")
        self._max_t = float("-inf")
        self.pair_dropped = 0
        self._pending_cam0 = None
        self._bridge = None
        if args.to_mono8:
            if cv2 is None or CvBridge is None:
                sys.exit("--to-mono8 需要 python3-opencv 与 cv_bridge")
            self._bridge = CvBridge()

        out = os.path.expanduser(args.out or self._default_path())
        out = os.path.abspath(out)
        d = os.path.dirname(out) or "."
        if not os.path.isdir(d):
            os.makedirs(d)

        # 先起节点再动 rosbag/日志，保证 loginfo 有 node 上下文
        rospy.init_node("bag_recorder", anonymous=True, disable_signals=True)
        self._install_signal_handlers()
        self._warn_if_shared(d)

        comp = {"none": rosbag.Compression.NONE,
                "lz4": rosbag.Compression.LZ4,
                "bz2": rosbag.Compression.BZ2}[args.compression]
        self.out = out
        self.bag = rosbag.Bag(out, "w", compression=comp,
                              chunk_threshold=int(args.chunk_mb * 1024 * 1024))
        rospy.loginfo("写入 %s（compression=%s, chunk=%.0fMB）", out, args.compression,
                      args.chunk_mb)

        topics = list(args.image_topics)
        if not args.imag_only:
            topics.append(args.imu_topic)

        self.q = queue.Queue(maxsize=args.max_queue)
        self._writer = threading.Thread(target=self._write_loop, name="bag_writer")
        self._writer.daemon = True
        self._writer.start()

        self.subs = []
        for t in args.image_topics:
            self.subs.append(rospy.Subscriber(t, Image, self._image_cb, queue_size=10,
                                              callback_args=t))
        if not args.imag_only:
            self.subs.append(rospy.Subscriber(args.imu_topic, Imu, self._imu_cb,
                                              queue_size=200))
        rospy.loginfo("订阅：%s", ", ".join(topics))
        self.t0 = time.time()
        self._last_health = self.t0

    # ---------------------------------------------------------------- 工具
    def _default_path(self):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return "~/bags/rec_%s.bag" % stamp

    def _warn_if_shared(self, d):
        for bad in ("/mnt/shares", "/media/sf_", "/mnt/hgfs"):
            if d.startswith(bad):
                rospy.logwarn("输出目录在共享文件夹 %s（vboxsf/NTFS），随机写延迟高，"
                              "是 bag 卡顿与丢帧的常见原因；建议先写 ~/bags 再拷出去", d)

    def _install_signal_handlers(self):
        def handler(signum, frame):
            if self.stop_event.is_set():
                _hard_exit(130)
            self.stop_event.set()
            try:
                sys.stderr.write("\n[bag_recorder] 收到中断，正在收尾关包（再按一次强制退出）...\n")
                sys.stderr.flush()
            except Exception:
                pass

        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, handler)
            except (ValueError, OSError):
                pass

    def _stopping(self):
        if self.stop_event.is_set():
            return True
        if self.args.duration > 0 and time.time() - self.t0 >= self.args.duration:
            return True
        try:
            return rospy.is_shutdown()
        except Exception:
            return True

    # ------------------------------------------------------------ 回调（只入队）
    def _image_cb(self, msg, topic):
        if self._stopping():
            return
        stamp = msg.header.stamp
        frame_id = msg.header.frame_id
        if self.args.to_mono8 and msg.encoding != "mono8":
            try:
                img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
                msg = self._bridge.cv2_to_imgmsg(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), "mono8")
            except Exception as exc:
                rospy.logwarn("转 mono8 失败，跳过该帧：%s", exc)
                return
            # cv2_to_imgmsg 会重置 header，务必回填采集时刻与坐标系
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id

        # 双目成对：只写成功配对的整对，绝不写入错位帧
        if self.args.pair_sync > 0:
            self._pair_cb(topic, msg, stamp)
            return
        self._enqueue([(topic, msg, stamp)])

    def _pair_cb(self, topic, msg, stamp):
        """cam0 先缓存，等 cam1 到达后整对同戳写入；配不上就整对作废。

        这样即使 ROS 传输层或本节点队列丢了某一侧，bag 里也不会出现
        cam0[i] 配 cam1[i-1] 的错位，代价是宁可少一对也不污染标定数据。
        """
        cam0, cam1 = self.args.image_topics[0], self.args.image_topics[1]
        if topic == cam0:
            if self._pending_cam0 is not None:
                self.pair_dropped += 1
                rospy.logwarn_throttle(5.0, "cam0 未等到配对 cam1（传输丢帧），"
                                       "整对丢弃，累计 %d 对", self.pair_dropped)
            self._pending_cam0 = (stamp, msg)
            return
        if topic != cam1:
            self._enqueue([(topic, msg, stamp)])
            return
        if self._pending_cam0 is None:
            self.pair_mismatch += 1
            rospy.logwarn_throttle(5.0, "cam1 先于 cam0 到达，丢弃该帧（累计 %d 次）",
                                   self.pair_mismatch)
            return
        ref_stamp, ref_msg = self._pending_cam0
        self._pending_cam0 = None
        dt = abs(stamp.to_sec() - ref_stamp.to_sec())
        if dt > self.args.pair_sync:
            self.pair_mismatch += 1
            self.pair_dropped += 1
            rospy.logwarn_throttle(5.0, "cam0/cam1 时间差 %.1fms 超阈 %.0fms，整对丢弃，"
                                   "累计 %d 对", dt * 1e3, self.args.pair_sync * 1e3,
                                   self.pair_dropped)
            return
        if dt > 0:
            self.pair_fixed += 1
        msg.header.stamp = ref_stamp
        # 整对一次性入队，同戳同批，永不错位
        self._enqueue([(cam0, ref_msg, ref_stamp), (cam1, msg, ref_stamp)])

    def _imu_cb(self, msg):
        if self._stopping():
            return
        self._enqueue([(self.args.imu_topic, msg, msg.header.stamp)])

    def _enqueue(self, entries):
        """entries 为一个不可分割的批次 [(topic, msg, stamp), ...]。

        双目必须整批进队列：否则 cam0 进、cam1 被丢，bag 里就会出现单边帧。
        """
        topics = [e[0] for e in entries]
        is_img = any(t in self.args.image_topics for t in topics)
        timeout = self.args.image_block if is_img else self.args.imu_block
        t = entries[0][2].to_sec()
        try:
            self.q.put((t, entries), timeout=timeout)
        except queue.Full:
            self.dropped += len(entries)
            rospy.logwarn_throttle(3.0, "写盘跟不上（队列 %d 满），已丢弃 %d 条（最近：%s）",
                                   self.args.max_queue, self.dropped, topics[0])

    # ------------------------------------------------------------ 写盘线程
    def _write_loop(self):
        heap = []
        seq = 0
        window = max(self.args.reorder_window, 0.0)

        def emit_ready(limit):
            while heap and (limit is None or heap[0][0] <= limit):
                _, _, entries = heapq.heappop(heap)
                for topic, msg, stamp in entries:
                    self._write_one(topic, msg, stamp)

        while True:
            got = False
            try:
                t, entries = self.q.get(timeout=0.1)
                got = True
            except queue.Empty:
                pass
            if got:
                heapq.heappush(heap, (t, seq, entries))
                seq += 1
                if t > self._max_t:
                    self._max_t = t
            if heap:
                emit_ready(self._max_t - window)
            if self.stop_event.is_set() and self.q.empty():
                emit_ready(None)      # 收尾：剩余全部按时间序写出
                break

    def _write_one(self, topic, msg, stamp):
        t = stamp.to_sec()
        if t < self._last_written_t:
            self.late += 1
        self._last_written_t = t
        try:
            self.bag.write(topic, msg, stamp)
        except Exception as exc:
            rospy.logerr("写入失败：%s", exc)
            return
        self._stats(topic, msg, stamp)
        self.written += 1

    def _stats(self, topic, msg, stamp):
        t = stamp.to_sec()
        self.counts[topic] = self.counts.get(topic, 0) + 1
        try:
            self.sizes[topic] = self.sizes.get(topic, 0) + msg._buffered_len
        except AttributeError:
            self.sizes[topic] = self.sizes.get(topic, 0) + len(getattr(msg, "data", b""))
        self.min_stamp.setdefault(topic, t)
        prev = self.last_stamp.get(topic)
        if prev is not None:
            gap = t - prev
            if gap > self.max_gap.get(topic, (0.0, None))[0]:
                self.max_gap[topic] = (gap, t)
        self.last_stamp[topic] = t
        if self.first_stamp is None or t < self.first_stamp:
            self.first_stamp = t
        if self.last_stamp_any is None or t > self.last_stamp_any:
            self.last_stamp_any = t

    # ------------------------------------------------------------ 主流程
    def run(self):
        rospy.loginfo("录制中，Ctrl+C 结束")
        try:
            while not self._stopping():
                time.sleep(0.05)
                self._health()
        except (KeyboardInterrupt, SystemExit):
            pass
        finally:
            self._cleanup()

    def _health(self):
        now = time.time()
        if now - self._last_health < self.args.health_period:
            return
        self._last_health = now
        dur = max(now - self.t0, 1e-6)
        parts = ["%s=%d(%.1fHz)" % (self._short(t), self.counts.get(t, 0),
                                    self.counts.get(t, 0) / dur)
                 for t in (list(self.args.image_topics) +
                           ([] if self.args.imag_only else [self.args.imu_topic]))]
        rospy.loginfo("已写 %d 条 | 队列 %d | 丢弃 %d | 配对修正 %d | %s",
                      self.written, self.q.qsize(), self.dropped, self.pair_fixed,
                      " ".join(parts))
        for t in self.args.image_topics:
            if self.counts.get(t, 0) == 0:
                rospy.logwarn("话题 %s 尚未收到任何消息，检查发布端是否在跑、话题名是否一致", t)

    @staticmethod
    def _short(topic):
        return topic.rsplit("/", 1)[-1]

    def _cleanup(self):
        self.stop_event.set()
        try:
            # 必须等写盘线程把队列排空再关包，否则可能写出损坏的 bag
            pending = self.q.qsize()
            self._writer.join(timeout=max(15.0, min(120.0, pending * 0.1)))
            if self._writer.is_alive():
                sys.stderr.write("警告：仍有 %d 条未写完（写盘过慢），bag 可能不完整\n"
                                 % self.q.qsize())
        except Exception:
            pass
        try:
            if self.bag is not None:
                self.bag.close()
        except Exception as exc:
            sys.stderr.write("关包异常：%s\n" % exc)

        dur = (self.last_stamp_any - self.first_stamp) if self.first_stamp else 0.0
        lines = ["", "=" * 62, "bag: %s" % self.out]
        try:
            lines.append("大小: %.1f MB   时间跨度: %.2f s" %
                         (os.path.getsize(self.out) / 1048576.0, dur))
        except OSError:
            lines.append("时间跨度: %.2f s" % dur)
        for t in self.args.image_topics + ([] if self.args.imag_only else [self.args.imu_topic]):
            n = self.counts.get(t, 0)
            gap, gap_at = self.max_gap.get(t, (0.0, None))
            lines.append("  %-24s %6d 帧  %6.1f Hz  %6.1f MB  最大间隔 %.3f s%s"
                         % (t, n, n / dur if dur > 0 else 0.0,
                            self.sizes.get(t, 0) / 1048576.0, gap,
                            "" if gap_at is None else "（@%.3f）" % gap_at))
        lines.append("  丢弃 %d 条 | 配对修正 %d 帧 | 配对超阈 %d 次 | 整对作废 %d 对"
                     " | 时间戳乱序 %d 条"
                     % (self.dropped, self.pair_fixed, self.pair_mismatch,
                        self.pair_dropped, self.late))
        if self.args.imag_only is False:
            c0 = self.counts.get(self.args.image_topics[0], 0)
            ci = self.counts.get(self.args.imu_topic, 0)
            if dur > 0:
                lines.append("  图像:IMU 帧数比 %.2f:1（IMU 应明显多于图像）" %
                             (ci / c0 if c0 else float("inf")))
        lines.append("=" * 62)
        try:
            sys.stderr.write("\n".join(lines) + "\n")
            sys.stderr.flush()
        except Exception:
            pass
        _hard_exit(0)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="双目 + IMU 稳健 bag 录制器")
    ap.add_argument("-o", "--out", help="输出 bag 路径，默认 ~/bags/rec_<时间>.bag")
    ap.add_argument("--image-topics", nargs=2, default=["/image/raw/cam0", "/image/raw/cam1"],
                    metavar=("CAM0", "CAM1"))
    ap.add_argument("--imu-topic", default="/imu0")
    ap.add_argument("--imag-only", action="store_true", help="只录图像，不订阅 IMU")
    ap.add_argument("--to-mono8", action="store_true",
                    help="bgr8 转 mono8 再写（体积约 1/3，标定/VINS 都够用）")
    ap.add_argument("--pair-sync", type=float, default=0.005,
                    help="cam0/cam1 视为同一帧的 stamp 差阈值（秒），默认 5ms，0 关闭")
    ap.add_argument("--duration", type=float, default=0.0, help="录制时长（秒），0=一直录")
    ap.add_argument("--compression", choices=["none", "lz4", "bz2"], default="none")
    ap.add_argument("--chunk-mb", type=float, default=8.0)
    ap.add_argument("--max-queue", type=int, default=400, help="写盘队列长度上限")
    ap.add_argument("--image-block", type=float, default=0.05,
                    help="队列满时图像回调最多等待秒数，超时才丢")
    ap.add_argument("--imu-block", type=float, default=2.0,
                    help="队列满时 IMU 回调最多等待秒数（IMU 不能断）")
    ap.add_argument("--reorder-window", type=float, default=0.1,
                    help="乱序重排窗口（秒）：按时间戳有序写入 bag，默认 100ms")
    ap.add_argument("--health-period", type=float, default=5.0)
    return ap.parse_args(argv)


def main():
    args = parse_args()
    rec = BagRecorder(args)
    rec.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())


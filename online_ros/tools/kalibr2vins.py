# -*- coding: utf-8 -*-
"""把 Kalibr 的 camchain-imucam-*.yaml 转成 VINS-Fusion 可直接使用的配置片段。

输入（Kalibr --output camchain-imucam 生成）：
    camchain-imucam-xxx.yaml  含 cam0/cam1 内参、畸变、分辨率、T_cam_imu、T_cn_cnm1 与 timeshift_cam_imu

输出（--outdir 指定，默认当前目录）：
    cam0.yaml / cam1.yaml      VINS 相机内参文件
    vins_imu_cam.yaml          含 body_T_cam0 / body_T_cam1 / estimate_extrinsic / td 的片段

方向约定（最容易错的地方）：
    Kalibr : T_cam_imu  —— 把 IMU 系下的点变到相机系   p_cam  = T_cam_imu * p_imu
    VINS   : body_T_cam —— 把相机系下的点变到 body 系  p_body = body_T_cam * p_cam
    因此     body_T_cam0 = inv(T_cam0_imu)
             body_T_cam1 = T_cn_cnm1 * body_T_cam0

用法：
    python kalibr2vins.py camchain-imucam-2026-10-27.yaml
    python kalibr2vins.py camchain-imucam-*.yaml --outdir ./vins_cfg
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np
import yaml

# Kalibr 相机模型 -> VINS 相机模型
MODEL_MAP = {
    "pinhole-radtan": ("PINHOLE", "radtan"),
    "pinhole-equi": ("KANNALA_BRANDT", "equidistant"),
    "pinhole-fov": ("PINHOLE", "fov"),
    "ds-none": ("PINHOLE", "radtan"),
}


class _Loader(yaml.SafeLoader):
    pass


_Loader.add_multi_constructor("tag:yaml.org,2002:python/object", lambda l, s, n: None)
_Loader.add_multi_constructor("!", lambda l, s, n: None)


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.load(fh, Loader=_Loader)


def mat(m):
    return np.array(m, dtype=float)


def rot_to_quat(R):
    """旋转矩阵 -> 四元数，返回 Eigen 顺序 (x, y, z, w)。"""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def inv_T(T):
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def check_orthonormal(T, name):
    R = T[:3, :3]
    err = float(np.max(np.abs(R @ R.T - np.eye(3))))
    if err > 1e-4:
        print("  [!] %s 旋转部分非正交，误差 %.2e，请检查源文件" % (name, err))


def fmt_row(v):
    return "[" + ", ".join("%.9g" % x for x in v) + "]"


def write_cam_yaml(path, cam, vins_model):
    intr = mat(cam["intrinsics"])
    dist = mat(cam["distortion_coeffs"])
    res = cam["resolution"]

    lines = ["# VINS-Fusion camera config, generated from Kalibr by kalibr2vins.py"]
    lines.append("model_type: %s" % vins_model)
    lines.append("camera_name: %s" % cam.get("rostopic", "camera"))
    lines.append("image_width: %d" % int(res[0]))
    lines.append("image_height: %d" % int(res[1]))
    lines.append("distortion_parameters:")
    if vins_model == "PINHOLE":
        lines.append("   k1: %.12g" % dist[0])
        lines.append("   k2: %.12g" % dist[1])
        lines.append("   p1: %.12g" % dist[2])
        lines.append("   p2: %.12g" % dist[3])
    else:
        for i, k in enumerate(["k1", "k2", "k3", "k4"]):
            lines.append("   %s: %.12g" % (k, dist[i] if i < len(dist) else 0.0))
    lines.append("projection_parameters:")
    lines.append("   fx: %.12g" % intr[0])
    lines.append("   fy: %.12g" % intr[1])
    lines.append("   cx: %.12g" % intr[2])
    lines.append("   cy: %.12g" % intr[3])
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


def write_body_T(fh, name, T):
    R = T[:3, :3]
    t = T[:3, 3]
    q = rot_to_quat(R)  # Eigen: x y z w
    data = (list(R[0]) + [t[0]] + list(R[1]) + [t[1]]
            + list(R[2]) + [t[2]] + [0.0, 0.0, 0.0, 1.0])
    fh.write("%s: !!opencv-matrix\n" % name)
    fh.write("   rows: 4\n   cols: 4\n   dt: d\n")
    fh.write("   data: %s\n" % fmt_row(data))
    fh.write("\n# %s 四元数 (x, y, z, w) = %s, 平移 = %s\n\n"
             % (name, fmt_row(q), fmt_row(t)))


def convert(path, outdir):
    d = load_yaml(path)
    if "cam0" not in d or "cam1" not in d:
        sys.exit("不是双目 camchain-imucam 文件：缺少 cam0/cam1")

    os.makedirs(outdir, exist_ok=True)
    c0, c1 = d["cam0"], d["cam1"]

    print("标定文件：%s" % path)
    for key in ("cam0", "cam1"):
        cam = d[key]
        print("  %s: model=%s  %dx%d  topic=%s"
              % (key, cam.get("camera_model"), cam["resolution"][0],
                 cam["resolution"][1], cam.get("rostopic")))

    # ---- 外参
    T_cam0_imu = mat(c0["T_cam_imu"])
    if "T_cn_cnm1" in c1:
        T_cam1_cam0 = mat(c1["T_cn_cnm1"])
    else:
        T_cam1_imu = mat(c1["T_cam_imu"])
        T_cam1_cam0 = T_cam1_imu @ inv_T(T_cam0_imu)
        print("  [i] cam1 上没有 T_cn_cnm1，已用 T_cam1_imu * inv(T_cam0_imu) 推算")

    body_T_cam0 = inv_T(T_cam0_imu)
    body_T_cam1 = T_cam1_cam0 @ body_T_cam0
    check_orthonormal(body_T_cam0, "body_T_cam0")
    check_orthonormal(body_T_cam1, "body_T_cam1")

    timeshift = None
    for key in ("cam0", "cam1"):
        if "timeshift_cam_imu" in d[key]:
            timeshift = float(d[key]["timeshift_cam_imu"])
            break

    # ---- 相机内参
    m0 = MODEL_MAP.get(c0["camera_model"])
    m1 = MODEL_MAP.get(c1["camera_model"])
    if not m0 or not m1:
        sys.exit("不支持的相机模型：%s / %s"
                 % (c0["camera_model"], c1["camera_model"]))
    if m0[0] != m1[0]:
        sys.exit("左右目相机模型不一致（%s vs %s），双目配置需一致"
                 % (m0[0], m1[0]))

    p0 = os.path.join(outdir, "cam0.yaml")
    p1 = os.path.join(outdir, "cam1.yaml")
    write_cam_yaml(p0, c0, m0[0])
    write_cam_yaml(p1, c1, m1[0])

    # ---- VINS 片段
    p2 = os.path.join(outdir, "vins_imu_cam.yaml")
    with open(p2, "w", encoding="utf-8") as fh:
        fh.write("# VINS-Fusion 配置片段，copy 到 euroc_stereo_imu_config.yaml\n")
        fh.write("# 生成来源：%s\n\n" % os.path.basename(path))
        fh.write("camera_model: %s\n" % m0[0])
        fh.write("distortion_model: %s\n" % m0[1])
        fh.write("image_width: %d\n" % int(c0["resolution"][0]))
        fh.write("image_height: %d\n" % int(c0["resolution"][1]))
        fh.write("cam0_calib: cam0.yaml\ncam1_calib: cam1.yaml\n")
        fh.write("image0_topic: %s\n" % c0.get("rostopic", "/cam0/image_raw"))
        fh.write("image1_topic: %s\n\n" % c1.get("rostopic", "/cam1/image_raw"))
        fh.write("# 0=直接用下面外参不优化  1=作初值继续优化  2=完全在线估计\n")
        fh.write("estimate_extrinsic: 0\n\n")
        write_body_T(fh, "body_T_cam0", body_T_cam0)
        write_body_T(fh, "body_T_cam1", body_T_cam1)
        fh.write("# 1=在线估计相机-IMU 时间偏移 td\n")
        fh.write("estimate_td: %d\n" % (1 if timeshift is None else 0))
        if timeshift is not None:
            fh.write("td: %.6f\n" % timeshift)
        fh.write("\n# 双目基线（自检用）= %.6f m\n"
                 % float(np.linalg.norm(T_cam1_cam0[:3, 3])))

    # ---- 自检输出
    print("\n相机-IMU 外参自检：")
    print("  |t_body_cam0| = %.4f m  （IMU 到左目光心的距离，量实物应接近）"
          % float(np.linalg.norm(body_T_cam0[:3, 3])))
    print("  基线 |t_cam1_cam0| = %.4f m"
          % float(np.linalg.norm(T_cam1_cam0[:3, 3])))
    ang = math.degrees(math.acos(max(-1.0, min(1.0,
          (float(np.trace(body_T_cam0[:3, :3])) - 1) / 2))))
    print("  body->cam0 旋转角 = %.2f deg" % ang)
    if timeshift is not None:
        print("  timeshift_cam_imu = %.6f s" % timeshift)
    else:
        print("  Kalibr 未给出 timeshift，建议 VINS 里设 estimate_td: 1")
    if abs(float(np.linalg.norm(T_cam1_cam0[:3, 3])) - 0.1) > 0.3:
        print("  [!] 双目基线偏离 0.1m 较多，确认标定板尺寸、双目是否同一刚体")

    print("\n已生成：\n  %s\n  %s\n  %s" % (p0, p1, p2))
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="Kalibr camchain-imucam.yaml -> VINS-Fusion 配置")
    ap.add_argument("calib", help="camchain-imucam-*.yaml 路径")
    ap.add_argument("--outdir", default=".", help="输出目录，默认当前目录")
    args = ap.parse_args()
    return convert(args.calib, args.outdir)


if __name__ == "__main__":
    sys.exit(main())

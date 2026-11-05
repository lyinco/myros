---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: 6985c3702255a52d22e7a43ec6ef6b82_7a626ce7d1fa11f1a301525400cd780f
    ReservedCode1: XGj52w+AbaWaJUdm6pY65i8yBh3QhI+veKclG55+WwFuBuQeXhbFQKTlMtOw5EE4ECtQsD5zFFPMGGyBEiI7JcVxaHMiEfo9f/xZZpV+nMoX+y1gDI6S6gUo7moPxqecYoALokS7r6c0HTOcLogCrXbtQxh2ofDR940UY1UXeWNqWkfO/q9Cz3Jo4D4=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: 6985c3702255a52d22e7a43ec6ef6b82_7a626ce7d1fa11f1a301525400cd780f
    ReservedCode2: XGj52w+AbaWaJUdm6pY65i8yBh3QhI+veKclG55+WwFuBuQeXhbFQKTlMtOw5EE4ECtQsD5zFFPMGGyBEiI7JcVxaHMiEfo9f/xZZpV+nMoX+y1gDI6S6gUo7moPxqecYoALokS7r6c0HTOcLogCrXbtQxh2ofDR940UY1UXeWNqWkfO/q9Cz3Jo4D4=
---

# VINS-Fusion IMU + 双目相机标定完整流程

> 适用环境：VirtualBox Ubuntu 20.04 客机 / ROS Noetic / 双目 USB 相机 + Hiwonder 10 轴串口 IMU（100Hz，`55 55 ID LEN DATA SUM` 帧）/ VINS-Fusion
> 配套脚本：`tools/kalibr2vins.py`（已用合成真值验证：外参方向、四元数顺序、基线数值均正确）

---

## 0. 先明确：这是一条三步流水线，不能一次做完

| 步骤 | 标什么 | 工具 | 产出 | 复用性 |
|---|---|---|---|---|
| ① 双目标定 | 左右目内参 + 畸变 + cam1 相对 cam0 外参 | `kalibr_calibrate_cameras` | `camchain-*.yaml` | 相机模组不动就不用重做 |
| ② IMU 内参 | 噪声密度 / 随机游走（`acc_n gyr_n acc_w gyr_w`） | `imu_allan.py`（已有） | `imu.yaml` | 与安装位置无关 |
| ③ 相机-IMU 联合标定 | `T_cam_imu` 6 自由度 + 时间偏移 `td` | `kalibr_calibrate_imu_camera` | `camchain-imucam-*.yaml` | 相机或 IMU 一动就要重做 |

顺序不能颠倒：③ 需要 ① 的 `camchain` 和 ② 的 `imu.yaml` 作为输入。

VINS-Fusion 只吃 6 轴数据（加速度 3 + 陀螺 3）；10 轴帧照常解析，发布时取加速度与陀螺两类即可，磁力计不上传。

---

## 1. 环境准备

### 1.1 kalibr（推荐 docker，避开 Python2/3 依赖地狱）

```bash
sudo apt install -y docker.io
sudo usermod -aG docker $USER          # 重新登录生效
xhost +local:docker

mkdir -p ~/data/kalib                  # bag / 标定板 / 输出都放这里
docker run -it --rm \
  -v ~/data/kalib:/data \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=unix$DISPLAY \
  stereo-vision/kalibr:latest bash
```

> 镜像 tag 以 kalibr 官方 wiki（ethz-asl/kalibr → Installation）为准。源码编译时 ROS Noetic 要用 kalibr 的 python3 分支，否则 `kalibr_calibrate_cameras` 会在 `import numpy` 处报 `SyntaxError`。

### 1.2 虚拟机内设备可见性

```bash
# 相机
ls /dev/video*                    # 无输出 → 驱动没起来
lsmod | grep uvcvideo || sudo modprobe uvcvideo
dmesg | grep -i uvc               # 看是否有 isoc 带宽不足等报错

# IMU（USB-TTL 直通后）
ls /dev/ttyUSB*
sudo usermod -aG dialout $USER    # 免 sudo 读串口，重登录生效
```

VirtualBox 直通注意：USB 控制器版本（2.0/3.0）要与宿主设置一致；相机与 IMU 同时直通时带宽不足会导致相机掉帧，标定前先 `rosbag info` 看实际频率。

### 1.3 标定板（apriltag grid）

```bash
kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 \
  --tsize 0.022 --tspace 0.3
```

打印后**用卡尺实测一个 tag 的边长**，按实测值写 yaml（填错会让所有平移按比例错）：

```yaml
# aprilgrid.yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.022      # 实测单 tag 边长，m
tagSpacing: 0.3     # 间距 / tagSize
```

板子贴在硬质平板上，软纸翘曲会直接抬高重投影误差。

---

## 2. 第一步：双目标定

### 2.1 录 bag

```bash
rostopic hz /cam0/image_raw /cam1/image_raw     # 应稳定在 30Hz 左右
rosbag record -O stereo.bag /cam0/image_raw /cam1/image_raw
```

左右目必须同机、同一 ROS master 录制，保证时间戳同源；否则 `T_cn_cnm1` 会带上无法解释的抖动。

### 2.2 采集动作

- 标定板占画面 1/3 以上，铺满中央区域；
- 覆盖上/下/左/右/四角 + 近/中/远三档距离 + ±30°~40° 倾斜；
- 每个姿态**静止约 1 秒**再移动（运动模糊是重投影误差主要来源）；
- 双目一次录 60~80 个姿态，两目同时看到板子。

### 2.3 标定

```bash
kalibr_calibrate_cameras \
  --bag stereo.bag \
  --topics /cam0/image_raw /cam1/image_raw \
  --models pinhole-radtan pinhole-radtan \
  --target aprilgrid.yaml \
  --show-extraction
```

- 视场角 > 120° 的广角双目，模型改 `pinhole-equi`；
- 两目模型**必须一致**（VINS yaml 只有一个 `camera_model` 字段）；
- `--show-extraction` 仅用于确认角点提取，正式跑可去掉。

### 2.4 验收

| 指标 | 合格 | 需返工 |
|---|---|---|
| 单目重投影误差 | < 0.3 px | > 0.8 px |
| 基线 \|t_cam1_cam0\| | 与卡尺值差 < 2 mm | 差 > 5 mm |

误差大优先怀疑：`tagSize` 填错（最常见）、板子不平、镜头失焦、姿态覆盖不足。

产出 `camchain-stereo.yaml`。

---

## 3. 第二步：IMU 内参（Allan 方差）

`ros_imu/imu_allan.py` 的输出直接对应 kalibr 的 `imu.yaml`：

```yaml
# imu.yaml
rostopic: /imu0
update_rate: 100.0
accelerometer_noise_density: 2.0e-03    # m/s^2/sqrt(Hz)
accelerometer_random_walk:   3.0e-04    # m/s^3/sqrt(Hz)
gyroscope_noise_density:     1.5e-04    # rad/s/sqrt(Hz)
gyroscope_random_walk:       2.0e-05    # rad/s^2/sqrt(Hz)
```

三个必须自查：

1. **单位换算**：Hiwonder 上报加速度是 g、陀螺是 dps。发布 ROS topic 前必须换成 m/s²（×9.80665）与 rad/s（×π/180）。这一步错，联合标定会直接崩，且报错不指向原因。
2. **静态零偏**：实测陀螺 X 轴 +3.38 dps（≈0.059 rad/s）。kalibr 会估初始 bias，但建议在 `imu_reader.py` 里先扣静态均值再发布，减少数值病态。
3. **频率**：100Hz 偏低（kalibr 建议 ≥200Hz）。协议若支持 200Hz 档位，先改再标，陀螺残差与 td 精度都会明显改善。

`g_norm` 用廊坊纬度算的 9.8013，与 kalibr 输出的重力模长对齐即证明加速度尺度正确。

---

## 4. 第三步：相机-IMU 联合标定

### 4.1 录数据（同一 bag 同时含图像与 IMU）

```bash
rosbag record -O imu_stereo.bag /cam0/image_raw /cam1/image_raw /imu0
```

必须同一 master 录制。若 IMU 用自带硬件时间戳、相机用驱动时间戳，两者差固定延迟（几百 ms），`--time-calibration` 只能估小量 td（±0.05s 内），救不回整段错位。

### 4.2 采集动作（决定成败的 60~90 秒）

1. 先静止 5 秒（给 bias 基准）；
2. 绕 X / Y / Z 各做 3 次 ±30°~45° 慢速往复旋转（激励陀螺，约 0.5 rad/s）；
3. 沿 X / Y / Z 各做 5 次小幅平移抖动（10~20cm，1~2Hz，激励加速度计）；
4. 最后 20 秒 6 自由度随机混合运动；
5. 全程标定板保持在两目视野内，禁止快速甩动。

### 4.3 标定

```bash
kalibr_calibrate_imu_camera \
  --bag imu_stereo.bag \
  --cam camchain-stereo.yaml \
  --imu imu.yaml \
  --target aprilgrid.yaml \
  --time-calibration
```

`--time-calibration` 用连续时间样条同时估 `td`；运行 5~20 分钟属正常。

### 4.4 产出与验收

`camchain-imucam-imu_stereo.yaml` 关键字段：

```yaml
cam0:
  T_cam_imu: [...]          # 4x4，把 IMU 系的点变到相机系
  timeshift_cam_imu: 0.00xx # 相机相对 IMU 的时间偏移，s
cam1:
  T_cn_cnm1: [...]          # cam1 相对 cam0
```

`results-imucam-imu_stereo.txt` 给重投影误差、陀螺/加计残差、重力模长：

| 检查项 | 期望 | 不合格含义 |
|---|---|---|
| 双目重投影误差 | 与 ① 同量级（< 0.5 px） | 板尺寸或采集质量有问题 |
| 重力模长 | ≈ 9.8013 ± 0.05 | 加速度单位/尺度错 |
| 陀螺残差 | 0.0x rad/s 量级、无系统漂移 | 时间戳错位、陀螺未转 rad/s |
| \|td\| | < 0.05 s | 链路延迟过大，查时间戳来源 |
| 重复性（推荐） | 连标两次：平移差 < 1cm、角度差 < 1° | 激励不足，重录 |

平移物理复核：`|t_body_cam0|` 应等于尺子量出的"IMU 中心到左目光心"距离，差 2cm 以上说明数据有问题。

---

## 5. 第四步：转成 VINS-Fusion 配置

方向是全流程最易错处：

```
Kalibr : T_cam_imu  —— p_cam  = T_cam_imu  * p_imu
VINS   : body_T_cam —— p_body = body_T_cam * p_cam
所以     body_T_cam0 = inv(T_cam0_imu)
        body_T_cam1 = T_cn_cnm1 * body_T_cam0
```

```bash
python tools/kalibr2vins.py camchain-imucam-imu_stereo.yaml --outdir vins_cfg
```

生成 `cam0.yaml`、`cam1.yaml`、`vins_imu_cam.yaml`，后者形如：

```yaml
camera_model: PINHOLE
distortion_model: radtan
image_width: 640
image_height: 480
cam0_calib: cam0.yaml
cam1_calib: cam1.yaml
image0_topic: /cam0/image_raw
image1_topic: /cam1/image_raw

estimate_extrinsic: 0

body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ ... 16 个数 ... ]

body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ ... 16 个数 ... ]

estimate_td: 0
td: 0.003600
```

三个必查：

1. `body_T_cam0/cam1` 是 **4x4 按行铺开**，不是 3x3+平移；
2. 脚本注释打印的四元数是 Eigen 顺序 **(x, y, z, w)**；手填老格式 `extrinsicRotation` 时注意顺序；
3. `estimate_extrinsic` 与 `estimate_td` 别自相矛盾（既设固定 td 又开在线估计）。

最后合并进 `src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml`，imu 段填第二步的四个噪声参数与 `g_norm: 9.8013`。

---

## 6. 第五步：跑起来验证

```bash
roslaunch vins vins_rviz.launch
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml
```

| 现象 | 判定 |
|---|---|
| 能初始化、出轨迹 | 外参方向大体对 |
| 反复 `IMU excitation not enough` | 采集激励不足，或 IMU 频率/噪声参数填错 |
| 初始化后立刻发散 / 轨迹镜像反向 | 外参未取逆（最常见），或四元数顺序错 |
| 静止时漂移大 | 噪声参数过小 + 零偏未扣 |
| 走 10m 闭环回原点 | XY 闭合误差 < 1% 行程即可用 |

注意 VINS 输出的位姿所在坐标系受 `output_path` / pose_graph 配置影响，用 `evo` 对齐时先确认是 body 系还是相机系。

---

## 7. 常见坑速查

| 现象 | 根因 | 处理 |
|---|---|---|
| kalibr 报 `No messages` / 0 帧 | topic 名不符；bag 内是 compressed 图；相机未直通 | `rosbag info` 核对 |
| 虚拟机内 `/dev/video*` 不存在 | uvcvideo 未加载 | `sudo modprobe uvcvideo` + `dmesg \| grep -i uvc` |
| 重投影误差 > 1px | `tagSize` 填错、板翘曲、运动模糊 | 卡尺实测 + 硬板 + 放慢动作 |
| 重力 9.8 变 1.0 量级 | 加速度单位仍是 g | 发布前 ×9.80665 |
| 陀螺残差大、td 跳变 | 陀螺仍是 dps；时间戳不同源 | ×π/180；同一 master 打时间戳 |
| 平移对但轨迹反向 | 用了 `T_cam_imu` 未求逆 | 用 `kalibr2vins.py` 重转 |
| `estimate_extrinsic: 0` 后段漂移 | 外参本身不准 | 改 `1` 在线精修，或重标 |
| 100Hz 下 td 不稳 | 采样率低 | 提升到 200Hz 重标 |

---

## 8. 命令速查

```bash
# ① 双目
rosbag record -O stereo.bag /cam0/image_raw /cam1/image_raw
kalibr_calibrate_cameras --bag stereo.bag \
  --topics /cam0/image_raw /cam1/image_raw \
  --models pinhole-radtan pinhole-radtan --target aprilgrid.yaml

# ② IMU 噪声
python ros_imu/imu_allan.py

# ③ 相机-IMU
rosbag record -O imu_stereo.bag /cam0/image_raw /cam1/image_raw /imu0
kalibr_calibrate_imu_camera --bag imu_stereo.bag \
  --cam camchain-stereo.yaml --imu imu.yaml \
  --target aprilgrid.yaml --time-calibration

# ④ 转换
python tools/kalibr2vins.py camchain-imucam-imu_stereo.yaml --outdir vins_cfg

# ⑤ 跑 VINS
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml
```

---

## 附：为什么不能只用 VINS 在线标定

`estimate_extrinsic: 2` + `estimate_td: 1` 能边跑边估外参，但它**以初始化成功为前提**：初值平移差 > 5cm 或旋转差 > 5° 时，要么初始化失败，要么解出一个错误但自洽的尺度并全程漂。离线 kalibr 负责给初值，在线标定只负责精修（`estimate_extrinsic: 1`）。
*（内容由AI生成，仅供参考）*

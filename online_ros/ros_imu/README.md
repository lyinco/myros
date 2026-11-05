---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: 6985c3702255a52d22e7a43ec6ef6b82_562ce2e8d1b411f1a301525400cd780f
    ReservedCode1: Il2gere/l8GDt52mDwehzdqsrCdOsIt+QdGibHkrHe6/XeLNieZ2ZqoJag1EvX1nhtyXngLOuYCyxmshdMN8LE8b2K9sf7y3tNFRfTQQbySS+wUJxYxUlDqDRWJ31TkZFVYe82MUe9ya9GRc6JGIOs5IBYQIdxSY98eIjOoQpepFNZZOSOkipzFwKtY=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: 6985c3702255a52d22e7a43ec6ef6b82_562ce2e8d1b411f1a301525400cd780f
    ReservedCode2: Il2gere/l8GDt52mDwehzdqsrCdOsIt+QdGibHkrHe6/XeLNieZ2ZqoJag1EvX1nhtyXngLOuYCyxmshdMN8LE8b2K9sf7y3tNFRfTQQbySS+wUJxYxUlDqDRWJ31TkZFVYe82MUe9ya9GRc6JGIOs5IBYQIdxSY98eIjOoQpepFNZZOSOkipzFwKtY=
---

# ros_imu — Hiwonder 10 轴 IMU USB(UART) 读取工具

依据《10轴 IMU 通信协议》（Hiwonder IMU V1.0）实现，从 USB 转串口读取 IMU 数据：

| 数据 | 来源帧 | 单位 | 换算 |
|---|---|---|---|
| 加速度计 | 0x03 | g | `lsb / LSB_per_g`（4G→8192，16G→2048 …） |
| 陀螺仪 | 0x03 | deg/s | `lsb / LSB_per_dps`（1024dps→32，2048dps→16 …） |
| 磁力计 | 0x04 | 原始 LSB | 需磁力计标定后才能转 µT |
| 温度计 | 0x03 末 2 字节 | ℃ | `int16 / 256` |

## 文件

- `imu_protocol.py` —— 协议常量、流式帧解析器（帧头重同步 + 校验和）、各帧解码、配置命令构造
- `imu_reader.py` —— 串口读取主程序（自动扫描 USB 串口、量程回读、终端刷新、CSV 记录、JSON 输出）
- `imu_calibrate.py` —— 静止零偏统计，可选下发陀螺仪校准（SENCAL）并 SAVE
- `imu_verify.py` —— 数据正确性自检：加速度合模、陀螺零偏、噪声、姿态角与加速度互校、磁力计稳定性、帧率、六面法、旋转比对
- `main.py` —— 旧版本，帧格式与协议不符（用了 `0x55` 单字节帧头、ID 0x51/0x52/0x54、固定 11 字节），已由上面两个文件取代

## 协议要点

```
上报帧: 55 55 | ID | LEN | DATA[LEN] | SUM       SUM = 前 4+LEN 字节累加 & 0xFF
配置帧: 55 AF | ID | LEN | DATA[LEN] | SUM       读: ID|0x80, LEN=0
字节序: 小端
```

| 帧 ID | 内容 | DATA 长度 |
|---|---|---|
| 0x01 | Roll/Pitch/Yaw（int16，×180/32768） | 6 |
| 0x02 | q0..q3（int16） | 8 |
| 0x03 | AccXYZ + GyroXYZ + GyroTemp | 14 |
| 0x04 | MagXYZ（原始 LSB） | 6 |
| 0x05 | 气压(uint32 Pa) / 高度(int32 cm) / 气压温度(int16 ×0.01℃) | 10 |

默认配置：115200 8N1、100Hz 回传、加速度 4G、陀螺仪 1024dps、10 轴上报名单 `0x1F`。
6 轴模块不输出 0x04；9 轴需 `RETURNSET` 的 bit3 打开才能拿到磁力计。

## 用法

```bash
pip install pyserial

python imu_reader.py                       # 自动扫描 USB 串口
python imu_reader.py -p COM3               # Windows 指定串口
python imu_reader.py -p /dev/ttyUSB0 -b 460800
python imu_reader.py --query               # 先回读 ACCFSR/GYROFSR 再换算
python imu_reader.py --csv imu_log.csv -t 30   # 录 30 秒 CSV
python imu_reader.py --json --print-hz 10  # JSON 行输出，便于管道/上位机
python imu_reader.py --raw                 # 附带每帧原始 hex，调试用
python imu_reader.py --list-ports          # 列出串口
```

量程被改过时显式指定编码，避免换算错误：

```bash
python imu_reader.py --accfsr 0x03 --gyrofsr 0x03   # 16G / 2048dps
```

## 注意

- 串口被占用（如串口助手、另一份脚本）时会报 `PermissionError 拒绝访问`，需先释放。
- TX/RX 需交叉连接；设备为主动上报模式，脚本只收不发，除非使用 `--query`。
- 协议文档中 `55 AF 04 01 01 8C`、`55 AF 03 01 02 8B` 等示例的校验值与其自身算法不符，本实现一律按算法动态计算（`0x0A`），不要照抄文档常量。
*（内容由AI生成，仅供参考）*

## VINS-Fusion IMU 参数标定（imu_allan.py）

VINS 配置中 5 个 imu 参数全部由本脚本给出（Allan 方差法）：

| 参数 | 含义 | 单位 |
|---|---|---|
| `acc_n` | 加速度计测量噪声密度（白噪声） | m/s^2/sqrt(Hz) |
| `gyr_n` | 陀螺仪测量噪声密度（白噪声） | rad/s/sqrt(Hz) |
| `acc_w` | 加速度计零偏随机游走 | m/s^3/sqrt(Hz) |
| `gyr_w` | 陀螺仪零偏随机游走 | rad/s^2/sqrt(Hz) |
| `g_norm` | 当地重力加速度大小 | m/s^2 |

用法：
```bash
python imu_allan.py -p COM3 --record 7200 --csv imu_static.csv   # 采 2 小时静止数据
python imu_allan.py --analyze imu_static.csv                     # 输出可直接粘贴的配置块
python imu_allan.py -p COM3 --record 7200 --analyze              # 一步到位
```

要点：
- 采集时设备必须完全静止（不碰桌面、远离风扇音箱），时长 >=45 分钟，理想 1~2 小时。
- 白噪声在 tau<=1s 段拟合，随机游走取长 tau 段；数据不足 10 分钟时 acc_w/gyr_w 偏差可达 30%~50%。
- 先跑 imu_verify.py 做 SENCAL 零偏校准并 SAVE，零偏会抬高 acc_w/gyr_w。
- g_norm 也可用公式算：g = 9.780327*(1 + 0.0053024*sin^2(phi) - 0.0000058*sin^2(2*phi)) - 3.086e-6*h，phi 为纬度、h 为海拔(m)。北纬 30 度约 9.7933，北纬 40 度约 9.8017。

查手册的快速路径（不用采数据，精度一般，可作初值）：
- 噪声密度取 datasheet 的 Noise Density：acc_n = ND[ug/sqrt(Hz)] * 9.80665e-6，gyr_n = ND[deg/s/sqrt(Hz)] * pi/180。
- 随机游走手册通常不给，只能实测；经验起步值 acc_w 约 1e-3、gyr_w 约 1e-4。

## ROS 发布 /imu0（imu_ros_node.py，Ubuntu 客机内运行）

把 IMU 数据转成 `sensor_msgs/Imu` 发布到 `/imu0`，供 VINS-Fusion、Kalibr 直接订阅。

### 一次性准备
```bash
sudo apt install -y python3-serial
sudo usermod -aG dialout $USER      # 之后重新登录，否则 /dev/ttyUSB0 无权限
```
VirtualBox：设备 -> USB -> 勾选 CH340（USB-TTL），客机内出现 `/dev/ttyUSB0`；
`ls -l /dev/ttyUSB0` 应显示 `crw-rw---- root dialout`。

### 启动
```bash
source /opt/ros/noetic/setup.bash
cd /path/to/ros_imu              # 脚本与 imu_protocol.py 必须同目录
python3 imu_ros_node.py --list-ports                   # 先确认设备名
python3 imu_ros_node.py -p /dev/ttyUSB0 --bias-seconds 3
```
常用参数：

| 参数 | 说明 |
|---|---|
| `-p, --port` | 串口，缺省自动扫描 CH340/CP210x |
| `--topic` | 默认 `/imu0` |
| `--frame-id` | 默认 `imu_link` |
| `--accfsr` / `--gyrofsr` | 量程编码，默认 4G/1024dps（`0x01`/`0x02`） |
| `--query` | 先向设备回读量程再换算，避免固件改过量程导致比例错 |
| `--bias-seconds N` | 启动时静止 N 秒估陀螺零偏并扣除（建议 3~5s） |
| `--gyro-bias x,y,z` | 手动指定零偏（rad/s） |
| `--acc-n` / `--gyr-n` | 仅用于填协方差，VINS 不读 |

### 验证
```bash
rostopic hz /imu0                     # 应约 100 Hz
rostopic echo -n 3 /imu0              # 静止时 z 轴约 +9.8 m/s^2
```
静止时 `linear_acceleration.z` 约 `+9.80665`，`angular_velocity` 约 0（已扣零偏）。

录制给 Kalibr / VINS：
```bash
rosbag record -O imu0.bag /imu0
```

### 与 VINS-Fusion 对接要点
- 配置 `imu_topic: "/imu0"`，时间戳为到达时刻（rospy 时钟），与相机同 master 即可。
- 单位必须是 m/s^2 与 rad/s，本节点已完成换算；磁力计不上传，纯 6 轴。
- 轴系沿用模块自身坐标（未旋转），若 IMU 与相机相对姿态特殊，由 `extrinsic_R/T` 标定吸收。
- `orientation_covariance[0] = -1`，VINS 不读姿态，无需依赖模块姿态解。

### 常见问题
- `Permission denied: /dev/ttyUSB0` -> `sudo usermod -aG dialout $USER` 后重新登录。
- 频率远低于 100 Hz -> 核对波特率 115200、Windows 端脚本是否仍占用该 COM（同一设备不能两头同时打开）。
- 数据全 0 或数值离谱 -> 先 `python3 imu_ros_node.py --query` 看量程，或临时指定 `--accfsr 0x01 --gyrofsr 0x02`。
- USB 掉线重插后设备名可能变成 `/dev/ttyUSB1`，用 `--list-ports` 确认。

### 未做 / 边界
- 不发布磁力计与温度 topic（VINS 与 Kalibr 均不需要）；需要时可在 `publish_motion` 旁加一个 `sensor_msgs/MagneticField` 发布。
- 未提供 ROS 时间同步；若与相机联合标定，建议先 `sudo apt install chrony` 并用同一 master 录制。

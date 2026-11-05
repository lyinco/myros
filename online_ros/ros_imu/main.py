import serial
import struct
import time

# ================= 配置区域 =================
# 请根据你的系统修改串口号
# Windows 示例: 'COM3'
# Linux 示例: '/dev/ttyUSB0'
# macOS 示例: '/dev/cu.usbserial-XXXX'
SERIAL_PORT = 'COM3'  # '/dev/ttyUSB0'
BAUDRATE = 115200

# 量程配置 (请根据你的IMU手册修改，此处为常见默认值)
ACCEL_RANGE = 16.0  # 单位: g
GYRO_RANGE = 2000.0  # 单位: °/s


# ============================================

def parse_imu_data():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"成功打开串口 {SERIAL_PORT}，等待数据...")
    except Exception as e:
        print(f"打开串口失败: {e}")
        return

    buffer = bytearray()

    try:
        while True:
            # 读取所有可用字节
            if ser.in_waiting:
                raw = ser.read(ser.in_waiting)
                print("RAW:", raw.hex())  # 临时调试用
                buffer.extend(raw)
                buffer.extend(ser.read(ser.in_waiting))

            # 在缓冲区中寻找完整的数据帧
            while len(buffer) >= 11:
                # 寻找帧头 0x55
                if buffer[0] != 0x55:
                    buffer.pop(0)
                    continue

                # 检查校验和 (前10个字节的和，取低8位)
                calc_sum = sum(buffer[0:10]) & 0xFF
                if calc_sum != buffer[10]:
                    # 校验失败，丢弃帧头，继续寻找
                    buffer.pop(0)
                    continue

                # 提取数据类型
                data_type = buffer[1]

                # --- 解析加速度 (TYPE: 0x51) ---
                if data_type == 0x51:
                    # 数据位于第2-7字节 (共6字节，3个int16)
                    ax_raw, ay_raw, az_raw = struct.unpack('<hhh', buffer[2:8])
                    # 温度位于第8-9字节
                    temp_raw = struct.unpack('<h', buffer[8:10])[0]

                    ax = ax_raw / 32768.0 * ACCEL_RANGE
                    ay = ay_raw / 32768.0 * ACCEL_RANGE
                    az = az_raw / 32768.0 * ACCEL_RANGE
                    temp = temp_raw / 100.0

                    print(f"[加速度] X:{ax:7.3f}g Y:{ay:7.3f}g Z:{az:7.3f}g | [温度] {temp:5.1f}℃")

                # --- 解析角速度 (TYPE: 0x52) ---
                elif data_type == 0x52:
                    # 数据位于第2-7字节
                    wx_raw, wy_raw, wz_raw = struct.unpack('<hhh', buffer[2:8])

                    wx = wx_raw / 32768.0 * GYRO_RANGE
                    wy = wy_raw / 32768.0 * GYRO_RANGE
                    wz = wz_raw / 32768.0 * GYRO_RANGE

                    print(f"[陀螺仪] X:{wx:8.2f}°/s Y:{wy:8.2f}°/s Z:{wz:8.2f}°/s")

                # --- 解析磁力计 (TYPE: 0x54) ---
                elif data_type == 0x54:
                    # 数据位于第2-7字节
                    hx_raw, hy_raw, hz_raw = struct.unpack('<hhh', buffer[2:8])
                    # 磁力计原始值单位通常是 LSB，不同型号换算不同，此处先输出原始值
                    # 部分型号需要根据手册乘以系数或进行椭圆拟合校准
                    print(f"[磁力计] 原始值 X:{hx_raw:6d} Y:{hy_raw:6d} Z:{hz_raw:6d}")

                # 成功解析一帧，从缓冲区移除这11个字节
                buffer = buffer[11:]

    except KeyboardInterrupt:
        print("\n程序终止。")
    finally:
        ser.close()


if __name__ == '__main__':
    parse_imu_data()
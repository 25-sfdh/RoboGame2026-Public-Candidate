#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WHEELTEC C50X 串口通信测试脚本（在树莓派上运行）

用法:
  python3 test_comm.py <串口> read               # 只读: 解析小车回传的24字节状态帧
  python3 test_comm.py <串口> move vx vy vz 秒   # 发送目标速度(mm/s)并同时读取状态帧
  python3 test_comm.py <串口> left  [速度] [秒]  # 不带循迹左移, 默认 200mm/s 走 0.3s
  python3 test_comm.py <串口> right [速度] [秒]  # 不带循迹右移, 默认 200mm/s 走 0.3s

示例:
  python3 test_comm.py /dev/ttyACM1 read
  python3 test_comm.py /dev/ttyACM1 move 100 0 0 5
  python3 test_comm.py /dev/ttyACM1 right
  python3 test_comm.py /dev/ttyACM1 left 150 0.5

说明:
  - 开机后前10秒小车不接收控制指令, move 模式请等开机满10秒再发
  - 电机转动需要打开小车上的使能开关
  - 测试时建议把小车悬空, 速度给 50~100 mm/s
"""
import serial
import struct
import sys
import time

HEAD, TAIL = 0x7B, 0x7D
FRAME_LEN = 24  # 小车 -> 上位机 状态帧长度

# ---- 不带循迹的左右横移(只发 vy, 不给 vx/wz, 所以车是原地平移不打转) ----
VY_RIGHT_SIGN = -1              # 与 trace_car_v2 同口径: vy 为正 = 左移, 为负 = 右移
STRAFE_SPEED_DEFAULT = 200      # mm/s, 与任务里 right_speed=200 同口径
STRAFE_SECONDS_DEFAULT = 0.3    # 秒, 与 trace_car_v2 里 move_for(0.3, vy=-200) 同口径


def bcc(data):
    c = 0
    for b in data:
        c ^= b
    return c


def build_speed_frame(vx, vy, vz):
    """构造 11 字节控制帧: 帧头(0x7B) 命令(0) 保留(0) Vx/Vy/Vz(mm/s,int16大端) 校验 帧尾(0x7D)"""
    f = bytearray([HEAD, 0x00, 0x00])
    for v in (vx, vy, vz):
        # 限幅到 int16 范围, 防止超大输入导致 struct 溢出崩溃
        f += struct.pack('>h', max(-32768, min(32767, int(v))))
    f.append(bcc(f[:9]))   # 前9字节异或
    f.append(TAIL)
    return bytes(f)


def read_frames(ser):
    """从串口缓冲里取一帧 24 字节状态帧, 校验帧尾和 BCC, 解析成功返回元组, 否则 None"""
    if ser.in_waiting == 0:
        return None
    data = ser.read(ser.in_waiting)
    i = data.find(bytes([HEAD]))
    if i == -1 or i + FRAME_LEN > len(data):
        return None
    buf = data[i:i + FRAME_LEN]
    if buf[FRAME_LEN - 1] != TAIL or bcc(buf[:FRAME_LEN - 2]) != buf[FRAME_LEN - 2]:
        return None
    vals = struct.unpack('>hhh', buf[2:8])     # Vx Vy Vz, mm/s
    acc = struct.unpack('>hhh', buf[8:14])     # 加速度计
    gyro = struct.unpack('>hhh', buf[14:20])   # 陀螺仪
    volt = struct.unpack('>h', buf[20:22])[0] / 1000.0
    return vals, acc, gyro, volt


def move(ser, vx, vy, vz, seconds=5.0, echo=True):
    """不带循迹: 20Hz 持续发 (vx, vy, vz) 走 seconds 秒, 结尾自动发停车帧。

    vy 就是左右: 正=左移, 负=右移(见 VY_RIGHT_SIGN)。只发 vy、vx=wz=0 时车原地平移。
    返回最后一帧状态 (vals, acc, gyro, volt), 一帧都没读到就是 None。
    """
    end = time.time() + seconds
    last = None
    while time.time() < end:
        ser.write(build_speed_frame(vx, vy, vz))   # 20Hz 持续发送, 断发1秒小车会急停
        r = read_frames(ser)
        if r:
            last = r
            if echo:
                vals, _, _, volt = r
                print('回传: Vx=%5d Vy=%5d Vz=%5d mm/s | 电压=%.2fV' %
                      (vals[0], vals[1], vals[2], volt))
        time.sleep(0.05)
    for _ in range(10):   # 发送停车帧
        ser.write(build_speed_frame(0, 0, 0))
        time.sleep(0.05)
    return last


def move_left(ser, speed=STRAFE_SPEED_DEFAULT, seconds=STRAFE_SECONDS_DEFAULT):
    """不带循迹左移: vy = +speed, 原地横移 seconds 秒。"""
    return move(ser, 0, -VY_RIGHT_SIGN * abs(speed), 0, seconds)


def move_right(ser, speed=STRAFE_SPEED_DEFAULT, seconds=STRAFE_SECONDS_DEFAULT):
    """不带循迹右移: vy = -speed, 原地横移 seconds 秒。"""
    return move(ser, 0, VY_RIGHT_SIGN * abs(speed), 0, seconds)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    port, mode = sys.argv[1], sys.argv[2]
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except serial.SerialException as e:
        print('打开串口失败: %s' % e)
        print('检查: 1) 设备是否存在 (ls /dev/ttyACM*) 2) 是否有权限 (装好udev规则或加sudo)')
        sys.exit(1)
    print('[*] 已打开 %s @ 115200' % port)

    if mode == 'read':
        print('[*] 只读模式: 小车约每秒回传20帧状态数据, Ctrl+C 退出')
        while True:
            r = read_frames(ser)
            if r:
                vals, acc, gyro, volt = r
                print('Vx=%5d Vy=%5d Vz=%5d mm/s | 电压=%.2fV | acc=(%d,%d,%d)' %
                      (vals[0], vals[1], vals[2], volt, acc[0], acc[1], acc[2]))
            else:
                time.sleep(0.02)

    elif mode == 'move':
        vx, vy, vz = int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
        dur = float(sys.argv[6]) if len(sys.argv) > 6 else 5.0
        print('[*] 发送速度 Vx=%d Vy=%d Vz=%d mm/s, 持续 %.1f 秒' % (vx, vy, vz, dur))
        print('[*] 注意: 开机不足10秒指令会被忽略; 请确认使能开关已打开; 建议悬空测试')
        move(ser, vx, vy, vz, dur)
        print('[*] 已发送停车指令')

    elif mode in ('left', 'right'):
        speed = int(sys.argv[3]) if len(sys.argv) > 3 else STRAFE_SPEED_DEFAULT
        dur = float(sys.argv[4]) if len(sys.argv) > 4 else STRAFE_SECONDS_DEFAULT
        name = '左移' if mode == 'left' else '右移'
        print('[*] 不带循迹%s %d mm/s, 持续 %.2f 秒 (原地平移, 不发 vx/wz)' % (name, speed, dur))
        print('[*] 注意: 开机不足10秒指令会被忽略; 请确认使能开关已打开; 建议悬空测试')
        (move_left if mode == 'left' else move_right)(ser, speed, dur)
        print('[*] 已发送停车指令')

    else:
        print(__doc__)
        sys.exit(1)


if __name__ == '__main__':
    main()

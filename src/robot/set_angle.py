# set_angle.py
import threading
import time
import struct
import serial
from fashionstar_uart_sdk import UartServoManager

import serial_ports

import trace_car_v2 as car

# 端口不写死编号：USB 重枚举会把 ttyUSB* 的编号改掉，这里走 serial_ports 解析
# /dev/serial/by-id 下的稳定路径。想看当前解析到哪个节点：python3 serial_ports.py
SERVO_BAUDRATE  = 115200

# 每个舵机 ID 对应的两个固定角度
# 结构：{ servo_id: {0: 角度A, 1: 角度B} }
# 需要改角度时只改这里
FIXED_ANGLES = {
    0: {0: 362.5,   1: 278.6 },
    1: {0: 274.5, 1: 181},
    2: {0: 720.0, 1: 810},   # 多圈累计比标定时多一圈：物理位 = 3.5/90.0 各 +360
}

# ----------------- 全局句柄（懒加载） -----------------
_uart = None
_control = None
_used_servo_ids = set()   # 记录用过哪些 ID，方便统一卸力
_init_lock = threading.Lock()   # 保护下面的懒加载，理由见 init_servo


def init_servo():
    """初始化串口和舵机管理器，只初始化一次。

    mission 脚本里 angle_begin 和 set_angle_012_0 是两条线程并行进来的，都会走到
    这里。没有锁的话两边各 open 一次串口：多出来的句柄没人关，之后两条线程还会往
    同一个口交错发帧，舵机收到的角度指令就乱了。

    两个全局量等构造成功后再一起赋值，失败路径不留下半初始化状态，下次调用会重试。
    """
    global _uart, _control
    with _init_lock:
        if _control is not None:
            return _control

        uart = serial.Serial(
            port=serial_ports.servo_port(),
            baudrate=SERVO_BAUDRATE,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            timeout=0
        )
        try:
            control = UartServoManager(uart, is_debug=True)
        except Exception:
            uart.close()          # 别把句柄漏在失败路径上
            raise
        _uart = uart
        _control = control
        return _control


def set_angle(servo_id, angle, sleep_time=1.0,
              velocity=240.0, power=60000, t_acc=300, t_dec=300,
              lock_wait=0.3):
    """
    控制指定 ID 的舵机转到目标角度（无反馈，等待固定 sleep_time）。

    :param servo_id:   舵机 ID，例如 0 / 1 / 2 / 3
    :param angle:      目标角度，例如 0.0、90.0
    :param sleep_time: 发送指令后等待时间，单位秒
    :param velocity:   速度
    :param power:      功率
    :param t_acc:      加速时间 ms
    :param t_dec:      减速时间 ms
    :param lock_wait:  锁力后等待时间，单位秒
    :return:           当前实际角度
    """
    control = init_servo()
    _used_servo_ids.add(servo_id)

    # 1. 开启舵机锁力，必须执行，否则舵机无动力
    control.stop_on_control_mode(servo_id=servo_id, method=0x11, power=0)
    time.sleep(lock_wait)

    # 2. 发送角度指令
    control.set_servo_angle(
        servo_id=servo_id,
        angle=angle,
        is_mturn=True,
        velocity=velocity,
        power=power,
        t_acc=t_acc,
        t_dec=t_dec
    )

    # 3. 等待舵机转动
    time.sleep(sleep_time)

    cur = control.query_servo_mturn_angle(servo_id)
    print(f"舵机 {servo_id} 转动完成，目标角度：{angle}，当前角度：{cur}")
    return cur


def get_angle(servo_id):
    """查询指定舵机当前的多圈角度，不发送运动命令。

    用于 set_angle_with_feedback 的线性预估：先读当前角度，再根据角度差和速度估算等待时间。
    """
    control = init_servo()
    _used_servo_ids.add(servo_id)
    return control.query_servo_mturn_angle(servo_id)


def set_angle_with_feedback(servo_id, target_angle, tolerance=8,
                            max_retries=10, velocity=240.0,
                            adjust_sleep_time=0.8, safety_factor=1.0,
                            min_sleep=0.2, **kwargs):
    """
    控制指定舵机转到 target_angle，带线性预估与反馈微调。

    策略：
      1. 第一次：读取当前角度，按线性关系估算等待时间
             t = |目标角度 - 当前角度| / velocity * safety_factor
         把该时间作为 sleep_time 传给 set_angle。
      2. 之后：若 |当前角度 - 目标角度| > tolerance，则用固定的
         adjust_sleep_time 重发命令微调，直到偏差 <= tolerance
         或达到 max_retries。

    :param servo_id:          舵机 ID
    :param target_angle:      目标角度
    :param tolerance:         允许偏差，默认 15°
    :param max_retries:       最大重试次数，默认 10
    :param velocity:          舵机速度（°/s），默认 240
    :param adjust_sleep_time: 后续微调每次等待时间（s），默认 0.8
    :param safety_factor:     第一次估算时间的放大系数，默认 1.0（纯线性）
    :param min_sleep:         第一次等待时间下限（s），默认 0.2
    :param kwargs:            透传给 set_angle 的其他参数
                              （如 power、t_acc、t_dec、lock_wait）
    :return:                  最后一次读取到的当前角度
    """
    cur = None
    for attempt in range(1, max_retries + 1):
        if attempt == 1:
            # 1) 先读当前角度
            current_before = get_angle(servo_id)
            diff_angle = abs(target_angle - current_before)
            # 2) 线性估算第一次转动的时间
            sleep_time = max(min_sleep, diff_angle / velocity * safety_factor)+0.5
            print(f"[set_angle_with_feedback] 舵机{servo_id} 当前 {current_before}° "
                  f"-> 目标 {target_angle}°，角度差 {diff_angle:.1f}°，"
                  f"速度 {velocity}°/s，预估等待 {sleep_time:.2f}s")
        else:
            sleep_time = adjust_sleep_time

        # 3) 发送角度命令
        cur = set_angle(
            servo_id,
            target_angle,
            sleep_time=sleep_time,
            velocity=velocity,
            **kwargs,
        )
        diff = abs(cur - target_angle)

        if diff <= tolerance:
            print(f"[set_angle_with_feedback] 舵机{servo_id} 第 {attempt} 次成功："
                  f"目标 {target_angle}，当前 {cur}，偏差 {diff:.2f} <= {tolerance}")
            return cur

        print(f"[set_angle_with_feedback] 舵机{servo_id} 第 {attempt} 次偏差过大："
              f"目标 {target_angle}，当前 {cur}，偏差 {diff:.2f} > {tolerance}，"
              f"重新发送（等待 {adjust_sleep_time}s）")

    print(f"[set_angle_with_feedback] 舵机{servo_id} 达到最大重试 {max_retries}，"
          f"最后一次当前 {cur}，偏差 {abs(cur - target_angle):.2f}")
    return cur


def set_fixed_angle(servo_id, mode, sleep_time=0.1,
                    velocity=400.0, power=30000, t_acc=150, t_dec=150,
                    hold=True, hold_power=0):
    """
    控制指定 ID 的舵机转到它自己的固定角度。

    :param servo_id:   舵机 ID，取值 0 / 1 / 2
    :param mode:       0 / 1，选择该 ID 的两个固定角度之一
    :param sleep_time: 等待时间（秒）
    :param velocity:   速度（0~65535）
    :param power:      功率（0~65535）
    :param t_acc:      加速时间 ms（0~65535）
    :param t_dec:      减速时间 ms（0~65535）
    :param hold:       True  -> 转动完成后保持锁力（不卸力）
                       False -> 转动完成后立即对该舵机卸力
    :return:           当前实际角度
    """
    if servo_id not in FIXED_ANGLES:
        raise ValueError(f"不支持的舵机 ID：{servo_id}，只支持 {list(FIXED_ANGLES.keys())}")
    if mode not in FIXED_ANGLES[servo_id]:
        raise ValueError(f"ID={servo_id} 的 mode 只支持 {list(FIXED_ANGLES[servo_id].keys())}")

    angle = FIXED_ANGLES[servo_id][mode]
    print(f"[固定角度] ID={servo_id}, mode={mode} -> {angle}°, hold={hold}")

    # 1. 转动（set_angle 内部不做卸力）
    cur = set_angle(
        servo_id=servo_id,
        angle=angle,
        sleep_time=sleep_time,
        velocity=velocity,
        power=power,
        t_acc=t_acc,
        t_dec=t_dec
    )

    # 2. 根据 hold 决定是否卸力
    if not hold:
        control = init_servo()
        control.stop_on_control_mode(servo_id, method=0x10, power=0)
        _used_servo_ids.discard(servo_id)   # 已卸力，避免 release_servo 重复卸
        print(f"舵机 {servo_id} 已卸力")

    return cur


def set_angles_sync(angle_dict, sleep_time=1.5,
                    velocity=150.0, power=60000, t_acc=300, t_dec=300,
                    lock_wait=0.3, hold=True):
    """
    同步控制多个舵机运动到各自的目标角度（多圈模式）。

    所有舵机在同一帧指令中发送，因此会同时启动，按各自的参数运动。

    :param angle_dict: 字典 {servo_id: target_angle}
                       例如 {0: 10.0, 1: 720.0, 2: -45.0}
    :param sleep_time: 发送指令后等待时间，单位秒
    :param velocity:   目标转速，单位 °/s
    :param power:      功率限制，单位 mW（0 表示不限制）
    :param t_acc:      加速时间 ms
    :param t_dec:      减速时间 ms
    :param lock_wait:  锁力后等待时间，单位秒
    :param hold:       True  -> 转动完成后保持锁力
                       False -> 转动完成后对所有舵机卸力
    :return:           字典 {servo_id: 当前角度}
    """
    if not angle_dict:
        print("[同步] angle_dict 为空，未执行任何操作")
        return {}

    control = init_servo()
    servo_ids = list(angle_dict.keys())

    # 1. 逐个开启锁力（必须先锁力，否则舵机无动力）
    for sid in servo_ids:
        control.stop_on_control_mode(servo_id=sid, method=0x11, power=0)
        _used_servo_ids.add(sid)
    time.sleep(lock_wait)

    # 2. 构造每个舵机的同步命令数据
    # 格式：<BiHHHH> = ID(B), 角度(i, 0.1°), 速度(H, 0.1dps),
    #                  t_acc(H), t_dec(H), power(H)
    velocity_raw = int(velocity * 10)
    command_data_list = []
    for sid in servo_ids:
        angle = angle_dict[sid]
        data = struct.pack(
            '<BiHHHH',
            sid,
            int(angle * 10),      # 角度：度 -> 0.1°
            velocity_raw,         # 速度：°/s -> 0.1dps
            t_acc,
            t_dec,
            power
        )
        command_data_list.append(data)

    # 3. 发送同步多圈角度控制命令（基于目标速度）
    print(f"[同步] 同步控制舵机 {servo_ids} -> {angle_dict}")
    control.send_sync_multiturnanglebyvelocity(
        command_id=control.CODE_SET_SERVO_ANGLE_MTURN_BY_VELOCITY,  # 15
        servo_num=len(servo_ids),
        command_data_list=command_data_list
    )

    # 4. 等待舵机转动完成
    time.sleep(sleep_time)

    # 5. 查询每个舵机的实际角度
    result = {}
    for sid in servo_ids:
        cur = control.query_servo_mturn_angle(sid)
        result[sid] = cur
        print(f"舵机 {sid} 转动完成，目标角度：{angle_dict[sid]}，当前角度：{cur}")

    # 6. 若不保持锁力，则统一卸力
    if not hold:
        for sid in servo_ids:
            control.stop_on_control_mode(sid, method=0x10, power=0)
            _used_servo_ids.discard(sid)
        print(f"舵机 {servo_ids} 已卸力")

    return result

#-----------------多圈角度控制----------------------------
def set_fixed_angles_sync(mode, sleep_time=1.5,
                          velocity=200.0, power=10000, t_acc=150, t_dec=150,
                          lock_wait=0.3, hold=True):
    """
    同步控制 FIXED_ANGLES 中所有舵机转动到各自 mode 对应的固定角度。

    :param mode:       0 或 1，选择每个 ID 的两个固定角度之一
    :param sleep_time: 等待时间（秒）
    :param velocity:   速度（°/s）
    :param power:      功率（mW）
    :param t_acc:      加速时间 ms
    :param t_dec:      减速时间 ms
    :param lock_wait:  锁力后等待时间，单位秒
    :param hold:       True 保持锁力，False 转动完成后卸力
    :return:           字典 {servo_id: 当前角度}
    """
    # 从 FIXED_ANGLES 中提取出该 mode 下所有舵机的目标角度
    angle_dict = {}
    for sid, modes in FIXED_ANGLES.items():
        if mode in modes:
            angle_dict[sid] = modes[mode]
        else:
            print(f"[固定同步] 舵机 {sid} 不支持 mode={mode}，已跳过")

    if not angle_dict:
        raise ValueError(f"没有任何舵机支持 mode={mode}，请检查 FIXED_ANGLES")

    print(f"[固定同步] mode={mode} -> {angle_dict}, hold={hold}")
    return set_angles_sync(
        angle_dict=angle_dict,
        sleep_time=sleep_time,
        velocity=velocity,
        power=power,
        t_acc=t_acc,
        t_dec=t_dec,
        lock_wait=lock_wait,
        hold=hold
    )

#-----------------------------------------------------
def set_angle_012_0():
    set_angles_sync(
        angle_dict={0: FIXED_ANGLES[0][0], 1: FIXED_ANGLES[1][0], 2: FIXED_ANGLES[2][0]},
        sleep_time=0.1,
        velocity=400.0,
        power=60000,
        t_acc=300,
        t_dec=300,
        hold=True          # 转动完成后保持锁力
    )

def set_angle_12_0():
    set_angles_sync(
        angle_dict={1: FIXED_ANGLES[1][0], 2: FIXED_ANGLES[2][0]},
        sleep_time=0.1,
        velocity=400.0,
        power=60000,
        t_acc=300,
        t_dec=300,
        hold=True          # 转动完成后保持锁力
    )

def set_angle_012_1():
    set_angles_sync(
        angle_dict={0: FIXED_ANGLES[0][1], 1: FIXED_ANGLES[1][1], 2: FIXED_ANGLES[2][1]},
        sleep_time=0.1,
        velocity=400.0,
        power=60000,
        t_acc=300,
        t_dec=300,
        hold=True          # 转动完成后保持锁力
    )

def set_angle_12_1():
    set_angles_sync(
        angle_dict={1: FIXED_ANGLES[1][1], 2: FIXED_ANGLES[2][1]},
        sleep_time=0.1,
        velocity=400.0,
        power=60000,
        t_acc=300,
        t_dec=300,
        hold=True          # 转动完成后保持锁力
    )

def release_servo():
    """释放所有用过的舵机，并关闭串口。"""
    global _uart, _control

    with _init_lock:
        if _control is not None:
            for sid in _used_servo_ids:
                try:
                    _control.stop_on_control_mode(sid, method=0x10, power=0)
                except Exception as e:
                    print(f"舵机 {sid} 卸力失败: {e}")

        if _uart is not None:
            _uart.close()

        _uart = None
        _control = None
        _used_servo_ids.clear()


    
    
#---------------------------------------------
angle_1  = -1986.8          # 高处橙色方块第一个
angle_2  = angle_1 - 165.4  # 高处橙色第二个
angle_3  = angle_1 - 205.1  # 高处橙色识别
angle_4  = angle_1 + 103.9  # 低处方块识别
angle_5  = angle_1 + 338    # 低处方块第一个
angle_6  = angle_1 + 178.1  # 低处方块第二个
angle_7  = angle_1 + 246.4  # 搭建区下面有一个
angle_8  = angle_1 + 415    # 搭建区放置最低
angle_9  = angle_1 - 91.7   # 搭建区下面有三个
angle_10 = angle_1 - 434.4  # 搭建区下面有五个
angle_11 = angle_1 - 610    #搭建区下面有六个

def main_servo_action_1():
    set_angle(3, angle_10, 4)             # 原 angle_5，抓第五个方块位置
    set_fixed_angle(2, 1)
    set_angle(3, angle_10 - 50, 1)


def main_servo_action_2():
    set_angle(3, (angle_8+angle_10)/2, 6) 
    set_angle(3, angle_8, 6)              # 原 angle_put，放置方块最低
    set_fixed_angle(2, 0)



if __name__ == "__main__":
    try:
        init_servo()
        #set_angle(3,-2878,3.9)
        #set_angle(3,-1916,3.9)
        #set_angle(3,-2050,1)
        #set_angle_with_feedback(3,angle_1)
        set_angle_with_feedback(3,angle_3)
        #set_angle_with_feedback(3,angle_9)
    finally:
        release_servo()
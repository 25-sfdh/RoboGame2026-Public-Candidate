#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双循迹 + IMU 路口转向封装模块。

流程：
    FOLLOW → BRAKE → ADVANCE(前进0.4s) → IMU_TURN
           → IMU_SETTLE → ALIGN → SETTLE → FOLLOW

正T 例外：
    FOLLOW → BRAKE → SETTLE → FOLLOW

对外主入口：
    run(route, speed=300, dry_run=None, debug_sensor=False, stop_after=None)
    run_for(seconds, speed=300)

run(route, ...) 参数：
    route       : [(target, after), ...]
    speed       : 前进速度 mm/s
    dry_run     : True 时只打印，不打开硬件
    debug_sensor: True 时打印前板黑线状态
    stop_after  : 数字秒数，从开始循迹起 N 秒后强制停车；None 不启用

route 列表每项 (target, after)：
    target : 'L' 左T / 'R' 右T / 'T' 正T   （也支持 2/3/1）
    after  : 'continue'     转向后继续循迹
             数字秒数        转向后循迹这么多秒后停
             'stop'         转向后 0.1s 停
             'auto'         正T停，左右T继续

丢线策略：
    前有 后有 → 双板融合
    前有 后丢 → 只用前板循迹，继续走
    前丢 后有 → 停车
    前后都丢 → 停车
"""

import struct
import time

import distance as _distance      # 复用 distance.py 里的 get_distance()
import serial_ports


# ============================== 可调参数 ==============================
DRY_RUN = False
# 端口不写死编号：USB 重枚举会把 ttyACM* 的编号改掉，这里走 serial_ports 解析
# /dev/serial/by-id 下的稳定路径。想看当前解析到哪个节点：python3 serial_ports.py
SERIAL_PORT = serial_ports.chassis_port()
BAUDRATE = 115200

CONTROL_HZ = 50
CONTROL_PERIOD = 1.0 / CONTROL_HZ

# ---- 速度 ----
FOLLOW_VX = 300
ADVANCE_VX = 300

# ---- 转弯前前进时间 ----
ADVANCE_TIME = 0.3

#--------测距-------------------------------
DISTANCE_POLL_INTERVAL = 0.05     # 多少秒读一次超声波
DISTANCE_OK_FRAMES = 3            # 连续 N 次距离达标才算数，防抖动
DISTANCE_MIN_CM = 2.0             # 低于这个值认为是噪声，忽略
DISTANCE_MAX_CM = 400.0           # 高于这个值认为超量程，忽略

# ============================== 后退 + 转向 ==============================
BACK_SPEED_DEFAULT = 200            # 默认后退速度 mm/s
BACK_EXTRA_TIME_DEFAULT = 0.4       # 后板找到线后再退多少秒
BACK_TIMEOUT_S = 6.0                # 最多后退多少秒，防卡死

# ---- 循迹 P 控制 ----
KY = 35.0
KW = 60.0
MAX_VY = 80
MAX_WZ = 300
VY_RIGHT_SIGN = -1
YAW_SIGN = -1

FRONT_BLACK_LEVEL = 1
REAR_BLACK_LEVEL = 1

# ---- 路口检测 ----
INTERSECTION_MIN_ACTIVE = 5
INTERSECTION_CONFIRM_FRAMES = 5
INTERSECTION_BRAKE_TIME = 0.20

# ---- 直角检测 ----
CORNER_SIDE_ACTIVE = 4

# ---- IMU 粗转 ----
IMU_TURN_WZ_ABS = 450
IMU_TURN_STOP_DEG_ABS = 82.0
IMU_TURN_TIMEOUT_S = 10.0
IMU_SETTLE_TIME = 0.30

# ---- 精对准 ----
ALIGN_KY = 25.0
ALIGN_KW = 45.0
ALIGN_MAX_VY = 50
ALIGN_MAX_WZ = 80
ALIGN_SEARCH_WZ = 40
ALIGN_INITIAL_WAIT_S = 0.30
TURN_ALIGN_ERROR = 0.5
TURN_ALIGN_CONFIRM_FRAMES = 5
TURN_ALIGN_TIMEOUT_S = 4.0
TURN_SETTLE_TIME = 0.15
INTERSECTION_COOLDOWN_S = 1.5

STOP_FRAME_COUNT = 6
STOP_INTERVAL = 0.03

FRONT_PINS = [17, 23, 24, 27, 22, 25, 5]
REAR_PINS = [21, 20, 19, 16, 13, 18, 6]
WEIGHTS = [-3, -2, -1, 0, 1, 2, 3]


# ============================== 状态名 ==============================
FOLLOW = "FOLLOW"
BRAKE = "BRAKE"
ADVANCE = "ADVANCE"
IMU_TURN = "IMU_TURN"
IMU_SETTLE = "IMU_SETTLE"
ALIGN = "ALIGN"
SETTLE = "SETTLE"
STOPPED = "STOPPED"


# ============================== 串口协议 ==============================
HEAD = 0x7B
TAIL = 0x7D
FRAME_LEN = 24
IMU_HZ = 20.0
GYRO_RATIO = 0.00026644
DEG = 57.29578


# ============================== 内部状态 ==============================
_state = STOPPED
_follow_speed = FOLLOW_VX
_stop_deadline = None
_turn_target = None
_turn_after = "continue"
_turn_stop_delay = 0.1
_state_t0 = 0.0
_turn_start_yaw = None
_align_count = 0
_cooldown_until = 0.0
_route = []
_route_index = 0
_confirm_count = 0

_serial = None
_imu = None
_front = []
_rear = []


# ============================== 底层工具 ==============================
def _bcc(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def _build_frame(vx, vy, wz):
    frame = bytearray([HEAD, 0x00, 0x00])
    for value in (vx, vy, wz):
        value = max(-32768, min(32767, int(value)))
        frame += struct.pack(">h", value)
    frame.append(_bcc(frame[:9]))
    frame.append(TAIL)
    return bytes(frame)


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class _IMU:
    def __init__(self):
        self.yaw = 0.0

    def feed(self, raw):
        if len(raw) != FRAME_LEN:
            return False
        if raw[0] != HEAD or raw[-1] != TAIL:
            return False
        if _bcc(raw[:-2]) != raw[-2]:
            return False
        gyro_z = struct.unpack(">h", raw[18:20])[0]
        self.yaw += gyro_z * GYRO_RATIO * DEG / IMU_HZ
        return True


def _read_board(sensors, black_level):
    raw = [int(s.value) for s in sensors]
    black = [1 if v == black_level else 0 for v in raw]
    n = sum(black)
    if n == 0:
        return black, None
    err = sum(w * b for w, b in zip(WEIGHTS, black)) / n
    return black, err


def _read_imu():
    n = _serial.in_waiting
    if n <= 0:
        return
    buf = _serial.read(n)
    i = 0
    while i + FRAME_LEN <= len(buf):
        if buf[i] == HEAD and _imu.feed(buf[i:i + FRAME_LEN]):
            i += FRAME_LEN
        else:
            i += 1


def _send(vx, vy, wz):
    _serial.write(_build_frame(vx, vy, wz))


def _stop_chassis():
    for _ in range(STOP_FRAME_COUNT):
        _serial.write(_build_frame(0, 0, 0))
        time.sleep(STOP_INTERVAL)


# ============================== 路口识别 ==============================
def _is_cross_or_t(black):
    return sum(black) >= INTERSECTION_MIN_ACTIVE


def _is_corner(black):
    left = sum(black[0:CORNER_SIDE_ACTIVE]) >= CORNER_SIDE_ACTIVE
    right = sum(black[-CORNER_SIDE_ACTIVE:]) >= CORNER_SIDE_ACTIVE
    return left or right


def _is_intersection(black):
    return _is_cross_or_t(black) or _is_corner(black)


# ============================== 硬件开/关 ==============================
def _init_hardware():
    global _serial, _imu, _front, _rear

    from gpiozero import DigitalInputDevice
    import serial

    _front = [DigitalInputDevice(p) for p in FRONT_PINS]
    _rear = [DigitalInputDevice(p) for p in REAR_PINS]
    port = serial_ports.chassis_port()   # 开之前重新解析，不用可能已过期的导入期结果
    _serial = serial.Serial(
        port, BAUDRATE, timeout=0.1, write_timeout=0.2
    )
    _imu = _IMU()
    print(f"硬件初始化完成: {port} @ {BAUDRATE}")


def _shutdown():
    global _serial

    if _serial is not None:
        try:
            _stop_chassis()
            print("已连续发送停车帧")
        except Exception as e:
            print(f"停车帧失败: {e}")
        try:
            _serial.close()
        except Exception:
            pass
        _serial = None

    for s in _front + _rear:
        try:
            s.close()
        except Exception:
            pass
    print("GPIO 已关闭，程序结束")


# ============================== 三个底层 API ==============================
def start_trace(speed):
    global _state, _follow_speed, _stop_deadline, _cooldown_until, _align_count

    _follow_speed = int(speed)
    _stop_deadline = None
    _align_count = 0
    _cooldown_until = time.monotonic() + INTERSECTION_COOLDOWN_S
    _state = FOLLOW
    print(f"[API] start_trace vx={_follow_speed}")


def end_trace_by_time(seconds):
    global _stop_deadline
    _stop_deadline = time.monotonic() + float(seconds)
    print(f"[API] end_trace_by_time {seconds}s")


def cross_turn(target, after="auto"):
    global _state, _turn_target, _turn_after, _turn_stop_delay
    global _state_t0, _align_count, _cooldown_until

    if target in ("T", "S", "正T", 0, 1):
        _turn_target = "T"
    elif target in ("L", "左T", 2):
        _turn_target = "L"
    elif target in ("R", "右T", 3):
        _turn_target = "R"
    else:
        raise ValueError(f"未知路口动作: {target}")

    if isinstance(after, (int, float)):
        _turn_after = "stop"
        _turn_stop_delay = float(after)
    elif after == "auto":
        _turn_after = "stop" if _turn_target == "T" else "continue"
        _turn_stop_delay = 0.1
    elif after == "stop":
        _turn_after = "stop"
        _turn_stop_delay = 0.1
    elif after == "continue":
        _turn_after = "continue"
        _turn_stop_delay = 0.1
    else:
        raise ValueError(f"未知 after 动作: {after}")

    _state_t0 = time.monotonic()
    _align_count = 0
    _cooldown_until = time.monotonic() + INTERSECTION_COOLDOWN_S
    _state = BRAKE
    print(f"[API] cross_turn target={_turn_target} after={after}")


# ============================== 状态机 ==============================
def _step(front_err, rear_err, now):
    global _state, _state_t0, _turn_start_yaw, _align_count

    # ---------------- FOLLOW ----------------
    if _state == FOLLOW:
        if front_err is None and rear_err is None:
            return 0, 0, 0

        if rear_err is None:
            vy = _clamp(VY_RIGHT_SIGN * KY * front_err, MAX_VY)
            return _follow_speed, int(round(vy)), 0

        if front_err is None:
            return 0, 0, 0

        lat = (front_err + rear_err) / 2.0
        yaw = front_err - rear_err
        vy = _clamp(VY_RIGHT_SIGN * KY * lat, MAX_VY)
        wz = _clamp(YAW_SIGN * KW * yaw, MAX_WZ)
        return _follow_speed, int(round(vy)), int(round(wz))

    # ---------------- 路口刹车 ----------------
    if _state == BRAKE:
        if now - _state_t0 >= INTERSECTION_BRAKE_TIME:
            if _turn_target == "T":
                _state = SETTLE
            else:
                _state = ADVANCE
            _state_t0 = now
        return 0, 0, 0

    # ---------------- 转弯前前进 0.4s ----------------
    if _state == ADVANCE:
        if now - _state_t0 >= ADVANCE_TIME:
            print(f"[ADVANCE] 前进 {ADVANCE_TIME}s 结束 -> IMU_TURN")
            _turn_start_yaw = None
            _state = IMU_TURN
            _state_t0 = now
            return 0, 0, 0
        return ADVANCE_VX, 0, 0

    # ---------------- IMU 粗转 ----------------
    if _state == IMU_TURN:
        if _turn_start_yaw is None:
            _turn_start_yaw = _imu.yaw

        if _turn_target == "L":
            wz_cmd = IMU_TURN_WZ_ABS
            stop_deg = IMU_TURN_STOP_DEG_ABS
        else:
            wz_cmd = -IMU_TURN_WZ_ABS
            stop_deg = -IMU_TURN_STOP_DEG_ABS

        delta = _imu.yaw - _turn_start_yaw
        reached = (delta >= stop_deg) if stop_deg > 0 else (delta <= stop_deg)

        if reached or (now - _state_t0) > IMU_TURN_TIMEOUT_S:
            print(f"[TURN] done delta={delta:+.2f}")
            _state = IMU_SETTLE
            _state_t0 = now
            return 0, 0, 0
        return 0, 0, wz_cmd

    # ---------------- 粗转停稳 ----------------
    if _state == IMU_SETTLE:
        if now - _state_t0 >= IMU_SETTLE_TIME:
            _state = ALIGN
            _state_t0 = now
            _align_count = 0
        return 0, 0, 0

    # ---------------- 双板精对准 ----------------
    if _state == ALIGN:
        if now - _state_t0 > TURN_ALIGN_TIMEOUT_S:
            print("[TURN] align timeout")
            _state = SETTLE
            _state_t0 = now
            return 0, 0, 0
        if now - _state_t0 < ALIGN_INITIAL_WAIT_S:
            return 0, 0, 0

        if front_err is None and rear_err is None:
            d = 1 if _turn_target == "L" else -1
            _align_count = 0
            return 0, 0, ALIGN_SEARCH_WZ * d

        if rear_err is None:
            vy = _clamp(VY_RIGHT_SIGN * ALIGN_KY * front_err, ALIGN_MAX_VY)
            return 0, int(round(vy)), 0

        if front_err is None:
            d = 1 if _turn_target == "L" else -1
            _align_count = 0
            return 0, 0, ALIGN_SEARCH_WZ * d

        lat = (front_err + rear_err) / 2.0
        yaw = front_err - rear_err
        if abs(lat) < TURN_ALIGN_ERROR and abs(yaw) < TURN_ALIGN_ERROR:
            _align_count += 1
            if _align_count >= TURN_ALIGN_CONFIRM_FRAMES:
                print("[TURN] align OK")
                _state = SETTLE
                _state_t0 = now
                return 0, 0, 0
        else:
            _align_count = 0

        vy = _clamp(VY_RIGHT_SIGN * ALIGN_KY * lat, ALIGN_MAX_VY)
        wz = _clamp(YAW_SIGN * ALIGN_KW * yaw, ALIGN_MAX_WZ)
        return 0, int(round(vy)), int(round(wz))

    # ---------------- 收尾 ----------------
    if _state == SETTLE:
        if now - _state_t0 >= TURN_SETTLE_TIME:
            start_trace(_follow_speed)
            if _turn_after == "stop":
                end_trace_by_time(_turn_stop_delay)
        return 0, 0, 0

    return 0, 0, 0


# ============================== 对外主入口 ==============================
def run(route, speed=300, dry_run=None, debug_sensor=False, stop_after=None):
    """按 route 列表跑完所有路口。

    参数：
        route        : [(target, after), ...]
        speed        : 前进速度 mm/s
        dry_run      : True 只打印不打开硬件
        debug_sensor : True 打印前板黑线状态
        stop_after   : 数字秒数，从开始循迹起 N 秒后强制停车；None 不启用
    """
    global _route, _route_index, _confirm_count, _state

    if dry_run is None:
        dry_run = DRY_RUN

    _route = list(route)
    _route_index = 0
    _confirm_count = 0

    if dry_run:
        print("DRY_RUN：不打开硬件，只打印路线")
        for i, (t, a) in enumerate(_route, 1):
            print(f"  路口 {i}: target={t} after={a}")
        if stop_after is not None:
            print(f"  stop_after = {stop_after}s")
        return

    _init_hardware()

    try:
        start_trace(speed)

        # 如果指定了 stop_after，就直接设定停车截止时间
        if stop_after is not None:
            end_trace_by_time(stop_after)

        while _state != STOPPED:
            now = time.monotonic()

            f_black, f_err = _read_board(_front, FRONT_BLACK_LEVEL)
            r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
            _read_imu()

            if debug_sensor and sum(f_black) > 0:
                print(f"[SENSOR] F={f_black} count={sum(f_black)} "
                      f"corner={_is_corner(f_black)}")

            # 定时停车检查（stop_after 或 end_trace_by_time 生效点）
            if _stop_deadline is not None and now >= _stop_deadline:
                _stop_chassis()
                print("[SAFE] end_trace_by_time 到期 -> STOP")
                _state = STOPPED
                break

            # 路口检测
            if _state == FOLLOW:
                if _is_intersection(f_black) and now >= _cooldown_until:
                    _confirm_count += 1
                    if _confirm_count >= INTERSECTION_CONFIRM_FRAMES:
                        _confirm_count = 0
                        if _route_index < len(_route):
                            target, after = _route[_route_index]
                            _route_index += 1
                            print(f"[ROUTE] 第 {_route_index} 个路口: "
                                  f"target={target} after={after} "
                                  f"black={f_black}")
                            cross_turn(target, after)
                        else:
                            end_trace_by_time(0.1)
                else:
                    _confirm_count = 0

            vx, vy, wz = _step(f_err, r_err, now)
            _send(vx, vy, wz)
            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C")
    except Exception as e:
        print(f"\n程序异常: {e}")
    finally:
        _shutdown()


def back_then_turn(target, after="continue",
                   back_speed=BACK_SPEED_DEFAULT,
                   extra_time=BACK_EXTRA_TIME_DEFAULT,
                   timeout=BACK_TIMEOUT_S,
                   debug=False):
    """后退直到后板找到黑线，再后退 extra_time 秒，然后转向。

    target : 'L' 左T / 'R' 右T / 'T' 正T
    after  : 'continue' / 'stop' / 数字秒数 / 'auto'
    back_speed : 后退速度 mm/s（正值，内部会取负）
    extra_time : 后板找到黑线后继续后退多少秒
    timeout    : 最多后退多少秒，超过就放弃并转向
    debug      : True 时打印后板状态
    """
    global _state, ADVANCE_TIME

    _init_hardware()
    saved_advance = ADVANCE_TIME

    try:
        # ---------- 阶段 1：后退直到后板找到黑线 ----------
        print(f"[BACK] 开始后退 vx=-{back_speed}")
        line_found_t = None
        start_t = time.monotonic()

        while True:
            now = time.monotonic()

            r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
            _read_imu()

            if debug:
                print(f"[BACK] R={r_black} err={r_err} "
                      f"found={line_found_t is not None}")

            if now - start_t > timeout:
                print(f"[BACK] 超时 {timeout}s，放弃后退，直接转向")
                break

            if line_found_t is None:
                if r_err is not None:
                    line_found_t = now
                    print(f"[BACK] 后板找到黑线 -> 继续后退 {extra_time}s")
                else:
                    _send(-back_speed, 0, 0)
            else:
                if now - line_found_t >= extra_time:
                    print("[BACK] 后退结束")
                    break
                _send(-back_speed, 0, 0)

            time.sleep(CONTROL_PERIOD)

        # 停车停稳
        _send(0, 0, 0)
        time.sleep(0.2)

        # ---------- 阶段 2：转向（跳过 ADVANCE 前进）----------
        ADVANCE_TIME = 0.0
        print(f"[BACK] 开始转向 target={target} after={after}（跳过前进）")
        cross_turn(target, after)

        # ---------- 阶段 3：推进状态机直到结束 ----------
        while _state != STOPPED:
            now = time.monotonic()

            f_black, f_err = _read_board(_front, FRONT_BLACK_LEVEL)
            r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
            _read_imu()

            if _stop_deadline is not None and now >= _stop_deadline:
                _stop_chassis()
                print("[SAFE] end_trace_by_time 到期 -> STOP")
                _state = STOPPED
                break

            vx, vy, wz = _step(f_err, r_err, now)
            _send(vx, vy, wz)
            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("\n[BACK] 收到 Ctrl+C")

    finally:
        ADVANCE_TIME = saved_advance
        _shutdown()

#---------------------------------------测距--------------------------------
def run_until_distance(target_cm, speed=300, timeout_s=None, debug=False):
    """向前循迹，直到超声波距离 <= target_cm 停车。

    target_cm  : 目标停止距离（厘米）
    speed      : 循迹速度 mm/s
    timeout_s  : 最多跑多少秒；None 不限制
    debug      : True 时打印距离
    """
    global _state, _stop_deadline

    _init_hardware()
    try:
        start_trace(speed)
        print(f"[DIST] 循迹直到距离 <= {target_cm} cm，speed={speed}")

        start_t = time.monotonic()
        next_poll = start_t
        ok_count = 0

        while _state != STOPPED:
            now = time.monotonic()

            # ---------- 循迹控制 ----------
            f_black, f_err = _read_board(_front, FRONT_BLACK_LEVEL)
            r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
            _read_imu()

            # ---------- 超时保护 ----------
            if timeout_s is not None and now - start_t > timeout_s:
                print(f"[DIST] 超时 {timeout_s}s -> STOP")
                _stop_chassis()
                _state = STOPPED
                break

            # ---------- 定时停车（外部设的 stop_deadline 仍生效）----------
            if _stop_deadline is not None and now >= _stop_deadline:
                _stop_chassis()
                print("[SAFE] end_trace_by_time 到期 -> STOP")
                _state = STOPPED
                break

            # ---------- 超声波测量 ----------
            if now >= next_poll:
                next_poll = now + DISTANCE_POLL_INTERVAL
                d = _distance.get_distance()

                if d is not None and DISTANCE_MIN_CM <= d <= DISTANCE_MAX_CM:
                    if debug:
                        print(f"[DIST] {d:.1f} cm (ok={ok_count})")
                    if d <= target_cm:
                        ok_count += 1
                    else:
                        ok_count = 0
                else:
                    ok_count = 0

                if ok_count >= DISTANCE_OK_FRAMES:
                    print(f"[DIST] 距离 {d:.1f} cm <= {target_cm} cm -> STOP")
                    _stop_chassis()
                    _state = STOPPED
                    break

            # ---------- 发速度 ----------
            vx, vy, wz = _step(f_err, r_err, now)
            _send(vx, vy, wz)
            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("\n[DIST] 收到 Ctrl+C")
    finally:
        _shutdown()
        
        
# ============================== 直行到指定距离停 ==============================
import distance as _distance      # 复用 distance.py 里的 get_distance()

DISTANCE_POLL_INTERVAL = 0.05     # 多少秒读一次超声波
DISTANCE_OK_FRAMES = 3            # 连续 N 次距离达标才算数，防抖动
DISTANCE_MIN_CM = 2.0             # 低于这个值认为是噪声，忽略
DISTANCE_MAX_CM = 400.0           # 高于这个值认为超量程，忽略


def go_until_distance(target_cm, speed=300, timeout_s=None, debug=False):
    """不循迹，直行直到超声波距离 <= target_cm 停车。

    target_cm : 目标停止距离（厘米）
    speed     : 前进速度 mm/s
    timeout_s : 最多跑多少秒；None 不限制
    debug     : True 时打印距离
    """
    global _state, _stop_deadline

    _init_hardware()
    try:
        _state = STOPPED            # 明确不进循迹状态机
        print(f"[GO] 直行直到距离 <= {target_cm} cm，speed={speed}")

        start_t = time.monotonic()
        next_poll = start_t
        ok_count = 0

        while True:
            now = time.monotonic()

            # ---------- 超时保护 ----------
            if timeout_s is not None and now - start_t > timeout_s:
                print(f"[GO] 超时 {timeout_s}s -> STOP")
                _stop_chassis()
                break

            # ---------- 超声波测量 ----------
            if now >= next_poll:
                next_poll = now + DISTANCE_POLL_INTERVAL
                d = _distance.get_distance()

                if d is not None and DISTANCE_MIN_CM <= d <= DISTANCE_MAX_CM:
                    if debug:
                        print(f"[GO] {d:.1f} cm (ok={ok_count})")
                    if d <= target_cm:
                        ok_count += 1
                    else:
                        ok_count = 0
                else:
                    ok_count = 0

                if ok_count >= DISTANCE_OK_FRAMES:
                    print(f"[GO] 距离 {d:.1f} cm <= {target_cm} cm -> STOP")
                    _stop_chassis()
                    break

            # ---------- 一直前进 ----------
            _send(speed, 0, 0)
            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("\n[GO] 收到 Ctrl+C")
    finally:
        _shutdown()
        

# ============================== 直接运行时的默认行为 ==============================
if __name__ == "__main__":
    run([
        ("T", "continue"),
        ("R", "continue"),
        ("L", "continue"),
        ("L", "continue"),
    ], speed=300)

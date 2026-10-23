"""双循迹 + IMU 路口转向封装模块。

流程：
    普通 T 形/十字路口：
        FOLLOW → BRAKE → ADVANCE(前进0.4s) → IMU_TURN
               → IMU_SETTLE → ALIGN → SETTLE → FOLLOW
    正T（不转弯）：
        FOLLOW → BRAKE → SETTLE → FOLLOW
    直角弯：
        FOLLOW → BRAKE → ADVANCE(前进0.4s) → IMU_TURN
               → IMU_SETTLE → CORNER_CHECK（前板在线上立即循迹）
               → SETTLE → FOLLOW

转向标识：
    'T' / 'S' / 0 / 1  → 正T，不转弯
    'L' / 2            → 左转 90°
    'R' / 3            → 右转 90°
    'B' / 4            → 掉头 180°
"""

import struct
import time

import distance as _distance      # 复用 distance.py 里的 get_distance()
import serial_ports
import distance_zuo as distance_zuo


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
ADVANCE_TIME = 0.0

# ---- 超声波测距 ----
DISTANCE_POLL_INTERVAL = 0.05
DISTANCE_OK_FRAMES = 3
DISTANCE_MIN_CM = 2.0
DISTANCE_MAX_CM = 400.0

# ---- 后退参数 ----
BACK_SPEED_DEFAULT = 200
BACK_EXTRA_TIME_DEFAULT = 0.4
BACK_TIMEOUT_S = 6.0

# ---- 后退压到拐口后的微调对正 ----
BACK_ALIGN_ENABLE = True          # 总开关
BACK_ALIGN_KW = 45.0              # 对正 P 增益（用后板误差调 wz）
BACK_ALIGN_MAX_WZ = 100           # 对正最大角速度
BACK_ALIGN_ERROR = 1.0            # |r_err| 小于这个值算对正
BACK_ALIGN_CONFIRM_FRAMES = 3     # 连续 N 帧对正才算成功
BACK_ALIGN_TIMEOUT_S = 1.0        # 对正超时

# ============================== 平移找线 + 循迹 ==============================
SHIFT_SPEED_DEFAULT = 200           # 默认平移速度 mm/s
SHIFT_TIMEOUT_S = 15.0               # 最多平移多少秒，防卡死
SHIFT_FIND_FRAMES = 2               # 连续 N 帧找到线才算找到

# ---- 循迹 P 控制 ----
KY = 120.0
KW = 90.0
MAX_VY = 160
MAX_WZ = 450
VY_RIGHT_SIGN = -1
YAW_SIGN = -1

FRONT_BLACK_LEVEL = 1
REAR_BLACK_LEVEL = 1

# ---- 路口检测 ----
INTERSECTION_MIN_ACTIVE = 5
INTERSECTION_CONFIRM_FRAMES = 3
INTERSECTION_BRAKE_TIME = 0.20   #进路口前刹停

# ---- 直角检测 ----
CORNER_SIDE_ACTIVE = 4

# ---- IMU 粗转 ----
IMU_TURN_WZ_ABS = 1200
IMU_TURN_STOP_DEG_ABS = 59.8
IMU_TURN_STOP_DEG_180 = 150.0    # 180° 掉头停止角
IMU_TURN_TIMEOUT_S = 10.0
IMU_SETTLE_TIME = 0.20   #转完等车身晃停

# ---- 精对准 ----
ALIGN_KY = 45.0
ALIGN_KW = 60.0
ALIGN_MAX_VY = 90
ALIGN_MAX_WZ = 180
ALIGN_SEARCH_WZ = 120
ALIGN_INITIAL_WAIT_S = 0.0 #对准前静默
TURN_ALIGN_ERROR = 1.5
TURN_ALIGN_CONFIRM_FRAMES = 3
TURN_ALIGN_TIMEOUT_S = 1.5  
TURN_SETTLE_TIME = 0.0 #对准后静默
INTERSECTION_COOLDOWN_S = 1.5

# ---- 直角弯检查 ----
CORNER_CHECK_TIMEOUT_S = 1.5     # 直角弯后最多搜索多少秒

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
CORNER_CHECK = "CORNER_CHECK"    # 直角弯后检查前板是否在线
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
_is_corner_turn = False          # 当前转弯是否为直角弯
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


def _is_corner_only(black):
    """只算直角弯，不算十字/T 字。"""
    return _is_corner(black) and not _is_cross_or_t(black)


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


def cross_turn(target, after="auto", corner=False):
    """路口动作。

    target : 'T'/'S'/1/0 正T不转弯
             'L'/2 左转 90°
             'R'/3 右转 90°
             'B'/4 掉头 180°
    after  : 'continue' / 'stop' / 数字秒数 / 'auto'
    corner : True 时表示直角弯，转完只要前板在线上就继续循迹
    """
    global _state, _turn_target, _turn_after, _turn_stop_delay
    global _state_t0, _align_count, _cooldown_until, _is_corner_turn

    if target in ("T", "S", "正T", 0, 1):
        _turn_target = "T"
    elif target in ("L", "左T", 2):
        _turn_target = "L"
    elif target in ("R", "右T", 3):
        _turn_target = "R"
    elif target in ("B", "掉头", "back", 4):
        _turn_target = "B"
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

    _is_corner_turn = corner
    _state_t0 = time.monotonic()
    _align_count = 0
    _cooldown_until = time.monotonic() + INTERSECTION_COOLDOWN_S
    _state = BRAKE
    print(f"[API] cross_turn target={_turn_target} after={after} "
          f"corner={corner}")


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

    # ---------------- 转弯前前进 ----------------
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
        elif _turn_target == "R":
            wz_cmd = -IMU_TURN_WZ_ABS
            stop_deg = -IMU_TURN_STOP_DEG_ABS
        elif _turn_target == "B":
            wz_cmd = -IMU_TURN_WZ_ABS
            stop_deg = -IMU_TURN_STOP_DEG_180
        else:
            wz_cmd = 0
            stop_deg = 0

        delta = _imu.yaw - _turn_start_yaw
        if stop_deg > 0:
            reached = delta >= stop_deg
        elif stop_deg < 0:
            reached = delta <= stop_deg
        else:
            reached = True

        if reached or (now - _state_t0) > IMU_TURN_TIMEOUT_S:
            print(f"[TURN] done delta={delta:+.2f}")
            _state = IMU_SETTLE
            _state_t0 = now
            return 0, 0, 0
        return 0, 0, wz_cmd

    # ---------------- 粗转停稳 ----------------
    if _state == IMU_SETTLE:
        if now - _state_t0 >= IMU_SETTLE_TIME:
            if _is_corner_turn:
                _state = CORNER_CHECK
                print("[CORNER] 进入直角弯检查")
            else:
                _state = ALIGN
            _state_t0 = now
            _align_count = 0
        return 0, 0, 0

    # ---------------- 直角弯检查：前板在线上立即循迹 ----------------
    if _state == CORNER_CHECK:
        # 前板在线上 → 直接继续循迹
        if front_err is not None:
            print("[CORNER] 前板在线 -> 继续循迹")
            _state = SETTLE
            _state_t0 = now
            return 0, 0, 0

        # 前板没在线，原地搜索旋转
        if now - _state_t0 > CORNER_CHECK_TIMEOUT_S:
            print("[CORNER] 搜索超时 -> SETTLE")
            _state = SETTLE
            _state_t0 = now
            return 0, 0, 0

        d = 1 if _turn_target == "L" else -1
        return 0, 0, ALIGN_SEARCH_WZ * d

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
            d = 1 if _turn_target in ("L", "B") else -1
            _align_count = 0
            return 0, 0, ALIGN_SEARCH_WZ * d

        if rear_err is None:
            vy = _clamp(VY_RIGHT_SIGN * ALIGN_KY * front_err, ALIGN_MAX_VY)
            return 0, int(round(vy)), 0

        if front_err is None:
            d = 1 if _turn_target in ("L", "B") else -1
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
def run(route, speed=300, dry_run=None, debug_sensor=False, stop_after=None,
        advance_vx=None, advance_time=None, initial_align=True):
    """按 route 列表跑完所有路口。

    额外参数：
        advance_vx    : 识别到路口后前进速度 mm/s（None=用全局 ADVANCE_VX）
        advance_time  : 识别到路口后前进时长 s（None=用全局 ADVANCE_TIME）
        initial_align : 起步前是否先做初始对齐（默认 True）
                        开启后：
                          - 前后板都在线 → 双板都对齐（横移 + 旋转）
                          - 只有前板在线 → 用前板误差对齐
                          - 只有后板在线 → 用后板误差对齐
                          - 都丢线       → 原地慢转找线
                        对齐成功或超时后才进入 FOLLOW。
    """
    global _route, _route_index, _confirm_count, _state
    global ADVANCE_VX, ADVANCE_TIME

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
        if advance_vx is not None or advance_time is not None:
            print(f"  advance_vx={advance_vx}  advance_time={advance_time}")
        print(f"  initial_align = {initial_align}")
        return

    # ---- 起步前初始对齐参数（仅本函数内使用）----
    INIT_ALIGN_TIMEOUT_S = 3.0          # 对齐超时
    INIT_ALIGN_ERROR = 0.5              # |err| 小于这个值算对齐
    INIT_ALIGN_CONFIRM_FRAMES = 3       # 连续 N 帧对齐才算成功
    INIT_ALIGN_MAX_VY = 60              # 最大横移速度
    INIT_ALIGN_MAX_WZ = 60              # 最大角速度
    INIT_ALIGN_SEARCH_WZ = 60           # 前后板都丢线时的原地搜索转速
    INIT_ALIGN_KY = 30.0                # 横移 P 增益
    INIT_ALIGN_KW = 30.0                # 旋转 P 增益

    saved_advance_vx = ADVANCE_VX
    saved_advance_time = ADVANCE_TIME
    if advance_vx is not None:
        ADVANCE_VX = int(advance_vx)
    if advance_time is not None:
        ADVANCE_TIME = float(advance_time)

    try:
        _init_hardware()

        # ---------------- 起步前初始对齐 ----------------
        if initial_align:
            print(f"[INIT_ALIGN] 起步前对齐：误差阈值 {INIT_ALIGN_ERROR}，"
                  f"超时 {INIT_ALIGN_TIMEOUT_S}s")
            align_t0 = time.monotonic()
            align_count = 0

            while True:
                now = time.monotonic()
                if now - align_t0 > INIT_ALIGN_TIMEOUT_S:
                    print("[INIT_ALIGN] 对齐超时 -> 直接起步")
                    break

                f_black, f_err = _read_board(_front, FRONT_BLACK_LEVEL)
                r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
                _read_imu()

                # 前后板都在线：双板都对齐（横移 + 旋转同时纠偏）
                if f_err is not None and r_err is not None:
                    lat = (f_err + r_err) / 2.0
                    yaw = f_err - r_err
                    if (abs(lat) < INIT_ALIGN_ERROR
                            and abs(yaw) < INIT_ALIGN_ERROR):
                        align_count += 1
                        _send(0, 0, 0)
                        if align_count >= INIT_ALIGN_CONFIRM_FRAMES:
                            print(f"[INIT_ALIGN] 双板对齐 OK "
                                  f"(lat={lat:+.2f} yaw={yaw:+.2f}) -> 起步")
                            break
                    else:
                        align_count = 0
                        vy = _clamp(VY_RIGHT_SIGN * INIT_ALIGN_KY * lat,
                                    INIT_ALIGN_MAX_VY)
                        wz = _clamp(YAW_SIGN * INIT_ALIGN_KW * yaw,
                                    INIT_ALIGN_MAX_WZ)
                        _send(0, int(round(vy)), int(round(wz)))

                # 只有前板在线：前板对齐
                elif f_err is not None:
                    if abs(f_err) < INIT_ALIGN_ERROR:
                        align_count += 1
                        _send(0, 0, 0)
                        if align_count >= INIT_ALIGN_CONFIRM_FRAMES:
                            print(f"[INIT_ALIGN] 前板对齐 OK "
                                  f"(f_err={f_err:+.2f}) -> 起步")
                            break
                    else:
                        align_count = 0
                        vy = _clamp(VY_RIGHT_SIGN * INIT_ALIGN_KY * f_err,
                                    INIT_ALIGN_MAX_VY)
                        _send(0, int(round(vy)), 0)

                # 只有后板在线：后板对齐
                elif r_err is not None:
                    if abs(r_err) < INIT_ALIGN_ERROR:
                        align_count += 1
                        _send(0, 0, 0)
                        if align_count >= INIT_ALIGN_CONFIRM_FRAMES:
                            print(f"[INIT_ALIGN] 后板对齐 OK "
                                  f"(r_err={r_err:+.2f}) -> 起步")
                            break
                    else:
                        align_count = 0
                        vy = _clamp(VY_RIGHT_SIGN * INIT_ALIGN_KY * r_err,
                                    INIT_ALIGN_MAX_VY)
                        _send(0, int(round(vy)), 0)

                # 都丢线：原地慢转找线
                else:
                    align_count = 0
                    _send(0, 0, INIT_ALIGN_SEARCH_WZ)

                time.sleep(CONTROL_PERIOD)

            _send(0, 0, 0)
            time.sleep(0.1)

        # ---------------- 起步进入循迹 ----------------
        start_trace(speed)

        if stop_after is not None:
            end_trace_by_time(stop_after)

        while _state != STOPPED:
            now = time.monotonic()

            f_black, f_err = _read_board(_front, FRONT_BLACK_LEVEL)
            r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
            _read_imu()

            if debug_sensor and sum(f_black) > 0:
                print(f"[SENSOR] F={f_black} count={sum(f_black)} "
                      f"corner={_is_corner(f_black)} "
                      f"cross={_is_cross_or_t(f_black)}")

            if _stop_deadline is not None and now >= _stop_deadline:
                _stop_chassis()
                print("[SAFE] end_trace_by_time 到期 -> STOP")
                _state = STOPPED
                break

            if _state == FOLLOW:
                if _is_intersection(f_black) and now >= _cooldown_until:
                    _confirm_count += 1
                    if _confirm_count >= INTERSECTION_CONFIRM_FRAMES:
                        _confirm_count = 0
                        if _route_index < len(_route):
                            target, after = _route[_route_index]
                            _route_index += 1
                            corner = _is_corner_only(f_black)
                            print(f"[ROUTE] 第 {_route_index} 个路口: "
                                  f"target={target} after={after} "
                                  f"corner={corner} black={f_black}")
                            cross_turn(target, after, corner=corner)
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
        ADVANCE_VX = saved_advance_vx
        ADVANCE_TIME = saved_advance_time
        _shutdown()


# ============================== 后退 + 转向 ==============================
def back_then_turn(target, after="continue",
                   back_speed=BACK_SPEED_DEFAULT,
                   extra_time=BACK_EXTRA_TIME_DEFAULT,
                   timeout=BACK_TIMEOUT_S,
                   debug=False):
    global _state, ADVANCE_TIME

    _init_hardware()
    saved_advance = ADVANCE_TIME

    try:
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

        _send(0, 0, 0)
        time.sleep(0.2)

        ADVANCE_TIME = 0.0
        print(f"[BACK] 开始转向 target={target} after={after}（跳过前进）")
        cross_turn(target, after)

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


# ============================== 后退到拐角再转向 ==============================
def back_until_corner_then_turn(target, after="continue",
                                back_speed=BACK_SPEED_DEFAULT,
                                extra_time=BACK_EXTRA_TIME_DEFAULT,
                                pre_turn_back=0.0,
                                timeout=BACK_TIMEOUT_S,
                                trigger="all",
                                corner_turn=False,
                                line_fallback=False,
                                confirm_frames=2,
                                align_after_press=BACK_ALIGN_ENABLE,
                                align_kw=BACK_ALIGN_KW,
                                align_error=BACK_ALIGN_ERROR,
                                align_confirm_frames=BACK_ALIGN_CONFIRM_FRAMES,
                                align_timeout=BACK_ALIGN_TIMEOUT_S,
                                debug=False):
    """后退到【后板压到拐口】再转向（back_then_turn 的兄弟函数）。

    target : 'L'/'2' 左转90°、'R'/'3' 右转90°、'T'/'S'/'0'/'1' 正T不转、
             'B'/'back'/'4' 掉头180°
    after  : 'continue' / 'stop' / 数字秒数 / 'auto'
    back_speed / extra_time / timeout / debug : 同 back_then_turn
    pre_turn_back : 转向前再单独给一小段反向速度的秒数（叠加在 extra_time 上）

    trigger : 后板出现什么形态才算"压到拐口"
              'all'    后板【7 个探头全部见黑】—— 丁字/十字口（默认）
              'center' 后板【中心 5 个探头全黑】—— 比 all 宽松，推荐
              'corner' 只认直角弯
              'any'    直角弯或十字/T 都认
              'line'   任意一个探头见黑就算（最宽松）
    corner_turn : 传给 cross_turn 的 corner 参数
    line_fallback : 后退超时那一刻，若后板至少压到黑线就按压到处理
    confirm_frames : 形态要连续满足多少帧才算数，防抖（默认 2）

    align_after_press : 压到拐口并退完后，是否用后板误差微调对正再转向
                        对正顺序：extra_time → pre_turn_back → 微调对正 → 转向
    align_kw / align_error / align_confirm_frames / align_timeout :
                        对正参数，见顶部常量
    """
    global _state, ADVANCE_TIME

    _init_hardware()
    saved_advance = ADVANCE_TIME
    trigger = str(trigger).lower()
    if trigger not in ("all", "center", "corner", "any", "line"):
        raise ValueError("trigger 只能是 all / center / corner / any / line")
    confirm_frames = max(1, int(confirm_frames))
    pre_turn_back = max(0.0, float(pre_turn_back or 0.0))
    align_confirm_frames = max(1, int(align_confirm_frames))

    # "压到拐口"的判据
    def _hit(black):
        n = sum(black)
        if trigger == "all":
            return n >= len(black)
        if trigger == "center":
            if len(black) < 5:
                return False
            mid = len(black) // 2
            return sum(black[mid - 2:mid + 3]) >= 5
        if trigger == "corner":
            return _is_corner_only(black)
        if trigger == "any":
            return _is_intersection(black)
        return n > 0

    mode = {"all": "丁字/十字口(后板全部见黑)",
            "center": "丁字/十字口(后板中心5个全黑)",
            "corner": "直角弯",
            "any": "拐角/路口",
            "line": "黑线"}[trigger]

    try:
        # ---------- 阶段 1：后退直到后板压出拐角形态 ----------
        print(f"[BACK-CORNER] 开始后退 vx=-{back_speed}，等后板压到{mode}")
        press_t = None
        press_count = 0
        start_t = time.monotonic()

        while True:
            now = time.monotonic()

            r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
            _read_imu()

            hit = _hit(r_black)

            if debug:
                print(f"[BACK-CORNER] R={r_black} err={r_err} "
                      f"count={sum(r_black)} corner={_is_corner(r_black)} "
                      f"cross={_is_cross_or_t(r_black)} hit={hit} "
                      f"confirm={press_count}/{confirm_frames}")

            # 先认拐角、再看超时
            if press_t is None:
                if hit:
                    press_count += 1
                    if press_count >= confirm_frames:
                        press_t = now
                        print(f"[BACK-CORNER] 后板压到{mode} -> "
                              f"继续后退 {extra_time}s")
                else:
                    press_count = 0

            # 超时只在"还没压到"时生效
            if press_t is None and now - start_t > timeout:
                if line_fallback and r_err is not None:
                    press_t = now
                    print(f"[BACK-CORNER] 超时 {timeout}s，但后板压到黑线 -> "
                          f"按压到处理，继续后退 {extra_time}s")
                else:
                    print(f"[BACK-CORNER] 超时 {timeout}s 仍未压到{mode}，"
                          f"放弃后退，直接转向")
                    break

            if press_t is not None and now - press_t >= extra_time:
                print("[BACK-CORNER] 后退结束")
                break

            _send(-back_speed, 0, 0)
            time.sleep(CONTROL_PERIOD)

        # ---------- 阶段 1.5a：转向前再退一小段 ----------
        if pre_turn_back > 0:
            print(f"[BACK-CORNER] 转向前再后退 {pre_turn_back}s")
            turn_back_end = time.monotonic() + pre_turn_back
            while time.monotonic() < turn_back_end:
                _send(-back_speed, 0, 0)
                time.sleep(CONTROL_PERIOD)

        _send(0, 0, 0)
        time.sleep(0.15)

        # ---------- 阶段 1.5b：微调对正（用后板误差调 wz）----------
        if align_after_press:
            print(f"[BACK-CORNER] 微调对正 "
                  f"(kw={align_kw} err<{align_error} "
                  f"确认{align_confirm_frames}帧 超时{align_timeout}s)")
            align_t0 = time.monotonic()
            align_cnt = 0
            while True:
                now = time.monotonic()

                if now - align_t0 > align_timeout:
                    print("[BACK-CORNER] 对正超时 -> 直接转向")
                    break

                r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
                _read_imu()

                if debug:
                    print(f"[BACK-CORNER][ALIGN] R={r_black} err={r_err} "
                          f"ok={align_cnt}/{align_confirm_frames}")

                if r_err is None:
                    # 后板完全看不到线，停止修正（已经偏出线外）
                    _send(0, 0, 0)
                    align_cnt = 0
                elif abs(r_err) < align_error:
                    align_cnt += 1
                    _send(0, 0, 0)
                    if align_cnt >= align_confirm_frames:
                        print(f"[BACK-CORNER] 对正 OK err={r_err:+.2f}")
                        break
                else:
                    align_cnt = 0
                    wz = _clamp(YAW_SIGN * align_kw * r_err, BACK_ALIGN_MAX_WZ)
                    _send(0, 0, int(round(wz)))

                time.sleep(CONTROL_PERIOD)

            _send(0, 0, 0)
            time.sleep(0.1)

        # ---------- 阶段 2：转向（跳过 ADVANCE 前进）----------
        ADVANCE_TIME = 0.0
        print(f"[BACK-CORNER] 开始转向 target={target} after={after} "
              f"corner={corner_turn}（跳过前进）")
        cross_turn(target, after, corner=corner_turn)

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
        print("\n[BACK-CORNER] 收到 Ctrl+C")

    finally:
        ADVANCE_TIME = saved_advance
        _shutdown()


# ============================== 循迹到指定距离停 ==============================
def run_until_distance(target_cm, speed=300, timeout_s=None, debug=False,
                       initial_align=True):
    """循迹到距离 <= target_cm 停。

    initial_align : 起步前是否先做初始对齐（默认 True）
                    开启后：
                      - 前后板都在线 → 双板都对齐（横移 + 旋转）
                      - 只有前板在线 → 用前板误差对齐
                      - 只有后板在线 → 用后板误差对齐
                      - 都丢线       → 原地慢转找线
                    对齐成功或超时后才进入 FOLLOW。
    """
    global _state, _stop_deadline

    # ---- 起步前初始对齐参数（仅本函数内使用）----
    INIT_ALIGN_TIMEOUT_S = 3.0          # 对齐超时
    INIT_ALIGN_ERROR = 0.5              # |err| 小于这个值算对齐
    INIT_ALIGN_CONFIRM_FRAMES = 3       # 连续 N 帧对齐才算成功
    INIT_ALIGN_MAX_VY = 150              # 最大横移速度
    INIT_ALIGN_MAX_WZ = 90              # 最大角速度
    INIT_ALIGN_SEARCH_WZ = 60           # 前后板都丢线时的原地搜索转速
    INIT_ALIGN_KY = 60.0                # 横移 P 增益
    INIT_ALIGN_KW = 30.0                # 旋转 P 增益

    _init_hardware()
    try:
        # ---------------- 起步前初始对齐 ----------------
        if initial_align:
            print(f"[INIT_ALIGN] 起步前对齐：误差阈值 {INIT_ALIGN_ERROR}，"
                  f"超时 {INIT_ALIGN_TIMEOUT_S}s")
            align_t0 = time.monotonic()
            align_count = 0

            while True:
                now = time.monotonic()
                if now - align_t0 > INIT_ALIGN_TIMEOUT_S:
                    print("[INIT_ALIGN] 对齐超时 -> 直接起步")
                    break

                f_black, f_err = _read_board(_front, FRONT_BLACK_LEVEL)
                r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
                _read_imu()

                # 前后板都在线：双板都对齐（横移 + 旋转同时纠偏）
                if f_err is not None and r_err is not None:
                    lat = (f_err + r_err) / 2.0
                    yaw = f_err - r_err
                    if (abs(lat) < INIT_ALIGN_ERROR
                            and abs(yaw) < INIT_ALIGN_ERROR):
                        align_count += 1
                        _send(0, 0, 0)
                        if align_count >= INIT_ALIGN_CONFIRM_FRAMES:
                            print(f"[INIT_ALIGN] 双板对齐 OK "
                                  f"(lat={lat:+.2f} yaw={yaw:+.2f}) -> 起步")
                            break
                    else:
                        align_count = 0
                        vy = _clamp(VY_RIGHT_SIGN * INIT_ALIGN_KY * lat,
                                    INIT_ALIGN_MAX_VY)
                        wz = _clamp(YAW_SIGN * INIT_ALIGN_KW * yaw,
                                    INIT_ALIGN_MAX_WZ)
                        _send(0, int(round(vy)), int(round(wz)))

                # 只有前板在线：前板对齐
                elif f_err is not None:
                    if abs(f_err) < INIT_ALIGN_ERROR:
                        align_count += 1
                        _send(0, 0, 0)
                        if align_count >= INIT_ALIGN_CONFIRM_FRAMES:
                            print(f"[INIT_ALIGN] 前板对齐 OK "
                                  f"(f_err={f_err:+.2f}) -> 起步")
                            break
                    else:
                        align_count = 0
                        vy = _clamp(VY_RIGHT_SIGN * INIT_ALIGN_KY * f_err,
                                    INIT_ALIGN_MAX_VY)
                        _send(0, int(round(vy)), 0)

                # 只有后板在线：后板对齐
                elif r_err is not None:
                    if abs(r_err) < INIT_ALIGN_ERROR:
                        align_count += 1
                        _send(0, 0, 0)
                        if align_count >= INIT_ALIGN_CONFIRM_FRAMES:
                            print(f"[INIT_ALIGN] 后板对齐 OK "
                                  f"(r_err={r_err:+.2f}) -> 起步")
                            break
                    else:
                        align_count = 0
                        vy = _clamp(VY_RIGHT_SIGN * INIT_ALIGN_KY * r_err,
                                    INIT_ALIGN_MAX_VY)
                        _send(0, int(round(vy)), 0)

                # 都丢线：原地慢转找线
                else:
                    align_count = 0
                    _send(0, 0, INIT_ALIGN_SEARCH_WZ)

                time.sleep(CONTROL_PERIOD)

            _send(0, 0, 0)
            time.sleep(0.1)

        # ---------------- 起步进入循迹 ----------------
        start_trace(speed)
        print(f"[DIST] 循迹直到距离 <= {target_cm} cm，speed={speed}")

        start_t = time.monotonic()
        next_poll = start_t
        ok_count = 0

        while _state != STOPPED:
            now = time.monotonic()

            f_black, f_err = _read_board(_front, FRONT_BLACK_LEVEL)
            r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
            _read_imu()

            if timeout_s is not None and now - start_t > timeout_s:
                print(f"[DIST] 超时 {timeout_s}s -> STOP")
                _stop_chassis()
                _state = STOPPED
                break

            if _stop_deadline is not None and now >= _stop_deadline:
                _stop_chassis()
                print("[SAFE] end_trace_by_time 到期 -> STOP")
                _state = STOPPED
                break

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

            vx, vy, wz = _step(f_err, r_err, now)
            _send(vx, vy, wz)
            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("\n[DIST] 收到 Ctrl+C")
    finally:
        _shutdown()


# ============================== 直行到指定距离停 ==============================
def go_until_distance(target_cm, speed=300, timeout_s=None, debug=False):
    global _state, _stop_deadline

    _init_hardware()
    try:
        _state = STOPPED
        print(f"[GO] 直行直到距离 <= {target_cm} cm，speed={speed}")

        start_t = time.monotonic()
        next_poll = start_t
        ok_count = 0

        while True:
            now = time.monotonic()

            if timeout_s is not None and now - start_t > timeout_s:
                print(f"[GO] 超时 {timeout_s}s -> STOP")
                _stop_chassis()
                break

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

            _send(speed, 0, 0)
            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("\n[GO] 收到 Ctrl+C")
    finally:
        _shutdown()


# ============================== 定时循迹入口 ==============================
def run_for(seconds, speed=300):
    run([], speed=speed, stop_after=seconds)


# ============================== 特殊动作：前进 + 右移 + 转弯 ==============================
def car_run_special(route, speed=300,
                    forward_speed=300, forward_time=0.5,
                    right_speed=200, right_time=0.5,
                    dry_run=None, debug=False):
    """循迹到路口后，先前进 forward_time 秒，再右移 right_time 秒，然后转弯/停止。

    route         : [(target, after), ...]
                    target: 'T' 正T不转 / 'L' 左转 / 'R' 右转 / 'B' 掉头
                    after : 'continue' / 'stop' / 数字秒数 / 'auto'
    speed         : 循迹速度 mm/s
    forward_speed : 前进速度 mm/s（正数 = 向前）
    forward_time  : 前进时长（秒）
    right_speed   : 右移速度 mm/s（正数 = 向右）
    right_time    : 右移时长（秒）
    dry_run       : True 只打印不打开硬件
    debug         : True 打印传感器状态
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
            print(f"  路口 {i}: target={t} after={a} | "
                  f"前进 {forward_speed}*{forward_time}s "
                  f"右移 {right_speed}*{right_time}s")
        return

    _init_hardware()
    try:
        start_trace(speed)
        print(f"[SPECIAL] 循迹速度={speed} | "
              f"前进={forward_speed}×{forward_time}s | "
              f"右移={right_speed}×{right_time}s")

        # 特殊阶段状态
        phase = None            # None / 'forward' / 'right'
        phase_t0 = 0.0
        pending_target = None
        pending_after = None
        pending_corner = False

        while True:
            now = time.monotonic()

            # ---------- 定时停车保护 ----------
            if _stop_deadline is not None and now >= _stop_deadline:
                _stop_chassis()
                print("[SAFE] 到期 -> STOP")
                _state = STOPPED
                break

            # ---------- 阶段 1：前进 ----------
            if phase == 'forward':
                if now - phase_t0 >= forward_time:
                    print(f"[SPECIAL] 前进结束 -> 右移 {right_time}s")
                    phase = 'right'
                    phase_t0 = now
                    _send(0, 0, 0)
                else:
                    _send(forward_speed, 0, 0)
                time.sleep(CONTROL_PERIOD)
                continue

            # ---------- 阶段 2：右移 ----------
            if phase == 'right':
                if now - phase_t0 >= right_time:
                    print(f"[SPECIAL] 右移结束 -> 转弯 target={pending_target}")
                    phase = None
                    _send(0, 0, 0)
                    time.sleep(0.15)
                    cross_turn(pending_target, pending_after,
                               corner=pending_corner)
                else:
                    # 向右：vy = VY_RIGHT_SIGN * 正值
                    vy = VY_RIGHT_SIGN * abs(right_speed)
                    _send(0, vy, 0)
                time.sleep(CONTROL_PERIOD)
                continue

            # ---------- 正常循迹 ----------
            f_black, f_err = _read_board(_front, FRONT_BLACK_LEVEL)
            r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
            _read_imu()

            if debug and sum(f_black) > 0:
                print(f"[SENSOR] F={f_black} count={sum(f_black)} "
                      f"corner={_is_corner(f_black)}")

            # 状态机已经结束，退出
            if _state == STOPPED:
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
                            corner = _is_corner_only(f_black)

                            # 记住待执行动作
                            pending_target = target
                            pending_after = after
                            pending_corner = corner

                            # 进入特殊阶段
                            phase = 'forward'
                            phase_t0 = now
                            print(f"[SPECIAL] 路口 {_route_index}: "
                                  f"target={target} after={after} "
                                  f"corner={corner}")
                            print(f"[SPECIAL] 先进 {forward_time}s @ "
                                  f"{forward_speed}，再右移 {right_time}s @ "
                                  f"{right_speed}")
                            _send(0, 0, 0)
                            time.sleep(CONTROL_PERIOD)
                            continue
                else:
                    _confirm_count = 0

            # 正常状态机输出
            vx, vy, wz = _step(f_err, r_err, now)
            _send(vx, vy, wz)
            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("\n[SPECIAL] 收到 Ctrl+C")
    except Exception as e:
        print(f"\n[SPECIAL] 程序异常: {e}")
    finally:
        _shutdown()


# ============================== 平移找线 + 循迹 ==============================
SHIFT_SPEED_DEFAULT = 300           # 默认平移速度 mm/s
SHIFT_TIMEOUT_S = 15.0               # 最多平移多少秒，防卡死
SHIFT_FIND_FRAMES = 2               # 连续 N 帧满足条件才算找到

# 右移检测右侧三个，左移检测左侧三个
SHIFT_RIGHT_IDX = (4, 5, 6)
SHIFT_LEFT_IDX = (1, 2, 3)


def _side_on_line(black, idx):
    """指定索引的三个传感器全部在线才返回 True。"""
    return all(black[i] == 1 for i in idx)


def shift_until_line(direction,
                     shift_speed=SHIFT_SPEED_DEFAULT,
                     after="stop",
                     search_time=None,
                     timeout=SHIFT_TIMEOUT_S,
                     find_frames=SHIFT_FIND_FRAMES,
                     use_rear=False,
                     debug=False):
    """横移找线：先往一个方向走一段，找不到就换反方向并一直走下去。

    方向与传感器对应：
        往右移 → 检测右侧三个传感器 (4, 5, 6) 全部在线
        往左移 → 检测左侧三个传感器 (1, 2, 3) 全部在线

    direction   : 'L' 初始往左 / 'R' 初始往右
    search_time : 初始方向的搜索时长（秒）
                  - 数值：先往 direction 走 search_time 秒；
                    没找到 → 换到反方向，然后【一直】往这个方向找。
                  - None（默认）：一直往 direction 方向走，不换向。
    shift_speed : 横移速度 mm/s（正数）
    after       : 找到线之后：'stop' / 'continue' / 数字（秒）
    timeout     : 总超时秒数；None = 不限时（默认 8s）
    find_frames : 连续 N 帧满足条件才算找到（防抖）
    use_rear    : True 用后板传感器，False（默认）用前板
    debug       : True 打印每帧传感器状态
    """
    global _state, _stop_deadline

    # ---------- 参数检查 ----------
    if direction not in ("L", "R", "l", "r"):
        raise ValueError(f"direction 只能是 'L' 或 'R'：{direction}")
    direction = direction.upper()

    if search_time is not None:
        search_time = float(search_time)
        if search_time <= 0:
            raise ValueError("search_time 必须 > 0 或 None")

    # ---------- 辅助（延迟取 board）----------
    def _idx_of(d):
        return SHIFT_RIGHT_IDX if d == "R" else SHIFT_LEFT_IDX

    def _vy_of(d):
        if d == "R":
            return VY_RIGHT_SIGN * abs(shift_speed)
        return -VY_RIGHT_SIGN * abs(shift_speed)

    def _board():
        return _rear if use_rear else _front

    def _level():
        return REAR_BLACK_LEVEL if use_rear else FRONT_BLACK_LEVEL

    _init_hardware()
    try:
        # ---------- 阶段 0：起始检查（已在线上就不动）----------
        board_black, _ = _read_board(_board(), _level())
        _read_imu()

        if _side_on_line(board_black, _idx_of(direction)):
            print(f"[SHIFT] 起始就在 {direction} 侧线上 -> 不移动")
            _send(0, 0, 0)
            time.sleep(0.15)
            line_found = True
        else:
            cur_dir = direction
            switched = False
            start_t = time.monotonic()
            seg_start = start_t
            found_count = 0
            line_found = False

            timeout_str = (f"，总超时 {timeout}s"
                           if timeout is not None else "（不限时）")
            if search_time is None:
                print(f"[SHIFT] 一直往 {cur_dir} 走找线{timeout_str}")
            else:
                print(f"[SHIFT] 先往 {cur_dir} 走 {search_time}s，"
                      f"找不到就换反方向一直走{timeout_str}")

            while True:
                now = time.monotonic()

                # 1) 读传感器 + 判断当前方向是否找到线
                board_black, _ = _read_board(_board(), _level())
                _read_imu()

                idx = _idx_of(cur_dir)
                side_ok = _side_on_line(board_black, idx)

                if debug:
                    marks = "".join(str(board_black[i]) for i in idx)
                    print(f"[SHIFT] dir={cur_dir} board={board_black} "
                          f"side={marks} ok={side_ok} cnt={found_count} "
                          f"switched={switched}")

                if side_ok:
                    # 已经压到线：先发零速度确认
                    found_count += 1
                    _send(0, 0, 0)
                    if found_count >= find_frames:
                        print(f"[SHIFT] 往 {cur_dir} 找到线 -> 停")
                        line_found = True
                        break
                else:
                    found_count = 0
                    _send(0, _vy_of(cur_dir), 0)

                # 2) 总超时检查
                if timeout is not None and now - start_t > timeout:
                    print(f"[SHIFT] 总超时 {timeout}s 未找到线 -> 停")
                    _send(0, 0, 0)
                    _stop_chassis()
                    _state = STOPPED
                    break

                # 3) 单向时长到了 → 换反方向（只换这一次）
                if (search_time is not None
                        and not switched
                        and now - seg_start >= search_time):
                    cur_dir = "R" if cur_dir == "L" else "L"
                    seg_start = now
                    found_count = 0
                    switched = True
                    print(f"[SHIFT] {search_time}s 没找到 -> "
                          f"换向，一直往 {cur_dir} 走")

                time.sleep(CONTROL_PERIOD)

            _send(0, 0, 0)
            time.sleep(0.15)

        if not line_found:
            return

        # ---------- 阶段 2：根据 after 决定 ----------
        if after == "stop":
            print("[SHIFT] after=stop -> 立即停车")
            _send(0, 0, 0)
            _stop_chassis()
            _state = STOPPED
            return

        if after == "continue":
            print("[SHIFT] after=continue -> 进入循迹")
            start_trace(_follow_speed)
            while _state != STOPPED:
                now = time.monotonic()
                f_black, f_err = _read_board(_front, FRONT_BLACK_LEVEL)
                r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
                _read_imu()

                if _stop_deadline is not None and now >= _stop_deadline:
                    _stop_chassis()
                    _state = STOPPED
                    break

                vx, vy, wz = _step(f_err, r_err, now)
                _send(vx, vy, wz)
                time.sleep(CONTROL_PERIOD)
            return

        try:
            seconds = float(after)
        except (TypeError, ValueError):
            raise ValueError(f"after 只能是 'stop' / 'continue' / 数字，"
                             f"当前是 {after}")

        print(f"[SHIFT] after={seconds}s -> 循迹 {seconds}s 后停")
        start_trace(_follow_speed)
        end_trace_by_time(seconds)

        while _state != STOPPED:
            now = time.monotonic()
            f_black, f_err = _read_board(_front, FRONT_BLACK_LEVEL)
            r_black, r_err = _read_board(_rear, REAR_BLACK_LEVEL)
            _read_imu()

            if _stop_deadline is not None and now >= _stop_deadline:
                _stop_chassis()
                print("[SAFE] 循迹时间到 -> STOP")
                _state = STOPPED
                break

            vx, vy, wz = _step(f_err, r_err, now)
            _send(vx, vy, wz)
            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("\n[SHIFT] 收到 Ctrl+C")
    finally:
        _shutdown()
    
    
# ---- 平移找线 ----
SHIFT_SPEED_DEFAULT = 300              # 横移速度 mm/s
SHIFT_TIMEOUT_S = 8.0                  # 总超时秒数（找不到线就放弃）
SHIFT_FIND_FRAMES = 2                  # 连续 N 帧找到线才算找到

# ---- 距离分档边界（cm）----
SHIFT_DIST_NEAR = 50        # < 50       → 直接向右
SHIFT_DIST_MID_LOW = 68     # 50 ~ 68    → 先向右 search_time，找不到再向左
SHIFT_DIST_MID_HIGH = 90    # 68 ~ 90    → 先向左 search_time，找不到再向右
                            # ≥ 90       → 直接向左

# ---- 距离采样与换向 ----
SHIFT_DIST_SAMPLES_DEFAULT = 5         # 测距采样次数
SHIFT_DIST_INTERVAL_DEFAULT = 0.05     # 采样间隔秒
SHIFT_DIST_SEARCH_TIME_DEFAULT = 1.5   # 中间两档单次尝试时长（秒）

# ============================== 距离分档横移找线 ==============================
def _read_distances(n, interval):
    """连续读 n 次测距，返回有效读数列表（cm）。"""
    vals = []
    for i in range(n):
        d = distance_zuo.get_distance()
        if d is not None and DISTANCE_MIN_CM <= d <= DISTANCE_MAX_CM:
            vals.append(d)
        if i < n - 1:
            time.sleep(interval)
    return vals


def _shift_with_fallback(primary, fallback, primary_time=1.5,
                         shift_speed=SHIFT_SPEED_DEFAULT,
                         use_rear=False, timeout=SHIFT_TIMEOUT_S,
                         find_frames=SHIFT_FIND_FRAMES,
                         debug=False):
    """先往 primary 方向横移 primary_time 秒；找不到线就换 fallback 方向，
    一直往 fallback 方向找到线或总超时。

    返回找到线时的方向 'R'/'L'，找不到返回 None。
    不动 _state，只发速度帧，可以安全嵌在别的流程里。
    """
    board = _rear if use_rear else _front
    black_level = REAR_BLACK_LEVEL if use_rear else FRONT_BLACK_LEVEL

    def _idx_of(d):
        return SHIFT_RIGHT_IDX if d == "R" else SHIFT_LEFT_IDX

    def _vy_of(d):
        if d == "R":
            return VY_RIGHT_SIGN * abs(shift_speed)
        return -VY_RIGHT_SIGN * abs(shift_speed)

    start_t = time.monotonic()
    phase_start = start_t
    cur_dir = primary
    switched = False
    found_count = 0

    while True:
        now = time.monotonic()
        board_black, _ = _read_board(board, black_level)
        _read_imu()

        side_ok = _side_on_line(board_black, _idx_of(cur_dir))
        if debug:
            marks = "".join(str(board_black[i]) for i in _idx_of(cur_dir))
            print(f"[SHIFT-DIST] dir={cur_dir} board={board_black} "
                  f"side={marks} found={found_count} ok={side_ok}")

        if side_ok:
            found_count += 1
            _send(0, 0, 0)
            if found_count >= find_frames:
                print(f"[SHIFT-DIST] 往 {cur_dir} 找到线")
                return cur_dir
        else:
            found_count = 0
            _send(0, _vy_of(cur_dir), 0)

        # 主方向时长到了 → 换向
        if not switched and now - phase_start >= primary_time:
            cur_dir = fallback
            switched = True
            phase_start = now
            found_count = 0
            print(f"[SHIFT-DIST] 主方向 {primary_time}s 未找到 → 换向 {fallback}")

        # 总超时
        if timeout is not None and now - start_t > timeout:
            print(f"[SHIFT-DIST] 总超时 {timeout}s 未找到线 → 停")
            _send(0, 0, 0)
            return None

        time.sleep(CONTROL_PERIOD)


def shift_by_distance_zuo(distance_zuo=None,
                          target_cm=None,      # 已弃用（保留只为兼容旧调用）
                          samples=SHIFT_DIST_SAMPLES_DEFAULT,
                          interval=SHIFT_DIST_INTERVAL_DEFAULT,
                          use_rear=False,
                          search_time=SHIFT_DIST_SEARCH_TIME_DEFAULT,
                          timeout=SHIFT_TIMEOUT_S,
                          debug=False,
                          **shift_kwargs):
    """多次读 distance_zuo，按距离分档决定横移方向找线。

    前置检查（优先）：
        循迹模块已经在线上（任一探头见黑）→ 不动作，不测距、不横移。

    分档规则（取多次读数的平均值）：
        d < SHIFT_DIST_NEAR                          → 向右找线
        SHIFT_DIST_NEAR ≤ d < SHIFT_DIST_MID_LOW     → 先向右 search_time 秒，
                                                       找不到再向左
        SHIFT_DIST_MID_LOW ≤ d < SHIFT_DIST_MID_HIGH → 先向左 search_time 秒，
                                                       找不到再向右
        d ≥ SHIFT_DIST_MID_HIGH                      → 向左找线

    参数：
        distance_zuo : 可选。单个数 / 列表 / None（自动采样 samples 次）
        target_cm    : 已弃用，忽略
        samples      : 采样次数
        interval     : 采样间隔秒
        use_rear     : True 用后板，False（默认）用前板
        search_time  : 中间两档单次尝试时长（秒），默认 1.5
        timeout      : 总超时秒数
        debug        : True 打印过程
        **shift_kwargs : 透传给 shift_until_line（after 等）
    """
    # ---------- 0. 前置：循迹模块已经在线上就不动 ----------
    already_inited = _serial is not None
    if not already_inited:
        _init_hardware()
    try:
        board = _rear if use_rear else _front
        black_level = REAR_BLACK_LEVEL if use_rear else FRONT_BLACK_LEVEL
        black, err = _read_board(board, black_level)
    finally:
        if not already_inited:
            _shutdown()

    if err is not None:
        print(f"[SHIFT-DIST] 循迹模块已在线上 (black={black}) → 不动作")
        return

    # ---------- 1. 取一组读数 ----------
    if distance_zuo is None:
        vals = _read_distances(int(samples), float(interval))
    elif isinstance(distance_zuo, (list, tuple)):
        vals = [float(v) for v in distance_zuo if v is not None]
    else:
        vals = [float(distance_zuo)]

    if not vals:
        print("[SHIFT-DIST] 没有有效测距读数 → 不动作")
        return

    d = sum(vals) / len(vals)   # 用平均值判决，避免单个噪声带偏
    if debug:
        print(f"[SHIFT-DIST] 读数 {len(vals)} 个: "
              f"{[round(v, 1) for v in vals]}，平均 {d:.1f}")

    # ---------- 2. 分档判决 ----------
    if d < SHIFT_DIST_NEAR:
        print(f"[SHIFT-DIST] {d:.1f} < {SHIFT_DIST_NEAR} → 向右")
        shift_until_line("R", use_rear=use_rear, **shift_kwargs)

    elif d < SHIFT_DIST_MID_LOW:
        print(f"[SHIFT-DIST] {SHIFT_DIST_NEAR} ≤ {d:.1f} < "
              f"{SHIFT_DIST_MID_LOW} → 先向右 {search_time}s，找不到再向左")
        found = _shift_with_fallback(
            primary="R", fallback="L", primary_time=search_time,
            use_rear=use_rear, timeout=timeout, debug=debug,
        )
        if found is not None:
            shift_until_line(found, use_rear=use_rear, **shift_kwargs)

    elif d < SHIFT_DIST_MID_HIGH:
        print(f"[SHIFT-DIST] {SHIFT_DIST_MID_LOW} ≤ {d:.1f} < "
              f"{SHIFT_DIST_MID_HIGH} → 先向左 {search_time}s，找不到再向右")
        found = _shift_with_fallback(
            primary="L", fallback="R", primary_time=search_time,
            use_rear=use_rear, timeout=timeout, debug=debug,
        )
        if found is not None:
            shift_until_line(found, use_rear=use_rear, **shift_kwargs)

    else:
        print(f"[SHIFT-DIST] {d:.1f} ≥ {SHIFT_DIST_MID_HIGH} → 向左")
        shift_until_line("L", use_rear=use_rear, **shift_kwargs)

# ============================== 左移到 50cm 后向右找线 ==============================
SHIFT_LEFT_DIST_DEFAULT = 40.0          # 目标距离（cm）
SHIFT_LEFT_POLL_INTERVAL = 0.05         # 测距轮询间隔（秒）
SHIFT_LEFT_CONFIRM_FRAMES = 5           # 连续 N 帧距离达标才算到
SHIFT_LEFT_LINE_CONFIRM_FRAMES = 2      # 连续 N 帧压到线才算找到（防抖）


def shift_left_until_distance_then_right(target_cm=SHIFT_LEFT_DIST_DEFAULT,
                                         shift_speed=SHIFT_SPEED_DEFAULT,
                                         timeout=SHIFT_TIMEOUT_S,
                                         poll_interval=SHIFT_LEFT_POLL_INTERVAL,
                                         confirm_frames=SHIFT_LEFT_CONFIRM_FRAMES,
                                         line_confirm_frames=SHIFT_LEFT_LINE_CONFIRM_FRAMES,
                                         use_rear=False,
                                         debug=False,
                                         **shift_kwargs):
    """先向左横移，直到测距 <= target_cm，然后向右找线。

    前置检查（优先）：
        循迹模块已经在线上（任一探头见黑）→ 不动作。

    左移途中：
        只要在到达目标距离之前压到黑线 → 立即停车并返回，
        不再执行“向右找线”。

    参数：
        target_cm      : 目标距离（cm），默认 40
        shift_speed    : 左移速度 mm/s，默认 SHIFT_SPEED_DEFAULT
        timeout        : 总超时秒数，默认 SHIFT_TIMEOUT_S
        poll_interval  : 测距轮询间隔秒
        confirm_frames : 连续 N 帧距离 <= target_cm 才算到达（防抖）
        line_confirm_frames : 连续 N 帧压到线才算“找到线”（防抖）
        use_rear       : True 用后板找线，False（默认）用前板
        debug          : True 打印每帧过程
        **shift_kwargs : 透传给 shift_until_line（after 等）
    """
    # ---------- 0. 前置：循迹模块已经在线上就不动 ----------
    already_inited = _serial is not None
    if not already_inited:
        _init_hardware()

    line_found = False
    try:
        board = _rear if use_rear else _front
        black_level = REAR_BLACK_LEVEL if use_rear else FRONT_BLACK_LEVEL
        black, err = _read_board(board, black_level)

        if err is not None:
            print(f"[LEFT-RIGHT] 循迹模块已在线上 (black={black}) → 不动作")
            return

        # ---------- 1. 向左横移：距离达标或压到线都停 ----------
        vy_left = -VY_RIGHT_SIGN * abs(shift_speed)   # 左移
        start_t = time.monotonic()
        next_poll = start_t
        ok_count = 0
        line_count = 0
        d = None

        print(f"[LEFT-RIGHT] 向左横移，直到距离 <= {target_cm} cm"
              f"（途中压到线也停）")

        while True:
            now = time.monotonic()

            if now - start_t > timeout:
                print(f"[LEFT-RIGHT] 超时 {timeout}s 未达到 "
                      f"{target_cm} cm → 停")
                _send(0, 0, 0)
                return

            # ---- 1a. 每帧都查一次循迹板：压到线就立刻停 ----
            black, err = _read_board(board, black_level)
            if err is not None:
                line_count += 1
                if line_count >= line_confirm_frames:
                    print(f"[LEFT-RIGHT] 左移途中压到线 (black={black}) → "
                          f"停止左移，不再向右找线")
                    _send(0, 0, 0)
                    time.sleep(0.1)
                    line_found = True
                    return
            else:
                line_count = 0

            # ---- 1b. 按间隔轮询测距 ----
            if now >= next_poll:
                next_poll = now + poll_interval
                d = distance_zuo.get_distance()

                if d is not None and DISTANCE_MIN_CM <= d <= DISTANCE_MAX_CM:
                    if debug:
                        print(f"[LEFT-RIGHT] d={d:.1f} cm (ok={ok_count})")
                    if d <= target_cm:
                        ok_count += 1
                    else:
                        ok_count = 0
                else:
                    ok_count = 0

                if ok_count >= confirm_frames:
                    print(f"[LEFT-RIGHT] 距离 {d:.1f} cm <= {target_cm} cm → "
                          f"停止左移，改为向右找线")
                    _send(0, 0, 0)
                    time.sleep(0.1)
                    break

            _send(0, vy_left, 0)
            time.sleep(CONTROL_PERIOD)
    finally:
        if not already_inited:
            _shutdown()

    # ---------- 2. 只有“因距离达标”才继续向右找线 ----------
    # 若 line_found 为 True 上面已 return，不会走到这里
    shift_until_line("R", use_rear=use_rear, **shift_kwargs)


# ------------------ 直接走 ---------------------------

def move_for(seconds, vx=300, vy=0, wz=0, debug=False):
    """不循迹，按给定速度直行/后退/横移固定时间。

    vx : 前进速度 mm/s，正=前进，负=后退
    vy : 横移速度 mm/s
    wz : 旋转速度 rad/s*1000
    seconds : 持续时间
    """
    _init_hardware()
    try:
        print(f"[MOVE] vx={vx} vy={vy} wz={wz} for {seconds}s")
        start_t = time.monotonic()
        while time.monotonic() - start_t < seconds:
            _send(vx, vy, wz)
            time.sleep(CONTROL_PERIOD)
        _stop_chassis()
    except KeyboardInterrupt:
        print("\n[MOVE] 收到 Ctrl+C")
    finally:
        _shutdown()


def move_distance(distance_mm, vx=300, vy=0, wz=0, debug=False):
    """不循迹，按速度估算固定距离。

    距离 = |vx| × 时间 → 时间 = distance_mm / |vx|
    注意：速度不一定准，实际距离会有误差。
    """
    if vx == 0 and vy == 0:
        raise ValueError("vx 和 vy 不能同时为 0")

    speed = max(abs(vx), abs(vy))
    seconds = abs(distance_mm) / speed
    print(f"[MOVE] 目标距离 {distance_mm}mm，速度 {speed}mm/s → {seconds:.2f}s")
    move_for(seconds, vx, vy, wz, debug)


# ============================== 直接运行时的默认行为 ==============================
if __name__ == "__main__":
    #shift_until_line('R', shift_speed=300, search_time=2.5, after='stop')
    move_distance(510,-600)
    move_for(1,0,0,-1540)
    #move_for(1,0,0,-1540)
    #shift_until_line('L', search_time=2, after='stop', debug=True)
    #move_distance(180,-500)
    #move_for(1,0,0,-1540)
    #go_until_distance(15,300)
    #shift_left_until_distance_then_right()
    #shift_by_distance_zuo()
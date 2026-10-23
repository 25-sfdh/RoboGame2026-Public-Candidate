#!/usr/bin/env python3
"""Move the chassis to the verified color-block pickup position.

This module contains no camera or color-detection code.  It consumes the
existing vision_runtime.py JSON API and preserves the control decisions from
trace_car/vision_visual_position_stop_test_PI_STABLE.py.
"""

import json
import math
import serial_ports
import struct
import time
from dataclasses import dataclass
from distance import get_distance
from typing import Any, Dict, List, Optional, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


BLOCK_COLOR1 = "orange"   # 要夹的方块颜色："orange" / "purple"
BLOCK_COLOR2 = "purple"   # 要夹的方块颜色："orange" / "purple"

# Raspberry Pi values from vision_runtime_config_PI_STABLE.json.  A real run
# refreshes these five values from /api/config before allowing motion; a config
# API failure stops the action and never falls back to these defaults.
TARGET_X = 333
ALIGN_DEADBAND = 50   # 跟服务端 vision_runtime_config.json 保持一致；实跑以服务端为准
ALIGN_VY_FAST = 300
ALIGN_VY_SLOW = 35
ALIGN_SLOW_THRESHOLD = 90   # 必须 > ALIGN_DEADBAND，否则 35 那档会被挤没、只剩快修

# Verified values from vision_visual_position_stop_test_PI_STABLE.py.
TARGET_BOTTOM_Y = 397
BOTTOM_Y_TOLERANCE = 15
BOTTOM_Y_SLOW_ZONE = 45
APPROACH_VX_FAST = 480
APPROACH_VX_SLOW = 160
APPROACH_CORRECTION_VY = 35  # 别低于底盘横向有效下限：15 实测推不动车（err*d 不变）

# 深度改用测距模块（distance.get_distance()）后的厘米口径阈值。
# 只有给 VisualBlockPositioner 传了 depth_cm_fn 才走这一套；不传则下面那套
# bbox_bottom_y 像素逻辑一个字都不变。
DEPTH_STOP_CM = 5.0
DEPTH_SLOW_CM = 15.0
DEPTH_TOO_CLOSE_CM = 2.0
DEPTH_MIN_CM = 2.0
DEPTH_MAX_CM = 400.0
DEPTH_BAD_HINT_FRAMES = 10

ALIGNED_CONFIRM_FRAMES = 5
POSITION_CONFIRM_FRAMES = 5
APPROACH_PAUSE_ERROR = 60
APPROACH_RESUME_ERROR = 40
APPROACH_RESUME_CONFIRM_FRAMES = 3
TARGET_LOST_STOP_FRAMES = 3
VISUAL_APPROACH_TIMEOUT_S = 12.0

CONTROL_HZ = 10
CONFIG_REFRESH_HZ = 2
STOP_FRAME_COUNT = 6
STOP_INTERVAL_S = 0.02

DEBUG_FRAME_HISTORY = 40

VISION_URL = "http://127.0.0.1:8000/api/targets"
CONFIG_URL = "http://127.0.0.1:8000/api/config"
HTTP_TIMEOUT_S = 0.2
API_STALE_TIMEOUT_S = 0.5

SERVICE_WAIT_S = 20.0
CONNECT_GRACE_S = 3.0
HEALTH_PROBE_TIMEOUT_S = 0.5
RETRY_INTERVAL_S = 0.25

SERIAL_PORT = serial_ports.chassis_port()
BAUDRATE = 115200

HEAD = 0x7B
TAIL = 0x7D
INT16_MIN = -32768
INT16_MAX = 32767
VALID_TARGET_STATES = {"LEFT", "RIGHT", "ALIGNED"}

WAIT_TARGET = "WAIT_TARGET"
ALIGN = "ALIGN"
APPROACH = "APPROACH"
APPROACH_REALIGN = "APPROACH_REALIGN"
POSITION_CONFIRM = "POSITION_CONFIRM"
SUCCESS = "SUCCESS"
STOPPED = "STOPPED"
ERROR = "ERROR"


@dataclass
class MotionCommand:
    """One pure chassis command produced by the controller."""

    vx: int = 0
    vy: int = 0
    wz: int = 0
    mode: str = "STOP"


@dataclass
class VisualPositionResult:
    """Terminal result returned by :meth:`VisualBlockPositioner.run`."""

    success: bool
    reason: str
    final_cx: Optional[float] = None
    final_error_x: Optional[float] = None
    final_bottom_y: Optional[float] = None
    final_depth_cm: Optional[float] = None
    elapsed: Optional[float] = None
    detail: Optional[str] = None
    final_state: str = STOPPED


@dataclass
class ControllerStep:
    """Hardware-independent result of one state-machine input frame."""

    state: str
    command: MotionCommand
    terminal_reason: Optional[str] = None


class VisionInputError(RuntimeError):
    """The vision response is unavailable or unsafe to act on."""


class ConfigInputError(RuntimeError):
    """The runtime configuration is unavailable or unsafe to act on."""


class ApiStaleError(VisionInputError):
    """The vision endpoint is reachable but its frame is stale."""


class SerialControlError(RuntimeError):
    """The chassis serial port could not be opened or written."""


class VisionServiceUnavailable(RuntimeError):
    """视觉服务没就绪，此时发车或定位都不可能成功。"""


SERVICE_REPAIR_HINT = """照下面三步修，修完再发车：
  1) 看谁占着摄像头（pipewire / wireplumber 是系统进程，忽略）：
       fuser -v /dev/video0
  2) 停掉占用者。摄像头只能被一个进程打开，比赛时只用视觉服务，
     localize.py / web_stream*.py / 各种 tuner 都要先退出：
       kill <占用PID>              # 不退再 sudo kill -9 <占用PID>
  3) 前台启动并确认（第 2 条 curl 另开一个窗口做）：
       cd /home/pi/cam/color && python3 vision_runtime.py
       curl -s http://127.0.0.1:8000/health      # 必须 "camera": true"""


def _health_url(vision_url: str) -> str:
    return vision_url.split("/api/", 1)[0].rstrip("/") + "/health"


def _is_connection_failure(error: BaseException) -> bool:
    seen = []
    node: Optional[BaseException] = error
    while node is not None and not any(node is item for item in seen):
        seen.append(node)
        if isinstance(
            node,
            (
                ConnectionRefusedError,
                ConnectionResetError,
                ConnectionAbortedError,
                TimeoutError,
                URLError,
            ),
        ):
            return True
        if isinstance(node, OSError) and getattr(node, "errno", None) in (
            101, 104, 110, 111, 113,
        ):
            return True
        node = node.__cause__ or node.__context__

    text = str(error)
    return "HTTP request failed" in text or "Connection refused" in text


def _service_status(
    reachable: bool, ok: bool, camera: bool, detail: str,
    target_count: Any = None,
) -> Dict[str, Any]:
    return {
        "reachable": reachable,
        "ok": ok,
        "camera": camera,
        "target_count": target_count,
        "detail": detail,
    }


def vision_service_status(
    vision_url: str = VISION_URL,
    timeout_s: float = HEALTH_PROBE_TIMEOUT_S,
) -> Dict[str, Any]:
    url = _health_url(vision_url)
    request = Request(url, headers={"Accept": "application/json"})
    try:
        with urlopen(request, timeout=timeout_s) as response:
            body = response.read()
    except HTTPError as error:
        return _service_status(
            False, False, False,
            "{} 有进程在应答，但它不是视觉服务（HTTP {}）：8000 被别的 "
            "web_stream / tuner 占了，先停掉它".format(url, error.code),
        )
    except (URLError, TimeoutError, OSError) as error:
        reason = getattr(error, "reason", error)
        return _service_status(
            False, False, False,
            "{} 连不上（{}）：视觉服务没在跑，或者起完就退出了".format(url, reason),
        )

    try:
        payload = json.loads(body)
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        return _service_status(
            False, False, False,
            "{} 的响应不是合法 JSON：{}".format(url, error),
        )
    if not isinstance(payload, dict):
        return _service_status(
            False, False, False,
            "{} 的响应不是 JSON 对象".format(url),
        )

    ok = payload.get("ok") is True
    camera = payload.get("camera") is True
    if ok and camera:
        detail = "视觉服务正常，camera=true，当前目标数 {}".format(
            payload.get("target_count")
        )
    elif ok:
        detail = (
            "视觉服务在跑，但 camera=false：摄像头没出图（被抢、掉线或采集线程挂了），"
            "/api/targets 会一直是空的，不要拿它定位"
        )
    else:
        detail = "{} 返回 ok=false：{}".format(url, payload)
    return _service_status(True, ok, camera, detail, payload.get("target_count"))


def wait_for_vision_service(
    timeout_s: float = SERVICE_WAIT_S,
    vision_url: str = VISION_URL,
    require_camera: bool = True,
    verbose: bool = True,
) -> bool:
    deadline = time.monotonic() + max(float(timeout_s), 0.0)
    printed = None
    attempt = 0
    while True:
        attempt += 1
        status = vision_service_status(vision_url)
        ready = status["ok"] and (status["camera"] or not require_camera)
        if ready:
            if verbose:
                print(
                    "[VISION] 服务就绪（第 {} 次探测，目标数 {}）".format(
                        attempt, status["target_count"]
                    )
                )
            return True

        now = time.monotonic()
        if now >= deadline:
            if verbose:
                print("[VISION] 等待 {:.0f}s 仍未就绪：{}".format(
                    timeout_s, status["detail"]))
            return False
        if verbose and status["detail"] != printed:
            printed = status["detail"]
            print(
                "[VISION] 还没就绪：{}；继续等，最多 {:.0f}s".format(
                    status["detail"], timeout_s
                )
            )
        time.sleep(min(RETRY_INTERVAL_S, max(deadline - now, 0.0)))


def require_vision_service(
    timeout_s: float = SERVICE_WAIT_S,
    vision_url: str = VISION_URL,
    require_camera: bool = True,
    verbose: bool = True,
) -> None:
    if wait_for_vision_service(timeout_s, vision_url, require_camera, verbose):
        return

    status = vision_service_status(vision_url)
    message = "视觉服务未就绪，不执行定位：{}".format(status["detail"])
    print("[VISION] {}".format(message))
    print("[VISION] {}".format(SERVICE_REPAIR_HINT))
    raise VisionServiceUnavailable(message)


def _fetch_targets_payload(
    vision_url: str = VISION_URL,
    timeout: float = HTTP_TIMEOUT_S,
) -> Any:
    request = Request(vision_url, headers={"Accept": "application/json"})
    with urlopen(request, timeout=timeout) as response:
        return json.loads(response.read())


def _color_visible(payload: Any, color: str) -> bool:
    if not isinstance(payload, dict) or payload.get("ok") is not True:
        return False
    targets = payload.get("targets")
    if not isinstance(targets, list):
        return False
    for target in targets:
        if isinstance(target, dict) and target.get("color") == color:
            return True
    return False


def bcc(data: bytes) -> int:
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def build_speed_frame(vx: int, vy: int, wz: int) -> bytes:
    frame = bytearray([HEAD, 0x00, 0x00])
    for value in (vx, vy, wz):
        value = max(INT16_MIN, min(INT16_MAX, int(value)))
        frame += struct.pack(">h", value)
    frame.append(bcc(frame[:9]))
    frame.append(TAIL)
    return bytes(frame)


def expected_state_for_error(
    error_x: float, align_deadband: int = ALIGN_DEADBAND
) -> str:
    if error_x < -align_deadband:
        return "LEFT"
    if error_x > align_deadband:
        return "RIGHT"
    return "ALIGNED"


def compute_align_command(
    error_x: float,
    align_deadband: int = ALIGN_DEADBAND,
    vy_fast: int = ALIGN_VY_FAST,
    vy_slow: int = ALIGN_VY_SLOW,
    slow_threshold: int = ALIGN_SLOW_THRESHOLD,
) -> MotionCommand:
    target_state = expected_state_for_error(error_x, align_deadband)
    if target_state == "ALIGNED":
        return MotionCommand(mode="ALIGNED")
    if abs(error_x) > slow_threshold:
        speed = vy_fast
        mode = "FAST"
    else:
        speed = vy_slow
        mode = "SLOW"
    vy = speed if target_state == "LEFT" else -speed
    return MotionCommand(vy=vy, mode=mode)


def compute_approach_speed(bottom_y: float) -> Tuple[int, str]:
    slow_start_y = TARGET_BOTTOM_Y - BOTTOM_Y_SLOW_ZONE
    position_min_y = TARGET_BOTTOM_Y - BOTTOM_Y_TOLERANCE
    position_max_y = TARGET_BOTTOM_Y + BOTTOM_Y_TOLERANCE

    if bottom_y > position_max_y:
        return 0, "TOO_CLOSE"
    if bottom_y >= position_min_y:
        return 0, "POSITION_RANGE"
    if bottom_y >= slow_start_y:
        return APPROACH_VX_SLOW, "FORWARD_SLOW"
    return APPROACH_VX_FAST, "FORWARD_FAST"


def compute_approach_speed_cm(depth_cm: float) -> Tuple[int, str]:
    if depth_cm < DEPTH_TOO_CLOSE_CM:
        return 0, "TOO_CLOSE"
    if depth_cm <= DEPTH_STOP_CM:
        return 0, "POSITION_RANGE"
    if depth_cm <= DEPTH_SLOW_CM:
        return APPROACH_VX_SLOW, "FORWARD_SLOW"
    return APPROACH_VX_FAST, "FORWARD_FAST"


def compute_approach_command(
    error_x: float,
    bbox_bottom_y: float,
    align_deadband: int = ALIGN_DEADBAND,
    vy_fast: int = ALIGN_VY_FAST,
    vy_slow: int = ALIGN_VY_SLOW,
    slow_threshold: int = ALIGN_SLOW_THRESHOLD,
    depth_cm: Optional[float] = None,
) -> MotionCommand:
    if depth_cm is None:
        vx, mode = compute_approach_speed(bbox_bottom_y)
    else:
        vx, mode = compute_approach_speed_cm(depth_cm)
    if mode == "TOO_CLOSE":
        return MotionCommand(mode=mode)

    target_state = expected_state_for_error(error_x, align_deadband)
    error_magnitude = abs(error_x)
    if error_magnitude > APPROACH_PAUSE_ERROR:
        align_command = compute_align_command(
            error_x, align_deadband, vy_fast, vy_slow, slow_threshold,
        )
        align_command.mode = "REALIGN_" + align_command.mode
        return align_command

    if mode == "POSITION_RANGE" and target_state == "ALIGNED":
        return MotionCommand(mode=mode)

    correction_vy = 0
    if target_state == "LEFT":
        correction_vy = APPROACH_CORRECTION_VY
        mode += "_CORRECT_LEFT"
    elif target_state == "RIGHT":
        correction_vy = -APPROACH_CORRECTION_VY
        mode += "_CORRECT_RIGHT"
    return MotionCommand(vx=vx, vy=correction_vy, mode=mode)


def _is_finite_number(value: Any) -> bool:
    return bool(
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _validated_config_integer(
    config: Dict[str, Any], key: str, minimum: int, maximum: int
) -> int:
    if key not in config:
        raise ConfigInputError("config.{} is missing".format(key))
    value = config[key]
    if not _is_finite_number(value) or not float(value).is_integer():
        raise ConfigInputError("config.{} must be an integer".format(key))
    integer = int(value)
    if not minimum <= integer <= maximum:
        raise ConfigInputError(
            "config.{} must be in range [{}, {}]".format(
                key, minimum, maximum
            )
        )
    return integer


def _validate_runtime_config(payload: Any) -> Dict[str, int]:
    if not isinstance(payload, dict):
        raise ConfigInputError("config API payload is not an object")
    if payload.get("ok") is not True:
        raise ConfigInputError("config API ok is not true")
    config = payload.get("config")
    if not isinstance(config, dict):
        raise ConfigInputError("config is missing or is not an object")

    runtime_config = {
        "TARGET_X": _validated_config_integer(config, "TARGET_X", 200, 440),
        "ALIGN_DEADBAND": _validated_config_integer(
            config, "ALIGN_DEADBAND", 3, 50
        ),
        "ALIGN_VY_FAST": _validated_config_integer(
            config, "ALIGN_VY_FAST", 10, 120
        ),
        "ALIGN_VY_SLOW": _validated_config_integer(
            config, "ALIGN_VY_SLOW", 5, 80
        ),
        "ALIGN_SLOW_THRESHOLD": _validated_config_integer(
            config, "ALIGN_SLOW_THRESHOLD", 20, 150
        ),
    }
    if runtime_config["ALIGN_VY_SLOW"] > runtime_config["ALIGN_VY_FAST"]:
        raise ConfigInputError("ALIGN_VY_SLOW must not exceed ALIGN_VY_FAST")
    if runtime_config["ALIGN_DEADBAND"] >= runtime_config["ALIGN_SLOW_THRESHOLD"]:
        raise ConfigInputError(
            "ALIGN_DEADBAND must be smaller than ALIGN_SLOW_THRESHOLD"
        )
    return runtime_config


def _validated_geometry_value(
    target: Dict[str, Any],
    key: str,
    minimum: float,
    maximum: float,
    minimum_inclusive: bool = True,
) -> float:
    if key not in target:
        raise VisionInputError("selected_target.{} is missing".format(key))
    value = target[key]
    if not _is_finite_number(value):
        raise VisionInputError("invalid {}: {!r}".format(key, value))
    number = float(value)
    lower_ok = number >= minimum if minimum_inclusive else number > minimum
    if not lower_ok or number > maximum:
        lower_symbol = ">=" if minimum_inclusive else ">"
        raise VisionInputError(
            "{} must be {} {} and <= {}".format(
                key, lower_symbol, minimum, maximum
            )
        )
    return number


def _select_target(payload: Dict[str, Any], color: Optional[str]) -> Any:
    if color is None:
        if "selected_target" not in payload:
            raise VisionInputError("selected_target is missing")
        return payload["selected_target"]

    targets = payload.get("targets")
    if not isinstance(targets, list):
        raise VisionInputError("targets is missing or is not an array")
    candidates = []
    for target in targets:
        if not isinstance(target, dict):
            raise VisionInputError("each target must be an object")
        if target.get("color") == color:
            candidates.append(target)
    if not candidates:
        return None
    try:
        return min(
            candidates,
            key=lambda target: (
                abs(float(target["error_x"])),
                -float(target["area"]),
            ),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise VisionInputError("invalid color-filter target: {}".format(error))


def _fmt(value: Optional[float], digits: int = 1) -> str:
    if value is None:
        return "?"
    return "{:.{d}f}".format(value, d=digits)


def _target_number(target: Any, *keys: str) -> Optional[float]:
    if not isinstance(target, dict):
        return None
    for key in keys:
        if key in target and _is_finite_number(target[key]):
            return float(target[key])
    bbox = target.get("bbox")
    if isinstance(bbox, dict):
        for key in keys:
            short = key.rsplit("_", 1)[-1]
            if short in bbox and _is_finite_number(bbox[short]):
                return float(bbox[short])
    return None


def _debug_target_dump(target: Any) -> str:
    if not isinstance(target, dict):
        return repr(target)
    parts = []
    for key in sorted(target):
        value = target[key]
        if isinstance(value, dict):
            inner = " ".join(
                "{}={}".format(k, _fmt(_target_number(value, k)))
                for k in sorted(value)
            )
            parts.append("{}{{{}}}".format(key, inner))
        elif _is_finite_number(value):
            parts.append("{}={}".format(key, _fmt(float(value))))
        else:
            parts.append("{}={!r}".format(key, value))
    return " ".join(parts)


def _debug_track(stats: Dict[str, Any], target: Optional[Dict[str, Any]]) -> None:
    stats["frames"] += 1
    bottom_y = _target_number(target, "bbox_bottom_y", "bottom_y")
    error_x = _target_number(target, "error_x")
    for key, value in (
        ("by_min", bottom_y),
        ("by_max", bottom_y),
        ("err_min", error_x),
        ("err_max", error_x),
    ):
        if value is None:
            continue
        current = stats[key]
        if current is None:
            stats[key] = value
        elif key.endswith("_min"):
            stats[key] = min(current, value)
        else:
            stats[key] = max(current, value)


def _validate_target_payload(
    payload: Any,
    runtime_config: Dict[str, int],
    color: Optional[str],
) -> Tuple[Optional[Dict[str, Any]], float]:
    if not isinstance(payload, dict):
        raise VisionInputError("API payload is not an object")
    if payload.get("ok") is not True:
        raise VisionInputError("API ok is not true")
    if "timestamp" not in payload or not _is_finite_number(payload["timestamp"]):
        raise VisionInputError("timestamp is missing or invalid")

    selected_target = _select_target(payload, color)
    timestamp = float(payload["timestamp"])
    if selected_target is None:
        return None, timestamp
    if not isinstance(selected_target, dict):
        raise VisionInputError("selected_target is not an object")

    target_state = selected_target.get("state")
    error_x = selected_target.get("error_x")
    cx = selected_target.get("cx")
    if not isinstance(target_state, str) or target_state not in VALID_TARGET_STATES:
        raise VisionInputError("invalid target state: {!r}".format(target_state))
    if not _is_finite_number(error_x):
        raise VisionInputError("invalid error_x: {!r}".format(error_x))
    if not _is_finite_number(cx):
        raise VisionInputError("invalid cx: {!r}".format(cx))

    expected_error_x = float(cx) - runtime_config["TARGET_X"]
    if abs(float(error_x) - expected_error_x) > 1.0:
        raise VisionInputError(
            "TARGET_X/error_x mismatch: cx={}, error_x={}, TARGET_X={}".format(
                cx, error_x, runtime_config["TARGET_X"]
            )
        )
    expected_state = expected_state_for_error(
        float(error_x), runtime_config["ALIGN_DEADBAND"]
    )
    if target_state != expected_state:
        raise VisionInputError(
            "state/error_x mismatch: state={}, expected={}, error_x={}, "
            "deadband={}".format(
                target_state, expected_state, error_x,
                runtime_config["ALIGN_DEADBAND"],
            )
        )

    _validated_geometry_value(selected_target, "bbox_bottom_y", 0.0, 480.0)
    _validated_geometry_value(selected_target, "cy", 0.0, 480.0)
    _validated_geometry_value(
        selected_target, "bbox_h", 0.0, 480.0, minimum_inclusive=False
    )
    return selected_target, timestamp


class VisualPositionController:
    """Pure state machine corresponding to the stable test program."""

    def __init__(self, runtime_config: Optional[Dict[str, int]] = None) -> None:
        self.runtime_config = dict(
            runtime_config
            or {
                "TARGET_X": TARGET_X,
                "ALIGN_DEADBAND": ALIGN_DEADBAND,
                "ALIGN_VY_FAST": ALIGN_VY_FAST,
                "ALIGN_VY_SLOW": ALIGN_VY_SLOW,
                "ALIGN_SLOW_THRESHOLD": ALIGN_SLOW_THRESHOLD,
            }
        )
        self.state = WAIT_TARGET
        self.aligned_confirm_count = 0
        self.realign_confirm_count = 0
        self.position_confirm_count = 0
        self.target_lost_count = 0
        self.has_seen_target = False
        self.approach_start_time: Optional[float] = None
        self.uses_depth_cm = False

    def update_runtime_config(self, runtime_config: Dict[str, int]) -> None:
        self.runtime_config = dict(runtime_config)

    def _depth_too_close(self, bottom_y: float, depth_cm: Optional[float]) -> bool:
        if self.uses_depth_cm:
            return depth_cm is not None and depth_cm < DEPTH_TOO_CLOSE_CM
        return bottom_y > TARGET_BOTTOM_Y + BOTTOM_Y_TOLERANCE

    def _depth_still_far(self, bottom_y: float, depth_cm: Optional[float]) -> bool:
        if self.uses_depth_cm:
            return depth_cm is not None and depth_cm > DEPTH_STOP_CM
        return bottom_y < TARGET_BOTTOM_Y - BOTTOM_Y_TOLERANCE

    def _align_command(self, error_x: float) -> MotionCommand:
        return compute_align_command(
            error_x,
            self.runtime_config["ALIGN_DEADBAND"],
            self.runtime_config["ALIGN_VY_FAST"],
            self.runtime_config["ALIGN_VY_SLOW"],
            self.runtime_config["ALIGN_SLOW_THRESHOLD"],
        )

    def _approach_command(
        self, error_x: float, bottom_y: float, depth_cm: Optional[float] = None
    ) -> MotionCommand:
        if self.uses_depth_cm and depth_cm is None:
            command = self._align_command(error_x)
            command.mode = "DEPTH_UNKNOWN"
            return command
        return compute_approach_command(
            error_x,
            bottom_y,
            self.runtime_config["ALIGN_DEADBAND"],
            self.runtime_config["ALIGN_VY_FAST"],
            self.runtime_config["ALIGN_VY_SLOW"],
            self.runtime_config["ALIGN_SLOW_THRESHOLD"],
            depth_cm=depth_cm,
        )

    def stop(self, reason: str) -> ControllerStep:
        self.state = SUCCESS if reason == SUCCESS else STOPPED
        return ControllerStep(self.state, MotionCommand(), reason)

    def step(
        self,
        target: Optional[Dict[str, Any]],
        now: float,
        depth_cm: Optional[float] = None,
    ) -> ControllerStep:
        if self.state in {SUCCESS, STOPPED, ERROR}:
            return ControllerStep(self.state, MotionCommand())

        if target is None:
            self.target_lost_count += 1
            self.aligned_confirm_count = 0
            self.realign_confirm_count = 0
            self.position_confirm_count = 0
            if self.target_lost_count >= TARGET_LOST_STOP_FRAMES:
                reason = "TARGET_LOST" if self.has_seen_target else "TARGET_NOT_FOUND"
                return self.stop(reason)
            return ControllerStep(self.state, MotionCommand(mode="NO_TARGET"))

        self.has_seen_target = True
        self.target_lost_count = 0
        error_x = float(target["error_x"])
        bottom_y = float(target["bbox_bottom_y"])

        if self._depth_too_close(bottom_y, depth_cm):
            return self.stop("TOO_CLOSE")

        if self.state == WAIT_TARGET:
            self.state = ALIGN

        if self.state == ALIGN:
            command = self._align_command(error_x)
            if command.mode == "ALIGNED":
                self.aligned_confirm_count += 1
            else:
                self.aligned_confirm_count = 0
            if self.aligned_confirm_count >= ALIGNED_CONFIRM_FRAMES:
                self.state = APPROACH
                self.approach_start_time = now
                self.realign_confirm_count = 0
                self.position_confirm_count = 0
                return ControllerStep(self.state, MotionCommand(mode="ALIGN_CONFIRMED"))
            return ControllerStep(self.state, command)

        if self.state == APPROACH:
            command = self._approach_command(error_x, bottom_y, depth_cm)
            if command.mode == "TOO_CLOSE":
                return self.stop("TOO_CLOSE")
            if command.mode.startswith("REALIGN_"):
                self.state = APPROACH_REALIGN
                self.realign_confirm_count = 0
                self.position_confirm_count = 0
                return ControllerStep(self.state, command)
            if (
                command.mode == "POSITION_RANGE"
                and abs(error_x) <= self.runtime_config["ALIGN_DEADBAND"]
            ):
                self.state = POSITION_CONFIRM
                self.position_confirm_count = 1
                return ControllerStep(self.state, command)
            return ControllerStep(self.state, command)

        if self.state == APPROACH_REALIGN:
            if self._depth_too_close(bottom_y, depth_cm):
                return self.stop("TOO_CLOSE")
            command = self._align_command(error_x)
            if abs(error_x) <= APPROACH_RESUME_ERROR:
                self.realign_confirm_count += 1
            else:
                self.realign_confirm_count = 0
            if self.realign_confirm_count >= APPROACH_RESUME_CONFIRM_FRAMES:
                self.state = APPROACH
                self.realign_confirm_count = 0
                return ControllerStep(self.state, MotionCommand(mode="REALIGN_COMPLETE"))
            return ControllerStep(self.state, command)

        if self.state == POSITION_CONFIRM:
            if self._depth_too_close(bottom_y, depth_cm):
                self.position_confirm_count = 0
                return self.stop("TOO_CLOSE")
            if self.uses_depth_cm and depth_cm is None:
                return ControllerStep(
                    self.state, MotionCommand(mode="DEPTH_UNKNOWN")
                )
            if self._depth_still_far(bottom_y, depth_cm):
                self.position_confirm_count = 0
                self.state = APPROACH
                return ControllerStep(
                    self.state, MotionCommand(mode="POSITION_BELOW_RANGE")
                )
            if abs(error_x) > self.runtime_config["ALIGN_DEADBAND"]:
                self.position_confirm_count = 0
                self.realign_confirm_count = 0
                self.state = APPROACH_REALIGN
                return ControllerStep(self.state, self._align_command(error_x))

            self.position_confirm_count += 1
            if self.position_confirm_count >= POSITION_CONFIRM_FRAMES:
                return self.stop(SUCCESS)
            return ControllerStep(
                self.state, MotionCommand(mode="POSITION_RANGE")
            )

        self.state = ERROR
        return ControllerStep(self.state, MotionCommand(), "VISION_ERROR")


class VisualBlockPositioner:
    """Reusable owner of the complete color-block positioning action."""

    def __init__(
        self,
        color: Optional[str] = None,
        timeout: float = VISUAL_APPROACH_TIMEOUT_S,
        dry_run: bool = False,
        vision_url: str = VISION_URL,
        config_url: str = CONFIG_URL,
        serial_port: Optional[str] = None,
        baudrate: int = BAUDRATE,
        http_timeout: float = HTTP_TIMEOUT_S,
        stale_timeout: float = API_STALE_TIMEOUT_S,
        connect_grace: float = CONNECT_GRACE_S,
        service_wait: float = SERVICE_WAIT_S,
        debug: bool = False,
        depth_cm_fn: Any = None,
    ) -> None:
        if color is not None and color not in {"orange", "purple"}:
            raise ValueError("color must be None, 'orange', or 'purple'")
        if timeout <= 0 or http_timeout <= 0 or stale_timeout <= 0:
            raise ValueError("timeouts must be positive")
        if connect_grace < 0 or service_wait < 0:
            raise ValueError("connect_grace and service_wait must not be negative")
        self.color = color
        self.timeout = float(timeout)
        self.dry_run = bool(dry_run)
        self.vision_url = vision_url
        self.config_url = config_url
        self.serial_port = serial_port or serial_ports.chassis_port()
        self.baudrate = int(baudrate)
        self.http_timeout = float(http_timeout)
        self.stale_timeout = float(stale_timeout)
        self.connect_grace = float(connect_grace)
        self.service_wait = float(service_wait)
        self.debug = bool(debug)
        self.depth_cm_fn = depth_cm_fn

        self._last_target_timestamp: Optional[float] = None
        self._last_new_timestamp_time: Optional[float] = None
        self._last_debug_candidates_time = 0.0
        self._debug_config_done = False
        self._last_depth_cm: Optional[float] = None
        self._depth_bad_frames = 0

    def _read_depth_cm(self) -> Optional[float]:
        value = None
        reason = None
        try:
            value = self.depth_cm_fn()
        except Exception as error:
            reason = "读取异常 {}".format(error)
        if reason is None:
            if not _is_finite_number(value):
                reason = "读数不是数字 {!r}".format(value)
            elif not (DEPTH_MIN_CM <= float(value) <= DEPTH_MAX_CM):
                reason = "读数 {} cm 不在 {}~{} 内".format(
                    value, DEPTH_MIN_CM, DEPTH_MAX_CM
                )

        if reason is not None:
            self._depth_bad_frames += 1
            if self.debug:
                if self._depth_bad_frames == DEPTH_BAD_HINT_FRAMES:
                    print(
                        "[POS] 测距连续 {} 帧读不到（最后原因：{}）。查三件事："
                        "1) 探头接线/供电 2) 前面有没有东西在量程内 "
                        "3) 有没有别的程序（例如 distance.py）同时占着 GPIO —— "
                        "占了就会像这样一直读不到".format(
                            self._depth_bad_frames, reason
                        )
                    )
                elif self._depth_bad_frames < DEPTH_BAD_HINT_FRAMES:
                    print("[POS] 测距不可用（{}），这一帧不往前开".format(reason))
            return None

        self._depth_bad_frames = 0
        return float(value)

    def _fetch_json(self, url: str, error_type: Any) -> Any:
        request = Request(url, headers={"Accept": "application/json"})
        try:
            with urlopen(request, timeout=self.http_timeout) as response:
                status = getattr(response, "status", 200)
                if status != 200:
                    raise error_type("HTTP status {}".format(status))
                body = response.read()
        except (HTTPError, URLError, TimeoutError, OSError) as error:
            raise error_type("HTTP request failed: {}".format(error)) from error
        try:
            return json.loads(body)
        except (json.JSONDecodeError, UnicodeDecodeError) as error:
            raise error_type("invalid JSON: {}".format(error)) from error

    def _fetch_runtime_config(self) -> Dict[str, int]:
        return _validate_runtime_config(
            self._fetch_json(self.config_url, ConfigInputError)
        )

    def _check_fresh_timestamp(self, timestamp: float, now: float) -> None:
        wall_delta = time.time() - timestamp
        if wall_delta > self.stale_timeout:
            raise ApiStaleError("vision frame timestamp is too old")
        if wall_delta < -self.stale_timeout:
            raise ApiStaleError("vision frame timestamp is unexpectedly in the future")

        if self._last_target_timestamp is None or timestamp != self._last_target_timestamp:
            self._last_target_timestamp = timestamp
            self._last_new_timestamp_time = now
            return
        if (
            self._last_new_timestamp_time is not None
            and now - self._last_new_timestamp_time > self.stale_timeout
        ):
            raise ApiStaleError("vision frame timestamp did not advance")

    def _fetch_selected_target(
        self, runtime_config: Dict[str, int], now: float
    ) -> Optional[Dict[str, Any]]:
        payload = self._fetch_json(self.vision_url, VisionInputError)
        target, timestamp = _validate_target_payload(
            payload, runtime_config, self.color
        )
        if self.debug:
            self._debug_candidates(payload, target)
        self._check_fresh_timestamp(timestamp, now)
        return target

    def _wait_for_service(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        while True:
            status = vision_service_status(self.vision_url)
            if status["ok"] and status["camera"]:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(RETRY_INTERVAL_S)

    def _handle_vision_failure(
        self,
        reason: str,
        error: BaseException,
        serial_connection: Any,
        started_at: float,
        last_target: Optional[Dict[str, Any]],
    ) -> Optional[VisualPositionResult]:
        self._stop_once(serial_connection)
        if self.connect_grace <= 0 or not _is_connection_failure(error):
            return self._result(reason, started_at, last_target, str(error))

        print(
            "[VISION] 视觉服务连接失败（{}），已停车，等它 {:.1f}s 恢复".format(
                error, self.connect_grace
            )
        )
        if not self._wait_for_service(self.connect_grace):
            return self._result(reason, started_at, last_target, str(error))
        print("[VISION] 视觉服务已恢复，继续定位")
        return None

    def _open_serial(self) -> Any:
        if self.dry_run:
            return None
        try:
            import serial
            return serial.Serial(
                self.serial_port,
                self.baudrate,
                timeout=0.1,
                write_timeout=0.2,
            )
        except Exception as error:
            raise SerialControlError(
                "could not open {} @ {}: {}".format(
                    self.serial_port, self.baudrate, error
                )
            ) from error

    @staticmethod
    def _write_command(serial_connection: Any, command: MotionCommand) -> None:
        if serial_connection is None:
            return
        try:
            serial_connection.write(
                build_speed_frame(command.vx, command.vy, command.wz)
            )
        except Exception as error:
            raise SerialControlError("serial write failed: {}".format(error)) from error

    @staticmethod
    def _stop_once(serial_connection: Any) -> None:
        VisualBlockPositioner._write_command(serial_connection, MotionCommand())

    @staticmethod
    def _stop_repeatedly(serial_connection: Any) -> Optional[str]:
        if serial_connection is None:
            return None
        first_error = None
        stop_frame = build_speed_frame(0, 0, 0)
        for _ in range(STOP_FRAME_COUNT):
            try:
                serial_connection.write(stop_frame)
            except Exception as error:
                if first_error is None:
                    first_error = "failed to send stop frame: {}".format(error)
            time.sleep(STOP_INTERVAL_S)
        return first_error

    def _result(
        self,
        reason: str,
        started_at: float,
        target: Optional[Dict[str, Any]],
        detail: Optional[str] = None,
        final_state: str = STOPPED,
    ) -> VisualPositionResult:
        return VisualPositionResult(
            success=reason == SUCCESS,
            reason=reason,
            final_cx=(float(target["cx"]) if target is not None else None),
            final_error_x=(
                float(target["error_x"]) if target is not None else None
            ),
            final_bottom_y=(
                float(target["bbox_bottom_y"]) if target is not None else None
            ),
            final_depth_cm=self._last_depth_cm,
            elapsed=time.monotonic() - started_at,
            detail=detail,
            final_state=final_state,
        )

    def _debug_banner(self) -> None:
        print("[POS] ===== 视觉定位调试日志开（debug=True）=====")
        print("[POS] 实际运行的模块文件 {}".format(__file__))
        if self.depth_cm_fn is None:
            print(
                "[POS] TARGET_BOTTOM_Y={} 容差±{} 到位窗口 {}~{} 减速点 {}".format(
                    TARGET_BOTTOM_Y,
                    BOTTOM_Y_TOLERANCE,
                    TARGET_BOTTOM_Y - BOTTOM_Y_TOLERANCE,
                    TARGET_BOTTOM_Y + BOTTOM_Y_TOLERANCE,
                    TARGET_BOTTOM_Y - BOTTOM_Y_SLOW_ZONE,
                )
            )
        else:
            print(
                "[POS] 本趟深度只看测距，TARGET_BOTTOM_Y={} 那套像素阈值不参与判断"
                "（停多远由 DEPTH_STOP_CM={}cm 决定）".format(
                    TARGET_BOTTOM_Y, DEPTH_STOP_CM
                )
            )
        print(
            "[POS] 速度 vx_fast={} vx_slow={} | 对齐 vy_fast={} vy_slow={} "
            "慢速分界={} | 对准需连续 {} 帧，到位需连续 {} 帧 | 趋近超时 {:.1f}s".format(
                APPROACH_VX_FAST, APPROACH_VX_SLOW,
                ALIGN_VY_FAST, ALIGN_VY_SLOW, ALIGN_SLOW_THRESHOLD,
                ALIGNED_CONFIRM_FRAMES, POSITION_CONFIRM_FRAMES, self.timeout,
            )
        )
        print(
            "[POS] 颜色过滤={} 视觉地址={} 串口={} dry_run={}".format(
                self.color, self.vision_url, self.serial_port, self.dry_run
            )
        )
        if self.depth_cm_fn is not None:
            print(
                "[POS] 深度=测距模块(厘米)：到位 <= {}cm，慢爬 <= {}cm，太近 < {}cm，"
                "噪声过滤 {}~{}cm（纵向看测距，横向看视觉）".format(
                    DEPTH_STOP_CM, DEPTH_SLOW_CM, DEPTH_TOO_CLOSE_CM,
                    DEPTH_MIN_CM, DEPTH_MAX_CM,
                )
            )
        else:
            print("[POS] 深度=视觉 bbox_bottom_y(像素)，本趟没接测距模块")

    def _debug_config(self, runtime_config: Dict[str, int]) -> None:
        if self._debug_config_done:
            return
        self._debug_config_done = True
        print(
            "[POS] 服务端生效配置 TARGET_X={} ALIGN_DEADBAND=±{} "
            "ALIGN_VY_FAST={} ALIGN_VY_SLOW={} ALIGN_SLOW_THRESHOLD={}".format(
                runtime_config["TARGET_X"],
                runtime_config["ALIGN_DEADBAND"],
                runtime_config["ALIGN_VY_FAST"],
                runtime_config["ALIGN_VY_SLOW"],
                runtime_config["ALIGN_SLOW_THRESHOLD"],
            )
        )

    def _debug_candidates(self, payload: Dict[str, Any], selected: Any) -> None:
        now = time.monotonic()
        if now - self._last_debug_candidates_time < 1.0:
            return
        self._last_debug_candidates_time = now
        targets = payload.get("targets")
        if not isinstance(targets, list):
            print("[POS] /api/targets 里没有 targets 数组：{}".format(payload))
            return
        print(
            "[POS] 候选目标 {} 个（<< 是本帧选中的，过滤色={}）".format(
                len(targets), self.color
            )
        )
        for index, item in enumerate(targets):
            mark = "<<" if item is selected else "  "
            print("[POS]   {} #{} {}".format(mark, index, _debug_target_dump(item)))

    def _debug_frame(
        self,
        step: ControllerStep,
        target: Optional[Dict[str, Any]],
        elapsed: float,
    ) -> str:
        head = "[POS] {:6.2f}s {:14s} {:22s}".format(
            elapsed, step.state, step.command.mode
        )
        if self.depth_cm_fn is None:
            depth_text = "未接测距"
        elif self._last_depth_cm is None:
            depth_text = "读不到"
        else:
            depth_text = "{:.1f}cm".format(self._last_depth_cm)
        if target is None:
            line = "{} 本帧没有目标，发停车帧".format(head)
        else:
            line = (
                "{head} vx={vx:4d} vy={vy:4d} wz={wz:3d} | cx={cx} err={err} "
                "by={by} d={d} w={w} h={h} area={area} 服务端state={state}"
            ).format(
                head=head,
                vx=step.command.vx,
                vy=step.command.vy,
                wz=step.command.wz,
                cx=_fmt(_target_number(target, "cx")),
                err=_fmt(_target_number(target, "error_x")),
                by=_fmt(_target_number(target, "bbox_bottom_y", "bottom_y")),
                d=depth_text,
                w=_fmt(_target_number(target, "bbox_w", "w")),
                h=_fmt(_target_number(target, "bbox_h", "h")),
                area=_fmt(_target_number(target, "area"), 0),
                state=target.get("state"),
            )
        print(line)
        return line

    def _debug_stop(
        self,
        result: Optional[VisualPositionResult],
        last_target: Optional[Dict[str, Any]],
        frame_history: List[str],
        stats: Dict[str, Any],
        runtime_config: Optional[Dict[str, int]],
        started_at: float,
    ) -> None:
        reason = result.reason if result is not None else "NO_RESULT"
        state = result.final_state if result is not None else "?"
        print(
            "[POS] ===== 结束：{}（最后状态 {}，总耗时 {:.2f}s，循环 {} 帧）=====".format(
                reason, state, time.monotonic() - started_at, stats["frames"]
            )
        )

        bottom_y = _target_number(last_target, "bbox_bottom_y", "bottom_y")
        lower = TARGET_BOTTOM_Y - BOTTOM_Y_TOLERANCE
        upper = TARGET_BOTTOM_Y + BOTTOM_Y_TOLERANCE
        if self.depth_cm_fn is not None:
            print(
                "[POS] 最后一次测距 {}（到位判据 <= {}cm，太近下限 {}cm）".format(
                    "读不到"
                    if self._last_depth_cm is None
                    else "{:.1f}cm".format(self._last_depth_cm),
                    DEPTH_STOP_CM, DEPTH_TOO_CLOSE_CM,
                )
            )
        if reason == "TOO_CLOSE":
            if self.depth_cm_fn is not None:
                print(
                    "[POS] TOO_CLOSE 判定（测距）：d={} 低于下限 {}cm".format(
                        _fmt(self._last_depth_cm), DEPTH_TOO_CLOSE_CM
                    )
                )
            else:
                print(
                    "[POS] TOO_CLOSE 判定：bottom_y={} 超过上限 {}"
                    "（TARGET_BOTTOM_Y {} + 容差 {}）".format(
                        _fmt(bottom_y), upper, TARGET_BOTTOM_Y, BOTTOM_Y_TOLERANCE
                    )
                )
            print(
                "[POS] 被判太近的那一帧：w={} h={} area={} cx={} err={}".format(
                    _fmt(_target_number(last_target, "bbox_w", "w")),
                    _fmt(_target_number(last_target, "bbox_h", "h")),
                    _fmt(_target_number(last_target, "area"), 0),
                    _fmt(_target_number(last_target, "cx")),
                    _fmt(_target_number(last_target, "error_x")),
                )
            )
            print(
                "[POS] 注意：这条判定在每个状态都会做，包括还没对准的 WAIT_TARGET/"
                "ALIGN。上面若显示 0.0s 附近就结束，说明车一进来就已经太近"
            )
        elif reason == "TIMEOUT":
            if bottom_y is None:
                print("[POS] 超时期间一直没有目标（看上面的 [VISION]/候选行）")
            else:
                print(
                    "[POS] 超时：{}s 内没把 bottom_y 开进 {}~{}，最后一次 {}"
                    "（距离上限还差 {}）".format(
                        _fmt(result.elapsed if result is not None else None),
                        lower, upper, _fmt(bottom_y),
                        _fmt(lower - bottom_y) if bottom_y < lower else "已过",
                    )
                )
        elif reason == "TARGET_NOT_FOUND":
            print("[POS] 目标丢失：连续 {} 帧没目标就停".format(TARGET_LOST_STOP_FRAMES))

        print(
            "[POS] 本趟 bottom_y 范围 {}~{}（到位窗口 {}~{}），"
            "error_x 范围 {}~{}".format(
                _fmt(stats["by_min"]), _fmt(stats["by_max"]),
                lower, upper,
                _fmt(stats["err_min"]), _fmt(stats["err_max"]),
            )
        )
        if runtime_config is not None:
            print(
                "[POS] 结束时服务端配置 TARGET_X={} ALIGN_DEADBAND=±{}".format(
                    runtime_config["TARGET_X"], runtime_config["ALIGN_DEADBAND"]
                )
            )
        if last_target is not None:
            print("[POS] 最后一次收到的目标 {}".format(_debug_target_dump(last_target)))
        if frame_history:
            print("[POS] 最后 {} 帧：".format(len(frame_history)))
            for line in frame_history[-12:]:
                print(line)

    def run(self, timeout: Optional[float] = None) -> VisualPositionResult:
        approach_timeout = self.timeout if timeout is None else float(timeout)
        if approach_timeout <= 0:
            raise ValueError("timeout must be positive")

        started_at = time.monotonic()
        controller = VisualPositionController()
        controller.uses_depth_cm = self.depth_cm_fn is not None
        serial_connection = None
        last_target = None
        result = None
        next_loop = started_at
        next_config_refresh = started_at
        runtime_config = None

        self._last_target_timestamp = None
        self._last_new_timestamp_time = None
        self._last_debug_candidates_time = 0.0
        self._debug_config_done = False
        self._last_depth_cm = None
        self._depth_bad_frames = 0

        frame_history: List[str] = []
        stats: Dict[str, Any] = {
            "frames": 0, "by_min": None, "by_max": None,
            "err_min": None, "err_max": None,
        }
        if self.debug:
            self._debug_banner()

        try:
            serial_connection = self._open_serial()

            if self.service_wait > 0:
                wait_for_vision_service(self.service_wait, self.vision_url)

            while result is None:
                now = time.monotonic()
                if (
                    controller.approach_start_time is not None
                    and now - controller.approach_start_time >= approach_timeout
                ):
                    controller.stop("TIMEOUT")
                    self._stop_once(serial_connection)
                    result = self._result(
                        "TIMEOUT", started_at, last_target, final_state=STOPPED
                    )
                    break

                if runtime_config is None or now >= next_config_refresh:
                    try:
                        runtime_config = self._fetch_runtime_config()
                    except ConfigInputError as error:
                        result = self._handle_vision_failure(
                            "VISION_ERROR", error, serial_connection,
                            started_at, last_target,
                        )
                        if result is not None:
                            break
                        continue
                    controller.update_runtime_config(runtime_config)
                    next_config_refresh = now + 1.0 / CONFIG_REFRESH_HZ
                    if self.debug:
                        self._debug_config(runtime_config)

                try:
                    target = self._fetch_selected_target(runtime_config, now)
                except ApiStaleError as error:
                    result = self._handle_vision_failure(
                        "API_STALE", error, serial_connection,
                        started_at, last_target,
                    )
                    if result is not None:
                        break
                    continue
                except VisionInputError as error:
                    result = self._handle_vision_failure(
                        "VISION_ERROR", error, serial_connection,
                        started_at, last_target,
                    )
                    if result is not None:
                        break
                    continue

                if target is not None:
                    last_target = target

                depth_cm = None
                if self.depth_cm_fn is not None:
                    depth_cm = self._read_depth_cm()
                self._last_depth_cm = depth_cm
                step = controller.step(target, now, depth_cm)

                command_time = time.monotonic()
                if (
                    controller.approach_start_time is not None
                    and command_time - controller.approach_start_time
                    >= approach_timeout
                ):
                    controller.stop("TIMEOUT")
                    self._stop_once(serial_connection)
                    result = self._result(
                        "TIMEOUT", started_at, last_target, final_state=STOPPED
                    )
                    break

                self._write_command(serial_connection, step.command)
                if self.debug:
                    frame_history.append(
                        self._debug_frame(step, target, now - started_at)
                    )
                    if len(frame_history) > DEBUG_FRAME_HISTORY:
                        del frame_history[0]
                    _debug_track(stats, target)

                if step.terminal_reason is not None:
                    result = self._result(
                        step.terminal_reason,
                        started_at,
                        last_target,
                        final_state=step.state,
                    )
                    break

                next_loop += 1.0 / CONTROL_HZ
                sleep_time = next_loop - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_loop = time.monotonic()

        except KeyboardInterrupt:
            try:
                self._stop_once(serial_connection)
            except SerialControlError:
                pass
            result = self._result(
                "INTERRUPTED", started_at, last_target, "Ctrl+C received"
            )
        except SerialControlError as error:
            result = self._result(
                "SERIAL_ERROR", started_at, last_target, str(error), ERROR
            )
        finally:
            stop_error = self._stop_repeatedly(serial_connection)
            if serial_connection is not None:
                try:
                    serial_connection.close()
                except Exception as error:
                    if stop_error is None:
                        stop_error = "failed to close serial: {}".format(error)
            if stop_error is not None and (
                result is None or result.reason != "SERIAL_ERROR"
            ):
                result = self._result(
                    "SERIAL_ERROR", started_at, last_target, stop_error, ERROR
                )

        if self.debug:
            self._debug_stop(
                result, last_target, frame_history, stats,
                runtime_config, started_at,
            )

        if result is None:
            return self._result(
                "VISION_ERROR", started_at, last_target,
                "positioning loop ended without a result", ERROR,
            )
        return result


def visual_block_position(
    color: Optional[str] = None,
    timeout: float = VISUAL_APPROACH_TIMEOUT_S,
    dry_run: bool = False,
    depth_cm_fn: Any = None,
) -> VisualPositionResult:
    """Convenience entry point for one complete positioning action."""
    return VisualBlockPositioner(
        color=color,
        timeout=timeout,
        dry_run=dry_run,
        depth_cm_fn=depth_cm_fn,
    ).run()


__all__ = [
    "MotionCommand",
    "VisionServiceUnavailable",
    "VisualBlockPositioner",
    "VisualPositionController",
    "VisualPositionResult",
    "bcc",
    "build_speed_frame",
    "compute_align_command",
    "compute_approach_command",
    "compute_approach_speed",
    "compute_approach_speed_cm",
    "require_vision_service",
    "vision_service_status",
    "visual_block_position",
    "wait_for_vision_service",
]


# ============================================================
# 视觉定位：靠近方块并对正，成功返回 True
# ============================================================
def find_block1(color=BLOCK_COLOR1):
    """使用正式视觉封装完成方块定位，返回 True/False。"""
    result = VisualBlockPositioner(
        color=color, debug=True, depth_cm_fn=get_distance
    ).run()
    print(
        "[POS] 定位结果 success={} reason={} state={} cx={} err={} bottom_y={} "
        "d_cm={} 耗时={:.2f}s".format(
            result.success, result.reason, result.final_state,
            result.final_cx, result.final_error_x, result.final_bottom_y,
            result.final_depth_cm,
            result.elapsed if result.elapsed is not None else -1.0,
        )
    )
    if not result.success:
        print(
            "视觉定位失败: "
            f"reason={result.reason}, detail={result.detail}, "
            f"final_state={result.final_state}"
        )
        return False
    return True


def find_block2(color=BLOCK_COLOR2):
    """使用正式视觉封装完成方块定位，返回 True/False。"""
    result = VisualBlockPositioner(
        color=color, debug=True, depth_cm_fn=get_distance
    ).run()
    print(
        "[POS] 定位结果 success={} reason={} state={} cx={} err={} bottom_y={} "
        "d_cm={} 耗时={:.2f}s".format(
            result.success, result.reason, result.final_state,
            result.final_cx, result.final_error_x, result.final_bottom_y,
            result.final_depth_cm,
            result.elapsed if result.elapsed is not None else -1.0,
        )
    )
    if not result.success:
        print(
            "视觉定位失败: "
            f"reason={result.reason}, detail={result.detail}, "
            f"final_state={result.final_state}"
        )
        return False
    return True


#----------------横向（旧版：固定速度搜索）-----------------------
def _lat_search_until_found(
    color: str,
    direction: str,
    speed: int,
    timeout_s: float,
    debug: bool = True,
) -> bool:
    if direction not in {"left", "right"}:
        raise ValueError("direction must be 'left' or 'right'")
    if speed <= 0:
        raise ValueError("speed must be positive")
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive")

    vy = int(speed) if direction == "left" else -int(speed)
    started_at = time.monotonic()
    next_loop = started_at
    serial_connection = None
    found = False

    if not wait_for_vision_service(timeout_s=min(SERVICE_WAIT_S, timeout_s)):
        print("[POS] 视觉服务未就绪，放弃搜索")
        return False

    try:
        import serial
        serial_connection = serial.Serial(
            serial_ports.chassis_port(),
            BAUDRATE,
            timeout=0.1,
            write_timeout=0.2,
        )
    except Exception as error:
        raise SerialControlError(
            "could not open chassis serial for search: {}".format(error)
        ) from error

    if debug:
        print(
            "[POS] 向{}搜索 {} 方块，vy={}，超时 {:.1f}s".format(
                "左" if direction == "left" else "右",
                color, vy, timeout_s,
            )
        )

    try:
        while True:
            elapsed = time.monotonic() - started_at
            if elapsed >= timeout_s:
                if debug:
                    print(
                        "[POS] {:.2f}s 内没找到 {} 方块，停止搜索".format(
                            timeout_s, color
                        )
                    )
                return False

            try:
                serial_connection.write(build_speed_frame(0, vy, 0))
            except Exception as error:
                raise SerialControlError(
                    "serial write failed during search: {}".format(error)
                ) from error

            try:
                payload = _fetch_targets_payload()
                if _color_visible(payload, color):
                    found = True
                    if debug:
                        print(
                            "[POS] {:.2f}s 找到 {} 方块，停止横移".format(
                                elapsed, color
                            )
                        )
                    break
            except (
                HTTPError, URLError, TimeoutError, OSError,
                json.JSONDecodeError, UnicodeDecodeError,
            ):
                pass

            next_loop += 1.0 / CONTROL_HZ
            sleep_time = next_loop - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_loop = time.monotonic()
    finally:
        try:
            stop_frame = build_speed_frame(0, 0, 0)
            for _ in range(STOP_FRAME_COUNT):
                serial_connection.write(stop_frame)
                time.sleep(STOP_INTERVAL_S)
        except Exception:
            pass
        try:
            serial_connection.close()
        except Exception:
            pass

    return found


def _until_find_block(
    color: str, direction: str, speed: int, timeout: float,
) -> bool:
    if not _lat_search_until_found(color, direction, speed, timeout):
        return False

    result = VisualBlockPositioner(
        color=color, debug=True, depth_cm_fn=get_distance
    ).run()

    print(
        "[POS] 定位结果 success={} reason={} state={} cx={} err={} bottom_y={} "
        "d_cm={} 耗时={:.2f}s".format(
            result.success, result.reason, result.final_state,
            result.final_cx, result.final_error_x, result.final_bottom_y,
            result.final_depth_cm,
            result.elapsed if result.elapsed is not None else -1.0,
        )
    )
    if not result.success:
        print(
            "视觉定位失败: "
            f"reason={result.reason}, detail={result.detail}, "
            f"final_state={result.final_state}"
        )
        return False
    return True


def left_until_find_block1(
    color: str = BLOCK_COLOR1,
    speed: int = 60,
    timeout: float = 30.0,
) -> bool:
    return _until_find_block(color, "left", speed, timeout)


def right_until_find_block1(
    color: str = BLOCK_COLOR1,
    speed: int = 60,
    timeout: float = 30.0,
) -> bool:
    return _until_find_block(color, "right", speed, timeout)


def left_until_find_block2(
    color: str = BLOCK_COLOR2,
    speed: int = 60,
    timeout: float = 30.0,
) -> bool:
    return _until_find_block(color, "left", speed, timeout)


def right_until_find_block2(
    color: str = BLOCK_COLOR2,
    speed: int = 60,
    timeout: float = 30.0,
) -> bool:
    return _until_find_block(color, "right", speed, timeout)

#----------------------------------------------------------


# ============================================================
# 三档速度版搜索 + 对齐（新增，不改原有函数）
#
#   一开始就看到方块      → 跳过 SEARCH，直接按 error_x 分档
#   看到目标 |error_x| 大 → fast_speed 快速靠近
#   看到目标 |error_x| 小 → slow_speed 慢速微调
#   没看到目标            → search_speed 沿初始方向搜索
#   进入 error_slow 范围  → 停下，交给 VisualBlockPositioner 做靠近
# ============================================================

def _find_color_target_pro(
    payload: Any, color: str
) -> Optional[Dict[str, Any]]:
    if not isinstance(payload, dict) or payload.get("ok") is not True:
        return None
    targets = payload.get("targets")
    if not isinstance(targets, list):
        return None
    best: Optional[Dict[str, Any]] = None
    best_abs: Optional[float] = None
    for target in targets:
        if not isinstance(target, dict) or target.get("color") != color:
            continue
        err = target.get("error_x")
        if not _is_finite_number(err):
            continue
        magnitude = abs(float(err))
        if best_abs is None or magnitude < best_abs:
            best = target
            best_abs = magnitude
    return best


def _lat_search_until_found_pro(
    color: str,
    direction: str,
    search_speed: int = 600,                    # ← ① 没看到方块时的速度
    fast_speed: int = 500,           # ← ② 误差较大时（快速靠近）
    slow_speed: int = 40,           # ← ③ 误差较小时（慢速微调）
    error_fast: int = ALIGN_SLOW_THRESHOLD,    # 快/慢分界（像素）
    error_slow: int = ALIGN_DEADBAND,          # 慢/停分界（像素）
    timeout_s: float = 30.0,
    debug: bool = True,
) -> bool:
    """横向搜索 + 对齐，三档速度版本（带首帧预检）。

    速度分档：
        一开始就看到方块       → 直接按 error_x 走 FAST / SLOW / ALIGNED
                                （不发 search_speed 那一帧）
        后续帧看到目标          → 同上，逐帧按 error_x 分档
        未看到目标              → search_speed（沿 direction 初始方向）
        |error_x| >= error_fast → fast_speed（朝目标方向快速靠近）
        |error_x| >  error_slow → slow_speed（朝目标方向慢速微调）
        |error_x| <= error_slow → vy=0，返回 True（交给定位器做靠近）
    """
    if direction not in {"left", "right"}:
        raise ValueError("direction must be 'left' or 'right'")
    for name, value in (
        ("search_speed", search_speed),
        ("fast_speed", fast_speed),
        ("slow_speed", slow_speed),
    ):
        if value <= 0:
            raise ValueError("{} must be positive".format(name))
    if error_slow >= error_fast:
        raise ValueError("error_slow must be smaller than error_fast")
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive")

    search_vy = int(search_speed) if direction == "left" else -int(search_speed)

    started_at = time.monotonic()
    next_loop = started_at
    serial_connection = None
    found = False

    if not wait_for_vision_service(timeout_s=min(SERVICE_WAIT_S, timeout_s)):
        print("[POS] 视觉服务未就绪，放弃搜索")
        return False

    try:
        import serial
        serial_connection = serial.Serial(
            serial_ports.chassis_port(),
            BAUDRATE,
            timeout=0.1,
            write_timeout=0.2,
        )
    except Exception as error:
        raise SerialControlError(
            "could not open chassis serial for search: {}".format(error)
        ) from error

    if debug:
        print(
            "[POS-PRO] 向{}搜索 {} 方块 | search={} fast={} slow={} "
            "快/慢分界={} 慢/停分界={} 超时 {:.1f}s".format(
                "左" if direction == "left" else "右",
                color, search_speed, fast_speed, slow_speed,
                error_fast, error_slow, timeout_s,
            )
        )

    # 首帧预检：已经看到方块就跳过 SEARCH
    first_frame_target: Optional[Dict[str, Any]] = None
    try:
        payload = _fetch_targets_payload()
        first_frame_target = _find_color_target_pro(payload, color)
    except (
        HTTPError, URLError, TimeoutError, OSError,
        json.JSONDecodeError, UnicodeDecodeError,
    ):
        first_frame_target = None

    precheck_target = first_frame_target

    try:
        while True:
            elapsed = time.monotonic() - started_at
            if elapsed >= timeout_s:
                if debug:
                    print(
                        "[POS-PRO] {:.2f}s 内没对准 {} 方块，停止搜索".format(
                            timeout_s, color
                        )
                    )
                return False

            vy = search_vy
            mode = "SEARCH"
            target = None

            if precheck_target is not None:
                target = precheck_target
                precheck_target = None
            else:
                try:
                    payload = _fetch_targets_payload()
                    target = _find_color_target_pro(payload, color)
                except (
                    HTTPError, URLError, TimeoutError, OSError,
                    json.JSONDecodeError, UnicodeDecodeError,
                ):
                    target = None

            if target is not None:
                err_value = target.get("error_x")
                if _is_finite_number(err_value):
                    err = float(err_value)
                    magnitude = abs(err)
                    if magnitude <= error_slow:
                        vy = 0
                        mode = "ALIGNED"
                    elif magnitude >= error_fast:
                        vy = int(fast_speed) if err < 0 else -int(fast_speed)
                        mode = "FAST"
                    else:
                        vy = int(slow_speed) if err < 0 else -int(slow_speed)
                        mode = "SLOW"

            try:
                serial_connection.write(build_speed_frame(0, vy, 0))
            except Exception as error:
                raise SerialControlError(
                    "serial write failed during search: {}".format(error)
                ) from error

            if debug:
                if target is None:
                    print(
                        "[POS-PRO] {:6.2f}s {:8s} vy={:+d}".format(
                            elapsed, mode, vy
                        )
                    )
                else:
                    print(
                        "[POS-PRO] {:6.2f}s {:8s} vy={:+d} | err={} cx={} "
                        "state={}".format(
                            elapsed, mode, vy,
                            _fmt(_target_number(target, "error_x")),
                            _fmt(_target_number(target, "cx")),
                            target.get("state"),
                        )
                    )

            if mode == "ALIGNED":
                found = True
                if debug:
                    print(
                        "[POS-PRO] {:.2f}s 已对准 {} 方块 -> 交给定位器做靠近".format(
                            elapsed, color
                        )
                    )
                break

            next_loop += 1.0 / CONTROL_HZ
            sleep_time = next_loop - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_loop = time.monotonic()
    finally:
        try:
            stop_frame = build_speed_frame(0, 0, 0)
            for _ in range(STOP_FRAME_COUNT):
                serial_connection.write(stop_frame)
                time.sleep(STOP_INTERVAL_S)
        except Exception:
            pass
        try:
            serial_connection.close()
        except Exception:
            pass

    return found


def _until_find_block_pro(
    color: str,
    direction: str,
    search_speed: int = 60,
    fast_speed: int = ALIGN_VY_FAST,
    slow_speed: int = ALIGN_VY_SLOW,
    error_fast: int = ALIGN_SLOW_THRESHOLD,
    error_slow: int = ALIGN_DEADBAND,
    timeout: float = 30.0,
) -> bool:
    if not _lat_search_until_found_pro(
        color, direction,
        search_speed=search_speed,
        fast_speed=fast_speed,
        slow_speed=slow_speed,
        error_fast=error_fast,
        error_slow=error_slow,
        timeout_s=timeout,
    ):
        return False

    result = VisualBlockPositioner(
        color=color, debug=True, depth_cm_fn=get_distance
    ).run()

    print(
        "[POS-PRO] 定位结果 success={} reason={} state={} cx={} err={} "
        "bottom_y={} d_cm={} 耗时={:.2f}s".format(
            result.success, result.reason, result.final_state,
            result.final_cx, result.final_error_x, result.final_bottom_y,
            result.final_depth_cm,
            result.elapsed if result.elapsed is not None else -1.0,
        )
    )
    if not result.success:
        print(
            "视觉定位失败: "
            f"reason={result.reason}, detail={result.detail}, "
            f"final_state={result.final_state}"
        )
        return False
    return True


def left_until_find_block1_pro(
    color: str = BLOCK_COLOR1,
    search_speed: int = 60,
    fast_speed: int = ALIGN_VY_FAST,
    slow_speed: int = ALIGN_VY_SLOW,
    error_fast: int = ALIGN_SLOW_THRESHOLD,
    error_slow: int = ALIGN_DEADBAND,
    timeout: float = 30.0,
) -> bool:
    return _until_find_block_pro(
        color, "left",
        search_speed=search_speed,
        fast_speed=fast_speed,
        slow_speed=slow_speed,
        error_fast=error_fast,
        error_slow=error_slow,
        timeout=timeout,
    )


def right_until_find_block1_pro(
    color: str = BLOCK_COLOR1,
    search_speed: int = 60,
    fast_speed: int = ALIGN_VY_FAST,
    slow_speed: int = ALIGN_VY_SLOW,
    error_fast: int = ALIGN_SLOW_THRESHOLD,
    error_slow: int = ALIGN_DEADBAND,
    timeout: float = 30.0,
) -> bool:
    return _until_find_block_pro(
        color, "right",
        search_speed=search_speed,
        fast_speed=fast_speed,
        slow_speed=slow_speed,
        error_fast=error_fast,
        error_slow=error_slow,
        timeout=timeout,
    )


def left_until_find_block2_pro(
    color: str = BLOCK_COLOR2,
    search_speed: int = 60,
    fast_speed: int = ALIGN_VY_FAST,
    slow_speed: int = ALIGN_VY_SLOW,
    error_fast: int = ALIGN_SLOW_THRESHOLD,
    error_slow: int = ALIGN_DEADBAND,
    timeout: float = 30.0,
) -> bool:
    return _until_find_block_pro(
        color, "left",
        search_speed=search_speed,
        fast_speed=fast_speed,
        slow_speed=slow_speed,
        error_fast=error_fast,
        error_slow=error_slow,
        timeout=timeout,
    )


def right_until_find_block2_pro(
    color: str = BLOCK_COLOR2,
    search_speed: int = 60,
    fast_speed: int = ALIGN_VY_FAST,
    slow_speed: int = ALIGN_VY_SLOW,
    error_fast: int = ALIGN_SLOW_THRESHOLD,
    error_slow: int = ALIGN_DEADBAND,
    timeout: float = 30.0,
) -> bool:
    return _until_find_block_pro(
        color, "right",
        search_speed=search_speed,
        fast_speed=fast_speed,
        slow_speed=slow_speed,
        error_fast=error_fast,
        error_slow=error_slow,
        timeout=timeout,
    )

#----------------------------------------------------------


# ============================================================
# 主入口
#
# 默认动作：向右找方块（三档速度 + 首帧预检 + 内部接测距）。
# --check / --wait 只查视觉服务，不动车。
# ============================================================
if __name__ == "__main__":

    right_until_find_block1_pro(search_speed=600,fast_speed=200,slow_speed=40)   # ← 改这里

import lgpio
import threading
import time

TRIG = 4
ECHO = 3

# GPIO 按需申请，不要在模块顶层申请。
# 顶层申请意味着任何一句 import distance 都会独占 GPIO12/1，而 gpiod 的线句柄是
# 独占的：只要还有别的进程持有它们（哪怕那个进程已经卡死不会退出），后续脚本在
# import 阶段就会直接 "GPIO busy" 起不来。放到第一次真正测距时再申请，导入就不再
# 是抢资源的时机。
_lock = threading.Lock()
_handle = None


def _open():
    """申请 TRIG/ECHO。失败时回滚 chip 句柄并保持未初始化，下次调用会重试。"""
    global _handle
    h = lgpio.gpiochip_open(4)
    try:
        lgpio.gpio_claim_output(h, TRIG)
        lgpio.gpio_claim_input(h, ECHO)
    except Exception:
        lgpio.gpiochip_close(h)   # 别把 chip 句柄漏在失败路径上
        raise
    _handle = h
    return h


def get_distance():
    # 持锁跑完整次测量：两条 while 之间不能让别的线程插进来发脉冲，
    # 否则量到的是别人的回波。
    with _lock:
        h = _handle if _handle is not None else _open()

        # 发送 10us 脉冲
        lgpio.gpio_write(h, TRIG, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(h, TRIG, 0)

        # 等待回波
        timeout = time.time() + 0.03
        while lgpio.gpio_read(h, ECHO) == 0:
            if time.time() > timeout:
                return None
        t_start = time.time()

        while lgpio.gpio_read(h, ECHO) == 1:
            if time.time() > timeout:
                return None
        t_end = time.time()

        duration = t_end - t_start
        distance = duration * 34300 / 2
        return distance


FAR_THRESHOLD_CM = 62.0
FAR_CONFIRM_FRAMES = 3
FRAME_GAP_S = 0.06


def is_far_62cm(threshold_cm=FAR_THRESHOLD_CM, frames=FAR_CONFIRM_FRAMES,
                frame_gap_s=FRAME_GAP_S):
    """连续 frames 帧测距都大于 threshold_cm 返回 1，否则返回 0。

    读不到（None）不算"远"：没测出来的一帧不能拿去确认，传感器掉线也不会
    被当成前方空旷。
    帧间隔不能省：前一次发射的余振会干扰后一次，背靠背连测会让后两帧读到
    同一个回波，3 帧确认就变成假的。
    """
    for index in range(frames):
        if index:
            time.sleep(frame_gap_s)
        distance = get_distance()
        if distance is None or distance <= threshold_cm:
            return 0
    return 1


def release_distance():
    """释放 GPIO12/1。mission 脚本的 finally 里调一下，退出时（尤其是被 kill 的
    情况下）把线让出来，避免留下残留占用把后续所有脚本都挡在 GPIO busy 上。"""
    global _handle
    with _lock:
        if _handle is not None:
            lgpio.gpiochip_close(_handle)
            _handle = None


if __name__ == "__main__":
    try:
        while True:
            d = get_distance()
            if d:
                print(f"距离: {d:.1f} cm")
            else:
                print("测量失败")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        release_distance()

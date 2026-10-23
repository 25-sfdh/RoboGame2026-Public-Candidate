"完整夹取两层六个（共十二个）"
import trace_car_v2 as car
import trace_car_v1 as car1
from set_angle import (
    set_angle,
    set_fixed_angle,
    release_servo,
    set_angle_012_0,
    set_angle_12_0,
    set_angle_012_1,
    set_angle_12_1,
    get_angle,
    set_angle_with_feedback,
)
import time
import sys
import serial
import threading
import test_comm
import traceback
from distance import get_distance
from visual_block_positioning import (
    VisualBlockPositioner,
    VisionServiceUnavailable,
    require_vision_service,
    find_block1,
    find_block2,
    left_until_find_block1_pro,
    left_until_find_block2_pro,
    right_until_find_block1_pro,
    right_until_find_block2_pro,
)
from Tuigan import tuigan, tuigan_release


# ============================================================
# 三个任务的 car.run 主速度
# ============================================================
RUN_SPEED_FROM_BEGIN = 600   # 任务1：从启动区抓三个橙色
RUN_SPEED_FROM_SET   = 600   # 任务2：从搭建区抓三个橙色
RUN_SPEED_PURPLE     = 500   # 任务3：从搭建区抓一紫两橙


# ============================================================
# 全局角度定义
# ============================================================
angle_1  = -1910.9          # 高处橙色方块第一个
angle_2  = angle_1 - 165.4  # 高处橙色第二个
angle_3  = angle_1 - 238.4  # 高处橙色识别
angle_4  = angle_1 + 103.9  # 低处方块识别
angle_5  = angle_1 + 338    # 低处方块第一个
angle_6  = angle_1 + 178.1  # 低处方块第二个
angle_7  = angle_1 + 246.4  # 搭建区下面有一个
angle_8  = angle_1 + 415  # 搭建区放置最低
angle_9  = angle_1 - 91.7   # 搭建区下面有三个
angle_10 = angle_1 - 434.4  # 搭建区下面有五个


# ============================================================
# 线程安全包装：保护舵机串口，收集子线程异常
# ============================================================
_servo_lock = threading.Lock()
_thread_errors = []


def safe_set_angle(*args, **kwargs):
    """加锁调用 set_angle，避免多线程同时写舵机串口。"""
    with _servo_lock:
        return set_angle(*args, **kwargs)


def safe_set_fixed_angle(*args, **kwargs):
    """加锁调用 set_fixed_angle，避免多线程同时写舵机串口。"""
    with _servo_lock:
        return set_fixed_angle(*args, **kwargs)


def safe_set_angle_with_feedback(*args, **kwargs):
    """加锁调用 set_angle_with_feedback，避免多线程同时写舵机串口。"""
    with _servo_lock:
        return set_angle_with_feedback(*args, **kwargs)


def set_a3(target_angle, tolerance=15, velocity=240.0,
           adjust_sleep_time=0.8, safety_factor=1.0, min_sleep=0.2,
           max_retries=10, **kwargs):
    """
    ID=3 专用舵机控制封装：
      - 第一次：读当前角度，按 t = |Δ角度| / velocity 线性估算等待时间；
      - 之后：若 |当前 - 目标| > tolerance，用 adjust_sleep_time 重发命令微调，
              直到偏差 <= tolerance 或达到 max_retries。
    其他 power / t_acc / t_dec / lock_wait 通过 kwargs 透传。
    """
    return safe_set_angle_with_feedback(
        3,
        target_angle,
        tolerance=tolerance,
        velocity=velocity,
        adjust_sleep_time=adjust_sleep_time,
        safety_factor=safety_factor,
        min_sleep=min_sleep,
        max_retries=max_retries,
        **kwargs,
    )


def thread_wrapper(func, *args, **kwargs):
    """子线程入口：捕获异常并记录，避免 join 后主线程完全无感。"""
    try:
        return func(*args, **kwargs)
    except Exception as e:
        _thread_errors.append(e)
        print(f"[THREAD ERROR] {getattr(func, '__name__', func)}: {e}")
        traceback.print_exc()


# ============================================================
# 任务1：从启动区抓三个橙色（原 grip_three_orange_not_turn.py）
# ============================================================
def from_begin_servo_action_0():
    print("[SERVO][FROM_BEGIN] 动作 0 开始")

    def angle_begin():
        set_a3(
            angle_3,                     # 原 angle_0，识别角度
            velocity=220.0,
            power=60000,
            t_acc=300,
            t_dec=300,
            lock_wait=0.3,
        )

    t1 = threading.Thread(target=thread_wrapper, args=(angle_begin,))
    t2 = threading.Thread(target=thread_wrapper, args=(set_angle_012_0,))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    print("[SERVO][FROM_BEGIN] 动作 0 完成")


def from_begin_servo_action_1():
    print("[SERVO][FROM_BEGIN] 动作 1 开始")
    set_a3(angle_1)                       # 原 angle_1，高处橙色第一个
    safe_set_fixed_angle(2, 1)
    set_a3(angle_3)                       # 原 angle_0，识别角度
    print("[SERVO][FROM_BEGIN] 动作 1 完成")


def from_begin_servo_action_2():
    print("[SERVO][FROM_BEGIN] 动作 2 开始")
    set_a3(angle_2)                       # 原 angle_2，高处橙色第二个
    safe_set_fixed_angle(2, 0)
    set_a3(angle_1)                       # 原 angle_1，高处橙色第一个

    set_angle_12_1()

    set_a3(angle_3)                       # 原 angle_0，识别角度
    print("[SERVO][FROM_BEGIN] 动作 2 完成")


def from_begin_servo_action_3():
    print("[SERVO][FROM_BEGIN] 动作 3 开始")
    set_a3(angle_2)                       # 原 angle_2，高处橙色第二个

    set_angle_12_0()

    set_a3(angle_1)                       # 原 angle_1，高处橙色第一个

    set_angle_012_1()

    set_a3(angle_3)                       # 原 angle_0，识别角度
    print("[SERVO][FROM_BEGIN] 动作 3 完成")


def from_begin_servo_action_4():
    set_a3(angle_8)                       # 原 angle_put，放置角度
    print("[SERVO][FROM_BEGIN] 动作 4 开始")
    set_angle_012_0()
    print("[SERVO][FROM_BEGIN] 动作 4 完成")


def task_from_begin_main():
    """任务1主流程：从启动区抓三个橙色并放置到搭建区。"""
    try:
        try:
            require_vision_service(timeout_s=20)
        except VisionServiceUnavailable as error:
            print(f"不发车：{error}")
            sys.exit(2)

        def trace_start_from_begin():
            car.car_run_special(
                [("T", "stop")],
                speed=RUN_SPEED_FROM_BEGIN,
                forward_speed=280,
                forward_time=0.1,
                right_speed=300,
                right_time=0.2,
            )

        # 并行：舵机初始化 和小车开始跑
        t1 = threading.Thread(target=thread_wrapper, args=(from_begin_servo_action_0,))
        t2 = threading.Thread(target=thread_wrapper, args=(trace_start_from_begin,))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        if not find_block1():
            print("第 1 个方块定位失败")
            sys.exit(2)
        from_begin_servo_action_1()

        car.move_for(0.8, vx=0, vy=-200)

        if not find_block1():
            print("第 2 个方块定位失败")
            sys.exit(2)
        from_begin_servo_action_2()

        car.move_for(0.8, vx=0, vy=-200)

        if not find_block1():
            print("第 3 个方块定位失败")
            sys.exit(2)
        from_begin_servo_action_3()

        def trace_to_set():
            car.move_distance(504,-600)
            car.move_for(1,0,0,-1540)
            car.run(
                [("R", "stop")],
                speed=RUN_SPEED_FROM_BEGIN,
            )

        t1 = threading.Thread(target=thread_wrapper, args=(trace_to_set,))
        t2 = threading.Thread(
            target=thread_wrapper,
            args=(set_a3, angle_8 - 80),
        )
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        car.run_until_distance(target_cm=7, speed=200)
        from_begin_servo_action_4()
    finally:
        print("三个橙色完成")


# ============================================================
# 任务2：从搭建区抓三个橙色（原 grip_three_orange_from_set.py）
# ============================================================
def from_set_servo_action_0():
    print("[SERVO][FROM_SET] 动作 0 开始")
    t1 = threading.Thread(target=thread_wrapper, args=(set_a3, angle_3))  # 原 angle_0
    t2 = threading.Thread(target=thread_wrapper, args=(set_angle_012_0()))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    print("[SERVO][FROM_SET] 动作 0 完成")


def from_set_servo_action_1():
    print("[SERVO][FROM_SET] 动作 1 开始")
    set_a3(angle_1)                       # 原 angle_1
    safe_set_fixed_angle(2, 1)
    set_a3(angle_3)                       # 原 angle_0
    print("[SERVO][FROM_SET] 动作 1 完成")


def from_set_servo_action_2():
    print("[SERVO][FROM_SET] 动作 2 开始")
    set_a3(angle_2)                       # 原 angle_2
    safe_set_fixed_angle(2, 0)
    set_a3(angle_1)                       # 原 angle_1

    set_angle_12_1()

    set_a3(angle_3)                       # 原 angle_0
    print("[SERVO][FROM_SET] 动作 2 完成")


def from_set_servo_action_3():
    print("[SERVO][FROM_SET] 动作 3 开始")
    set_a3(angle_2)                       # 原 angle_2

    set_angle_12_0()

    set_a3(angle_1)                       # 原 angle_1

    set_angle_012_1()

    set_a3(angle_3)                       # 原 angle_0
    print("[SERVO][FROM_SET] 动作 3 完成")


def from_set_servo_action_4():
    set_a3(angle_8)                       # 原 angle_put
    print("[SERVO][FROM_SET] 动作 4 开始")
    set_angle_012_0()
    print("[SERVO][FROM_SET] 动作 4 完成")


def task_from_set_main():
    """任务2主流程：从搭建区抓三个橙色。"""
    try:
        def trace_start_from_set():
            car.move_distance(375,-500)
            car.move_for(1,0,0,-1540)
            car.run(
                [("R", "stop")],
                speed=RUN_SPEED_FROM_SET,
            )
            car.move_for(0.2,0,600)

        # 并行：舵机初始化 和小车开始跑
        t1 = threading.Thread(target=thread_wrapper, args=(trace_start_from_set,))
        t2 = threading.Thread(target=thread_wrapper, args=(from_set_servo_action_0,))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        if not find_block1():
            print("第 1 个方块定位失败")
            sys.exit(2)
        from_set_servo_action_1()

        car.move_for(0.8, vx=0, vy=-200)

        if not find_block1():
            print("第 2 个方块定位失败")
            sys.exit(2)
        from_set_servo_action_2()

        car.move_for(0.8, vx=0, vy=-200)

        if not find_block1():
            print("第 3 个方块定位失败")
            sys.exit(2)
        from_set_servo_action_3()

        def trace_to_set():
            car.move_distance(504,-600)
            car.move_for(1,0,0,-1540)
            car.run(
                [
                    ("R", "stop"),
                ],
                speed=RUN_SPEED_FROM_SET,
            )

        t1 = threading.Thread(target=thread_wrapper, args=(trace_to_set,))
        t2 = threading.Thread(
            target=thread_wrapper,
            args=(set_a3, angle_8 - 100),
        )
        t1.start()
        t2.start()
        t1.join()
        t2.join()
        
        car.move_for(1.4, vx=0, vy=300)
        car.go_until_distance(target_cm=7, speed=300)
        from_set_servo_action_4()
    finally:
        print("三个橙色完成")


# ============================================================
# 任务3：从搭建区抓一紫两橙（原 grip_purple_and_two_orange.py）
# ============================================================
def purple_servo_action_0():
    print("[SERVO][PURPLE] 动作 0 开始")
    set_angle_012_0()
    set_a3(angle_4)                       # 原 angle_0，方块识别
    tuigan(1)
    time.sleep(2)
    print("[SERVO][PURPLE] 动作 0 完成")


def purple_servo_action_1():
    print("[SERVO][PURPLE] 动作 1 开始")
    set_a3(angle_5)                       # 原 angle_1，方块第一个
    safe_set_fixed_angle(2, 1)
    set_a3(angle_4)                       # 原 angle_0，方块识别
    tuigan(2)
    time.sleep(2)
    print("[SERVO][PURPLE] 动作 1 完成")


def purple_servo_action_2():
    print("[SERVO][PURPLE] 动作 2 开始")
    set_a3(angle_6)                       # 原 angle_2，方块第二个
    safe_set_fixed_angle(2, 0)
    set_a3(angle_5)                       # 原 angle_1，方块第一个
    set_angle_12_1()
    set_a3(angle_4)                       # 原 angle_0，方块识别
    print("[SERVO][PURPLE] 动作 2 完成")


def purple_servo_action_3():
    print("[SERVO][PURPLE] 动作 3 开始")
    set_a3(angle_6)                       # 原 angle_2，方块第二个
    set_angle_12_0()
    set_a3(angle_5)                       # 原 angle_1，方块第一个
    set_angle_012_1()
    set_a3(angle_4)                       # 原 angle_0，方块识别
    print("[SERVO][PURPLE] 动作 3 完成")


def purple_servo_action_4():
    print("[SERVO][PURPLE] 动作 4 开始")
    set_a3(angle_9)                       # 放置方块
    set_angle_012_0()
    print("[SERVO][PURPLE] 动作 4 完成")


def task_purple_main():
    """任务3主流程：从搭建区开始抓一紫两橙。"""
    try:
        #从已经在线上开始后退
        def trace_from_set():
            car.run([("R", "stop")], speed=-500, advance_time=0.9)
            #car.run_until_distance(target_cm=15, speed=400)

        t1 = threading.Thread(target=thread_wrapper, args=(purple_servo_action_0,))
        t2 = threading.Thread(target=thread_wrapper, args=(trace_from_set,))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        if not find_block2():
            print("第 1 个方块定位失败")
            sys.exit(2)
        purple_servo_action_1()

        car.move_distance(750,-500)
        car.move_for(1,0,0,-1540)
        car.run_until_distance(target_cm=20, speed=200)

        if not find_block1():
            print("第 2 个方块定位失败")
            sys.exit(2)
        purple_servo_action_2()

        car.move_for(0.8, vx=0, vy=-200)

        if not find_block1():
            print("第 3 个方块定位失败")
            sys.exit(2)
        purple_servo_action_3()

        def back_task():
            car.shift_until_line("L", shift_speed=300, after="stop")
            car.run([("T", "stop"),("B", "stop")], 
                    speed=-RUN_SPEED_PURPLE)

        t1 = threading.Thread(target=thread_wrapper, args=(back_task,))
        t2 = threading.Thread(
            target=thread_wrapper,
            args=(set_a3, angle_9 - 80),
        )
        t1.start()
        t2.start()
        t1.join()
        t2.join()
        
    finally:
        print("一紫两橙抓取完成")
        
#---------------------------------------------------   
def task_purple_main_2():
    """任务3主流程：从搭建区开始抓一紫两橙。"""
    try:
        #从已经在线上开始后退
        def trace_from_set():
            car.run([("R", "stop")], speed=-RUN_SPEED_PURPLE, advance_time=0.6)
            car.run_until_distance(target_cm=15, speed=400)

        t1 = threading.Thread(target=thread_wrapper, args=(purple_servo_action_0,))
        t2 = threading.Thread(target=thread_wrapper, args=(trace_from_set,))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        if not find_block2():
            print("第 1 个方块定位失败")
            sys.exit(2)
        purple_servo_action_1()

        car.move_distance(750,-500)
        car.move_for(1,0,0,-1540)
        car.run_until_distance(target_cm=30, speed=200)

        if not find_block1():
            print("第 2 个方块定位失败")
            sys.exit(2)
        purple_servo_action_2()

        car.move_for(0.8, vx=0, vy=200)

        if not find_block1():
            print("第 3 个方块定位失败")
            sys.exit(2)
        purple_servo_action_3()

        def back_task():
            car.shift_until_line("R", shift_speed=300, after="stop")
            car.run([("T", "continue"),
                    ("B", "stop")], 
                    speed=-RUN_SPEED_PURPLE)

        t1 = threading.Thread(target=thread_wrapper, args=(back_task,))
        t2 = threading.Thread(
            target=thread_wrapper,
            args=(set_a3, angle_9 - 50),
        )
        t1.start()
        t2.start()
        t1.join()
        t2.join()
        
    finally:
        print("一紫两橙抓取完成")


# ============================================================
# 主流程舵机动作
# ============================================================
def purple_set_angle():
    if not find_block1():
        print("三个橙色方块定位失败")
        sys.exit(2)
    set_a3(angle_9)
    set_angle_012_0()


def main():
    try:
        task_from_begin_main()  #从启动区抓三橙
        
        car.move_distance(860, vx=-500)
        task_purple_main() #抓一紫两橙
        
        #搭建
        car.move_distance(200, vx=300)
        purple_set_angle()
        
        task_from_set_main() #从搭建区抓三橙
        
        car.move_distance(900, vx=-500)
        car.shift_until_line("R",300,"stop")
        
        task_purple_main_2() #抓一紫两橙
        
        #搭建
        car.move_for(1.4, vx=0, vy=300)
        car.move_distance(200, vx=300)
        purple_set_angle()
        
        #结束动作
        car.move_distance(240, vx=-300)
        car.shift_until_line("R",300,"stop")

    finally:
        release_servo()
        tuigan_release()
        print("[SERVO] 已卸力并关串口")


# 主入口
if __name__ == "__main__":
    main()
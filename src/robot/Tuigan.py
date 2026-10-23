#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树莓派 5（Raspberry Pi 5）GPIO 9 / 10 / 11 三状态切换

管脚用 BCM 编号（gpiozero 用的就是这个编号），对应 40Pin 排针的物理脚：
    GPIO 9  -> 物理脚 21
    GPIO 10 -> 物理脚 19
    GPIO 11 -> 物理脚 23

状态表（数值顺序固定为 GPIO9, GPIO10, GPIO11）：
    状态 1 -> 1 0 0
    状态 2 -> 0 0 1
    状态 3 -> 1 1 1   ← 默认状态，不带参数运行时就是它

只有这三个状态，脚本从头到尾都不会把三个脚一起拉低成 0 0 0（连建脚和退出时也不会）。

用法：
    python3 gpio_9_10_11_3states.py                # 不带参数 = 默认状态 3 (1 1 1)，并保持
    python3 gpio_9_10_11_3states.py list           # 只看状态表，不碰硬件
    python3 gpio_9_10_11_3states.py 1              # 切到状态 1 并保持（Ctrl+C 退出时回到 1 1 1）
    python3 gpio_9_10_11_3states.py 2 --hold 3     # 切到状态 2，保持 3 秒后回到 1 1 1 退出
    python3 gpio_9_10_11_3states.py cycle -t 0.5   # 1->2->3 循环，每步 0.5 秒
    python3 gpio_9_10_11_3states.py cycle -t 1 -n 9  # 循环 9 步就停
    python3 gpio_9_10_11_3states.py 1 --dry-run    # 不接硬件，只打印，先在电脑上验证逻辑

退出行为：脚本结束时把三个脚恢复成默认状态 1 1 1，并调用 pinctrl 把电平固定住，这样
    进程退出后（在断电重启或手动改之前）电平仍然是 1 1 1，不会变成悬空输入。

接线提醒：
  * 三个 GPIO 都是 3.3V 电平，只能接 3.3V 逻辑的输入。要驱动继电器、LED 等负载时，
    每个脚串一个限流电阻（LED 一般 220Ω~1kΩ），感性负载加续流二极管并共地。
  * 一定要和树莓派共地（GND 接排针 6/9/14/20/25/30/34/39 任一）。
  * 树莓派的单个 GPIO 建议不超过 16mA，三个脚同时输出也别超总电流限制。

注意：
  * 树莓派 5 上 RPi.GPIO、wiringPi、sysfs 都不好使了，必须走 gpiozero（lgpio 后端）
    或 python3-gpiod，本脚本用 gpiozero，树莓派 OS Bookworm 默认已装。
    如果提示缺库：sudo apt install python3-gpiozero python3-lgpio
  * GPIO 9 / 10 / 11 就是 SPI0 的 MISO / MOSI / SCLK。如果系统里开了 SPI
    （/boot/firmware/config.txt 里有 dtparam=spi=on），这几个脚会被 SPI 驱动占用，
    表现是电平拉不动或报 "GPIO busy"。用之前先关掉：
        sudo raspi-config  ->  Interface Options  ->  SPI  ->  No  ->  重启
    或者把 config.txt 里的 dtparam=spi=on 注释掉后重启。
  * 同理，如果用了别的占用这几个脚的设备树覆盖层（比如某些屏幕、HAT），也要先关掉。
"""
import argparse
import shutil
import subprocess
import sys
import time

# ===== 配置区 =====
PINS = (9, 10, 11)           # 3 个 GPIO 的 BCM 编号，数组顺序 = 状态表里的位序
STATES = {
    "1": (1, 0, 0),
    "2": (0, 0, 1),
    "3": (1, 1, 1),
}
DEFAULT_STATE = "3"          # 不带参数运行时切到的状态，也就是 (1, 1, 1)
CYCLE_ORDER = ("1", "2", "3")  # cycle 模式的状态循环顺序
PIN_HEADER = {9: 21, 10: 19, 11: 23}  # BCM 编号 -> 排针物理脚，仅用于打印
# ==================


class DummyPin:
    """--dry-run 用的假管脚，只在屏幕上记录电平，不碰真实硬件。"""

    def __init__(self, pin, initial_value=False):
        self.pin = pin
        self.value = int(bool(initial_value))

    def close(self):
        pass


def make_pins(dry_run):
    """建好 3 个输出管脚，直接按默认状态初始化，避免上电瞬间闪一下别的电平。"""
    initial = STATES[DEFAULT_STATE]
    if dry_run:
        print("⚠️ dry-run 模式：不会操作真实 GPIO，只打印电平\n")
        return [DummyPin(p, v) for p, v in zip(PINS, initial)]

    try:
        from gpiozero import DigitalOutputDevice
    except ImportError:
        sys.exit("缺少 gpiozero。请先执行：sudo apt install python3-gpiozero python3-lgpio")

    try:
        # initial_value 直接用默认状态，(1,1,1) 就是建脚时一次到位
        return [DigitalOutputDevice(pin, initial_value=bool(v))
                for pin, v in zip(PINS, initial)]
    except Exception as e:
        sys.exit(f"GPIO 初始化失败：{e}\n"
                 "  1) 确认脚本是在树莓派上跑的\n"
                 "  2) 确认装了 lgpio：sudo apt install python3-lgpio\n"
                 "  3) 若报 GPIO busy，说明 SPI 占用了 9/10/11，先关掉 SPI 并重启")


def apply_state(pins, name):
    """把状态表里的一行同时写到 3 个管脚上，并打印实际电平。"""
    values = STATES[name]
    # 先算好再一起写，保证同一次切换里 3 个脚的电平是同一组值
    for dev, v in zip(pins, values):
        dev.value = v

    # 回读一遍，确认真的写进去了
    readback = tuple(int(d.value) for d in pins)
    bits = " ".join(str(v) for v in values)
    pins_desc = "  ".join(f"GPIO{p}={v}" for p, v in zip(PINS, values))
    print(f"[{time.strftime('%H:%M:%S')}] 状态 {name}: {bits}   ({pins_desc})")

    if readback != tuple(values):
        print(f"  ⚠️ 回读不一致：期望 {values}，实际 {readback}")
        return False
    return True


def freeze_pin_mux(values, dry_run):
    """用 pinctrl 把引脚再配一遍输出电平。

    gpiozero 的 close() 只是把脚释放（变回输入），进程一退出就没人维持电平了。
    pinctrl 是直接写引脚配置，写完命令就退出，电平会一直留着，所以脚本结束后
    仍停在期望状态。拿不到 pinctrl 就跳过，不影响主流程。
    """
    if dry_run:
        return
    exe = shutil.which("pinctrl")
    if not exe:
        print("  （系统里没有 pinctrl，退出后电平不再维持，如需保持请用 Shell 版脚本）")
        return
    for pin, v in zip(PINS, values):
        subprocess.run([exe, "set", str(pin), "op", "dh" if v else "dl"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def close_all(pins, dry_run=False):
    """退出前把管脚置成默认状态并释放，全程不会出现 0 0 0。

    先写电平再 close，然后交给 pinctrl 把电平固定住，避免程序结束后电平悬着。
    """
    values = STATES[DEFAULT_STATE]
    for dev, v in zip(pins, values):
        dev.value = v
    for dev in pins:
        dev.close()
    freeze_pin_mux(values, dry_run)
    desc = " ".join(str(v) for v in values)
    print(f"GPIO 9/10/11 已置成默认状态 {desc} 并释放")


def print_table():
    title = "  ".join(f"GPIO{p}" for p in PINS)
    print(f"\n状态表（{title}）")
    print("-" * (len(title) + 22))
    for name in ("1", "2", "3"):
        bits = "    ".join(str(v) for v in STATES[name])
        mark = "  ← 默认" if name == DEFAULT_STATE else ""
        print(f"  状态 {name:<3}: {bits}{mark}")
    print("  （只有这三种，脚本不会输出 0 0 0）")
    print("\n排针物理脚： " + "  ".join(f"GPIO{p}->{PIN_HEADER[p]}脚" for p in PINS))
    print()


def main():
    parser = argparse.ArgumentParser(
        description="树莓派 5：GPIO 9/10/11 三状态切换",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例：python3 gpio_9_10_11_3states.py cycle -t 0.5")
    parser.add_argument("action", nargs="?", default=DEFAULT_STATE,
                        choices=["1", "2", "3", "cycle", "list"],
                        help=f"要执行的动作，默认 {DEFAULT_STATE}（即 1 1 1，默认状态）")
    parser.add_argument("-t", "--interval", type=float, default=1.0,
                        help="cycle 模式每步持续秒数，默认 1.0")
    parser.add_argument("-n", "--count", type=int, default=0,
                        help="cycle 模式总共走多少步，0 = 一直循环（默认）")
    parser.add_argument("--hold", type=float, default=0.0,
                        help="切到单个状态后保持多少秒，0 = 保持到按 Ctrl+C")
    parser.add_argument("--dry-run", action="store_true",
                        help="不操作真实硬件，只打印电平")
    args = parser.parse_args()

    if args.action == "list":
        print_table()
        return

    if args.interval <= 0:
        sys.exit("--interval 必须大于 0")
    if args.count < 0:
        sys.exit("--count 不能是负数")

    pins = make_pins(args.dry_run)
    try:
        if args.action == "cycle":
            print(f"开始循环 {' -> '.join(CYCLE_ORDER)}"
                  f"，每步 {args.interval}s，Ctrl+C 停止\n")
            step = 0
            while args.count == 0 or step < args.count:
                apply_state(pins, CYCLE_ORDER[step % len(CYCLE_ORDER)])
                step += 1
                if args.count and step >= args.count:
                    break
                time.sleep(args.interval)
            print(f"\n循环结束，共 {step} 步")
        else:
            apply_state(pins, args.action)
            if args.hold > 0:
                print(f"保持 {args.hold}s ...")
                time.sleep(args.hold)
            else:
                print("已保持，按 Ctrl+C 退出（退出时会回到默认状态 1 1 1）")
                while True:
                    time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C")
    finally:
        close_all(pins, args.dry_run)

_pins = None

# 全局保存 GPIO 对象，防止函数返回后被回收导致引脚释放
_tuigan_pins = None


def tuigan(state, dry_run=False):
    """切换到指定状态并保持。state: 1/2/3"""
    global _tuigan_pins
    name = str(state)
    if name not in STATES:
        raise ValueError("state 只能是 1、2、3")
    if _tuigan_pins is None:
        _tuigan_pins = make_pins(dry_run=dry_run)
    apply_state(_tuigan_pins, name)
    if not dry_run:
        freeze_pin_mux(STATES[name], dry_run=False)
    return _tuigan_pins


def tuigan_release(dry_run=False):
    """恢复默认状态 3 并释放 GPIO。"""
    global _tuigan_pins
    if _tuigan_pins is not None:
        close_all(_tuigan_pins, dry_run=dry_run)
        _tuigan_pins = None

if __name__ == "__main__":
    tuigan(2)
    time.sleep(2)


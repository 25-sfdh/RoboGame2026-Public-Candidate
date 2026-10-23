"""串口设备路径解析。

树莓派上 USB 转串口的 /dev/ttyUSB*、/dev/ttyACM* 编号是会变的：USB 总线一旦过流
或重枚举，同一个适配器就可能从 ttyUSB0 变成 ttyUSB1。2026-09-21 那次就是这么坏的，
dmesg 里两次 "over-current change"，第二次重枚举之后舵机那路 CH340 从 ttyUSB0 变成
了 ttyUSB1，而 set_angle.py 里写死的还是 ttyUSB0，于是所有舵机指令都报
"No such file or directory"。

所以一律不要写死编号，而是用 /dev/serial/by-id/ 下按厂商和序列号生成的符号链接：
重枚举之后内核会重建这些链接，名字不变，代码不用动。

by-id 万一不可用（udev 还没处理完、驱动没加载），退回按编号通配，保证不会因为一个
目录不存在就整台车起不来。

命令行自检，随时可以看当前解析到哪：
    python3 serial_ports.py
"""

import glob
import os

BY_ID_DIR = "/dev/serial/by-id"

# 舵机总线：CH340 (1a86:7523)。这个适配器没有序列号，所以 by-id 名字是固定的
# "1a86_USB_Serial"；以后若换用带序列号的适配器，把新名字加到元组最前面即可。
SERVO_BY_ID = ("usb-1a86_USB_Serial-if00-port0",)
SERVO_GLOBS = ("/dev/ttyUSB*",)

# 底盘：CDC 设备 (1a86:55d4)，带序列号 CHASSIS_SERIAL_REDACTED。第二个名字是不带序列号时的兜底，
# 换了底盘板子也能认出来。
CHASSIS_BY_ID = (
    "usb-1a86_USB_Single_Serial_CHASSIS_SERIAL_REDACTED-if00",
    "usb-1a86_USB_Single_Serial-if00",
)
CHASSIS_GLOBS = ("/dev/ttyACM*",)


def resolve(by_id_names, glob_patterns):
    """按 by-id 优先、编号通配兜底的顺序，返回第一个存在的设备路径。

    都找不到时返回首选 by-id 路径。这样报错信息里出现的是「应该在哪」，比返回空字符串
    或者一个不存在的编号更容易看出问题。
    """
    for name in by_id_names:
        path = os.path.join(BY_ID_DIR, name)
        if os.path.exists(path):
            return path
    for pattern in glob_patterns:
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return os.path.join(BY_ID_DIR, by_id_names[0])


def servo_port():
    """舵机总线当前的设备路径。"""
    return resolve(SERVO_BY_ID, SERVO_GLOBS)


def chassis_port():
    """底盘串口当前的设备路径。"""
    return resolve(CHASSIS_BY_ID, CHASSIS_GLOBS)


def describe():
    """返回一份人类可读的解析结果，用于排查。"""
    lines = []
    for label, by_id_names, globs, path in (
        ("舵机 (CH340)", SERVO_BY_ID, SERVO_GLOBS, servo_port()),
        ("底盘 (CDC)", CHASSIS_BY_ID, CHASSIS_GLOBS, chassis_port()),
    ):
        source = "by-id" if path.startswith(BY_ID_DIR) else "编号通配兜底"
        lines.append(f"  {label}: {path}    [{source}]")
        for name in by_id_names:
            full = os.path.join(BY_ID_DIR, name)
            real = os.path.realpath(full) if os.path.exists(full) else "不存在"
            lines.append(f"      期望的 by-id: {full} -> {real}")
        for pattern in globs:
            hits = sorted(glob.glob(pattern))
            lines.append(f"      通配 {pattern}: {hits if hits else '无'}")
    return "\n".join(lines)


def _main():
    print("串口解析结果：")
    print(describe())
    print()
    print("提示：只要上面是 by-id 开头的那条，重枚举改编号就不会影响脚本。")
    missing = [p for p in (servo_port(), chassis_port()) if not os.path.exists(p)]
    if missing:
        print()
        print("注意：以下路径当前不存在，检查线是否插好、适配器是否被系统识别：")
        for path in missing:
            print(f"  {path}")


if __name__ == "__main__":
    _main()

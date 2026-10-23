# System Architecture

## 模块如何协同

`grip_two_six_servo_new.py` 调用运动、视觉对位、舵机和推杆函数，按固定阶段推进。它是任务编排脚本。`trace_car_v2.py` 和 `visual_block_positioning.py` 各自维护局部状态机，并分别向底盘串口发送速度命令。

| 输入/接口 | 消费模块 | 控制作用 |
| --- | --- | --- |
| 前后各七路数字循迹输入 | `trace_car_v2.py` | 横向误差、姿态误差与路口判断 |
| 底盘串口 IMU 数据 | `trace_car_v2.py` | 转弯粗转判断 |
| 视觉服务的目标、配置、健康接口 | `visual_block_positioning.py` | 目标有效性、横向对位与位置状态 |
| 超声距离 | 视觉客户端与运动函数 | 接近分档与距离停车 |
| 舵机多圈角度读回 | `set_angle.py` | 等待估算、偏差判断和限次重发 |
| GPIO 9/10/11 | `Tuigan.py` | 推杆状态切换 |

## 局部状态与上层流程

底盘主要经过 `FOLLOW → BRAKE → ADVANCE → IMU_TURN → IMU_SETTLE → ALIGN/CORNER_CHECK → SETTLE`。视觉主要经过 `WAIT_TARGET → ALIGN → APPROACH → POSITION_CONFIRM`，横向误差过大时进入 `APPROACH_REALIGN`。

任务入口在 `find_block1/2()` 失败时退出，清理阶段调用舵机卸力与推杆资源清理；推杆代码仍可能保持 GPIO 输出，不能据此认定执行机构已断电。部分底盘动作与升降动作放在线程里并行执行；`join()` 仅等待线程结束，没有核验 `_thread_errors`。舵机重试耗尽也未返回明确失败状态供任务层判断。不能把局部重试和局部停机概括为全系统容错。

## 版本证据

比赛入口由参赛者确认，归档文件哈希为 `9998191028CBA49770500AEF8235F6A43F44E4C2BEB29F2882C14AF3D7E7BF7D`。服务端 `vision_runtime.py` 是原始归档中的候选，未纳入当前仓库；保存配置的 `legacy_shadow` 不能证明比赛日实际配置。详细边界见 [fact_check.md](fact_check.md)。

# Software Structure

## 为什么保留 `src/robot/` 的平铺结构

比赛源码采用 `import trace_car_v2`、`from set_angle import ...` 等同目录导入。直接分到 `src/task/`、`src/control/`、`src/servo/` 会改变导入环境；本候选保留十个历史文件：九个原样保存，`serial_ports.py` 按授权只脱敏两处 USB 序列号，源/候选 SHA-256 分别记录，用职责表表达模块划分。若将来包化，应另建开发版本并核验行为。

| 职责 | 文件 |
| --- | --- |
| Task | `grip_two_six_servo_new.py` |
| Control | `trace_car_v2.py`、`distance.py`、`distance_zuo.py`、`serial_ports.py` |
| Vision client | `visual_block_positioning.py` |
| Manipulator | `set_angle.py`、`Tuigan.py` |
| Imported compatibility/debug modules | `trace_car_v1.py`、`test_comm.py` |

入口仍导入最后两个模块，即使当前主流程未调用其中函数，忠实保存时也须保留。

## 视觉服务与配置

归档 `pi_robot_code/cam/color/` 中的 `vision_runtime.py`、`vision_legacy_stable.py`、`vision_shadow_common.py` 是候选服务与直接依赖。`legacy_shadow` 和 `experimental` 分支还可能加载面检测等模块；不能只复制三个文件就宣称依赖完整。这些服务与配置均未纳入当前候选；是否以后加入需另行确认。

机器人角度、速度和若干阈值写在源码中，视觉配置文件也有固定部署路径。本候选没有 `config/` 或配置示例；不能简单挪动 JSON 并声称现有代码会读取新位置。

## 测试与实验边界

原始归档中的 `tests/test_visual_block_positioning.py`（本候选未纳入） 导入 `robot_lib.visual_block_positioning`，还引用旧目录的稳定源文件。它并非自动验证最终 `pi_current_20261002` 栈的测试。首轮暂不作为核心 CI，待在独立副本中适配和核验。

以下实验均未纳入本候选。原始归档中，边缘调试包的几何测试配套 `vision_edge_align_test.py`，可作为 `experiments/edge_alignment/` 的离线验证资料。高速测试需要保留其匹配的旧控制栈和参数，不抽取某一文件来混用最终版本。目录与来源映射见 [FILE_SELECTION.md](../FILE_SELECTION.md)。

## 来源与运行边界

源文件映射与哈希见 [SOURCE_MANIFEST.json](../SOURCE_MANIFEST.json)。`fact_check.md` 保留既有核查原文，其中部分证据来自未纳入的服务、实验或历史资料，不能把整张表视为当前仓库的运行验收。十个 Python 文件仅做 AST 语法解析，未导入模块或测试硬件。入口会驱动哪些设备及陈旧 CLI 注释见 [SECURITY_FINDINGS.md](../SECURITY_FINDINGS.md)。

## 2026-10-23 标识脱敏

`serial_ports.py` 第 29、32 行的同一历史序列号已替换为 `CHASSIS_SERIAL_REDACTED`。该占位标识不能匹配真实设备；解析函数、通用 by-id 回退和 ttyACM 通配逻辑均未修改。设备选择结果可能不同，实际运行前必须核对本机映射。本轮只做静态检查，不宣称运行行为完全等价，详见 [SOURCE_CHANGE_PROPOSALS.md](../SOURCE_CHANGE_PROPOSALS.md)。

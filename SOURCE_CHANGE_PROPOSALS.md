# Source Change Proposals and Applied Changes

更新日期：2026-10-23。P1 已按用户明确选择“仅在候选副本脱敏”执行；P2 未获修改授权，继续保留。原始比赛目录未修改。

| 编号 | 文件 / 行号 | 原内容（脱敏显示） | 处理 | 影响与证据 |
| --- | --- | --- | --- | --- |
| P1 | `src/robot/serial_ports.py:29,32` | 注释中的设备序列号，以及 `usb-1a86_USB_Single_Serial_<序列号>-if00` | 两处序列号替换为 `CHASSIS_SERIAL_REDACTED` | 仅这两行的同一字符串改变，换行与其余源码字节保持。不能再称此文件原样复制；占位 by-id 不匹配真实硬件，现有通用 by-id / ttyACM 回退仍保留，设备选择结果未真机验证。 |
| P2 | `src/robot/visual_block_positioning.py:156` | 历史部署路径 `/home/pi/cam/color` | 未修改，以文档说明保留 | 通用历史路径并非已确认凭据；服务与启动环境未核定。 |

P1 原始源码 SHA-256：`96918886D2C705A821CC012F420326F3FCCA5891BDFD2D35A1F05054B9AB3CA8`  
P1 候选源码 SHA-256：`1CC1F849500AA681C4BDCBCD37831BF45BA1BD1E23911A7FD26678C8A78BD4C2`

其他九个 Python 文件与原件 SHA-256 一致。既有错误、控制流程、函数名与文件名均未调整；两处替换的字节差异及语法已静态核对，未执行、导入硬件代码。原始标识不写入候选报告或差异摘录，原件仍保存在原始归档中。

2026-10-23 用户随后明确选择 MIT，标准许可证已加入根目录。此处仅记录源码标识脱敏与许可选择；图片清理另见 [media/README.md](media/README.md)。

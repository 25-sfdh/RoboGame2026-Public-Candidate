# 文件整理方案

本文件记录已建立的历史快照范围。十个核心源码已按白名单建立，原始资料保留在本地。2026-10-23 按用户授权仅脱敏 `serial_ports.py` 两处 USB 序列号，其余九个源码原样保留。用户已确认当前白名单公开范围与个人叙述；本历史快照采用 MIT 许可。

## 1. Core Code

来源统一为 `代码/robogame三审速通/pi_current_20261002/`，候选目标统一为 `src/robot/`。

| 文件 | 作用 | 首轮处理 |
| --- | --- | --- |
| `grip_two_six_servo_new.py` | 确认的比赛入口 | 原样保留，记录哈希 |
| `trace_car_v2.py` | 循迹/路口/运动 | 与入口同版本保留 |
| `visual_block_positioning.py` | 视觉客户端/对位 | 保留接口与已知局限说明 |
| `set_angle.py`、`Tuigan.py` | 舵机/夹爪/推杆 | 保留角度反馈、重试和时序证据 |
| `distance.py`、`distance_zuo.py`、`serial_ports.py` | 测距和设备解析 | 保留间接依赖 |
| `trace_car_v1.py`、`test_comm.py` | 入口仍导入的兼容/调试模块 | 暂保留，不在文档中当作主控制方案 |

十个文件的来源、实际目标、大小与 SHA-256 见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。九个文件仍与原始字节一致；`serial_ports.py` 仅有已记录的两处序列号替换，源与候选哈希分别保存。本快照采用 MIT 许可。

视觉服务候选来自 `pi_robot_code/cam/color/`：`vision_runtime.py`、`vision_legacy_stable.py`、`vision_shadow_common.py` 及相关配置/分支依赖。比赛日版本、完整依赖、路径与权限未确认，暂不列入无条件公开清单。核定后按原内部关系放入 `src/vision/`；不把服务占位文档冒充已完成模块。

## 2. Experiments（本候选未纳入）

以下只保留以后筛选时的背景说明，不属于本次白名单。

| 来源（均相对 `代码/robogame三审速通/`） | 拟议位置 | 边界 |
| --- | --- | --- |
| `vision_edge_debug_20261001/` 中的两个主脚本、几何测试、说明及必要 dependencies | `experiments/edge_alignment/` | 保持配套关系，排除缓存和未审阅数据；标注 dry-run/离线实验 |
| `pi_review_20260921/fast_baseline_600/` | `experiments/fast_follow/`，可选 | 实验与旧控制栈完整配套；不能混用最终底盘参数 |
| 面分割、动态尺度、shadow、吸盘视觉实现及匹配测试 | 暂缓选择 | 仅当能解释目标、依赖和证据时作为独立实验纳入 |
| `tests/test_visual_block_positioning.py` | `tests/`，待适配 | 当前导入旧 `robot_lib`，不能直接宣称测试最终比赛栈 |

本候选不创建 `tests/` 或 `experiments/`，也不提供 CI 通过标识。

## 3. Archive（本候选未纳入）

`grip_two_six.py`、`grip_six_whole.py`、`grip_nine.py`、`grip_nine_v2.py` 等历史入口，优先以版本说明记录；需要公开代码时只选能说明关键变化的少数版本，放入 `archive/`。某历史入口仍依赖其他任务脚本时，应说明其依赖，不能作为单文件可运行版本。

`main_project/`、`remote_review_20260919/`、`pi_latest_main_project_20260929/` 等整组快照继续留在本地。文件名、日期和缓存不能证明历史执行效果；内容重复只标记，不删除。`archive/` 可在首版公开时省略，不上传全量历史备份。

## 4. 不公开文件

| 类型 | 本次定位/理由 | 计划 |
| --- | --- | --- |
| SSH 记录 | `代码/robogame三审速通/.ssh_known_hosts` | 排除 |
| 内网地址 | 旧审查文档、`cam/record/web_stream.py`、`cam/color/web_stream*.py`、`VISION_API_README.md` 共六个文本候选 | 排除非核心文档；后续所选文件逐项审核，不改原件 |
| 用户路径和本地配置 | Windows 用户目录、固定部署路径、编辑器设置、整机环境 | 不批量复制；系统通用路径与个人路径分别核对 |
| 第三方资料与比赛内部材料 | `RG/`、`robogame培训/`、比赛规则/赛程、外部工程与 SDK 环境 | 原则上仅链接获准来源；确有许可时才另选 |
| 未选 CAD、视频、图片 | 不在本轮五张图片的确认范围内 | 不批量加入；已确认的五张图片见下节 |
| 大型二进制、模型、压缩包 | 当前原始目录 39 个文件大于 10 MB，包括外部工程与环境 | 排除，不能直接上传整个目录 |
| 缓存、临时日志、原始采集 | `.pyc`、`.swp`、`.bak.*`、`.local/`、`venv/`、摄像头采集/日志 | 不进入公开选择，内部保留 |

IP 字面量不都表示私人信息：回环地址、监听地址和真实部署地址用途不同；源码中的值需按用途审核，不能仅靠 `.gitignore` 完成内容检查。

## 5. 本地审核附加文件

原白名单包含六份 docs、`media/README.md` 和顶层文档；当前范围另含下节记录的 11 份机械说明；另加入标准 MIT `LICENSE`，生成 `LICENSE_STATUS.md`、`SECURITY_FINDINGS.md`、`SOURCE_CHANGE_PROPOSALS.md`、`LOCAL_RELEASE_AUDIT.md`。未复制 `PUBLISH_PLAN.md`、`mentor_review.md`、`LICENSE_PENDING.md`、配置说明或第三方源码。每份复制文档的来源、源哈希与候选状态见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)；完整实存清单见 [LOCAL_RELEASE_AUDIT.md](LOCAL_RELEASE_AUDIT.md)。

## 6. 已确认的五张展示图片

2026-10-23 用户确认五张所附图片可在本项目公开展示，并确认奖牌对应本团队项目。当前白名单增加 `media/robot-overview.jpg`、`media/team-award.jpg` 与三张 `media/competition-*.jpg`。公开副本只去除 JPEG 隐藏元数据，原图保留；保留水印、记录来源哈希。图片未自动纳入 MIT 复用许可，具体范围见 [media/README.md](media/README.md)。

## 7. 机械说明文档（2026-10-24 确认范围）

当前包含 [机械系统主页](mechanical/README.md)、六份 `mechanical/docs/` 专题说明，以及 CAD、工程图、BOM、图片四份状态说明，共 11 份 Markdown。它们是新编说明，依据内部设计记录、CAD 文件名/目录清点和五张已确认图片，不是原始比赛文档的原样复制。逐文件来源依据与当前哈希见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。

机械说明文本的团队公开范围依据本人于 2026-10-24 的确认，无独立书面核验；该确认不包含 CAD/工程图文件及第三方模型的公开或再分发授权。没有新增 CAD、工程图、BOM 文件或图片。当前总计 44 文件（39 文本 + 5 JPEG），机械说明计入这 39 个文本文件。

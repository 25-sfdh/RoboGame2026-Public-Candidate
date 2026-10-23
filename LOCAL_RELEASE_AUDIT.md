# Historical Snapshot Audit

记录日期：2026-10-23；定向修复与复查：2026-10-24。本记录描述 historical engineering snapshot / as-is 的文件与静态核查范围，适用于本地副本或 GitHub 展示。共 44 个文件：39 个文本文件与 5 张静态 JPEG（含 Git 初始化生成的 .gitattributes，以及 2026-10-23 新增的 11 份机械说明）。

## 1. 实际目录

```text
RoboGame2026-Public-Candidate/
├── docs/
│   ├── architecture.md
│   ├── development_notes.md
│   ├── fact_check.md
│   ├── hardware.md
│   ├── lessons_learned.md
│   └── software_structure.md
├── media/
│   ├── competition-field.jpg
│   ├── competition-stacking.jpg
│   ├── competition-transport.jpg
│   ├── README.md
│   ├── robot-overview.jpg
│   └── team-award.jpg
├── mechanical/
│   ├── README.md
│   ├── cad/README.md
│   ├── drawings/README.md
│   ├── bom/README.md
│   ├── images/README.md
│   └── docs/
│       ├── system_overview.md
│       ├── chassis_design.md
│       ├── lifting_mechanism.md
│       ├── gripper_design.md
│       ├── design_iterations.md
│       └── assembly_notes.md
├── src/
│   └── robot/
│       ├── distance.py
│       ├── distance_zuo.py
│       ├── grip_two_six_servo_new.py
│       ├── serial_ports.py
│       ├── set_angle.py
│       ├── test_comm.py
│       ├── trace_car_v1.py
│       ├── trace_car_v2.py
│       ├── Tuigan.py
│       └── visual_block_positioning.py
├── .gitattributes
├── .gitignore
├── FILE_SELECTION.md
├── LICENSE
├── LICENSE_STATUS.md
├── LOCAL_RELEASE_AUDIT.md
├── README.md
├── RELEASE_CHECKLIST.md
├── SECURITY_FINDINGS.md
├── SOURCE_CHANGE_PROPOSALS.md
└── SOURCE_MANIFEST.json
```

## 2. 来源与当前哈希

来源路径相对原始工程归档；源哈希保存在 SOURCE_MANIFEST.json，下面 SHA256 为此快照当前文件。manifest 自身哈希仅在本表记录，避免自哈希循环。原始比赛源码保留，九个公开源码原样、一份仅两处 USB 序列号脱敏。

| Public路径 | 原始来源 | SHA256 | 状态 |
| --- | --- | --- | --- |
| `src/robot/grip_two_six_servo_new.py` | `代码/robogame三审速通/pi_current_20261002/grip_two_six_servo_new.py` | `9998191028CBA49770500AEF8235F6A43F44E4C2BEB29F2882C14AF3D7E7BF7D` | 原样复制；SHA-256 一致 |
| `src/robot/trace_car_v2.py` | `代码/robogame三审速通/pi_current_20261002/trace_car_v2.py` | `FA98C3E8FE380EA05CF5BFBF6BEEF12E0BAA5B2317F024B803EC9410DF79A810` | 原样复制；SHA-256 一致 |
| `src/robot/trace_car_v1.py` | `代码/robogame三审速通/pi_current_20261002/trace_car_v1.py` | `AF31956587135C48C0A2FA59128EF4A4F6D1B17261F56F024829550D18DA7EF5` | 原样复制；SHA-256 一致 |
| `src/robot/visual_block_positioning.py` | `代码/robogame三审速通/pi_current_20261002/visual_block_positioning.py` | `D6F24E6E6FFA4754E42F23A124FEA4417F7FFAD0B0D44B7FF772CA566842174D` | 原样复制；SHA-256 一致 |
| `src/robot/set_angle.py` | `代码/robogame三审速通/pi_current_20261002/set_angle.py` | `878BB5C74C08CF731287DDF72F0DDD1C13E067EFF105B5D32157A95904818AB3` | 原样复制；SHA-256 一致 |
| `src/robot/distance.py` | `代码/robogame三审速通/pi_current_20261002/distance.py` | `048A14A1F6725FDC74BDA5AD97454B12E78EBFEDD29F143C81A36047279EE217` | 原样复制；SHA-256 一致 |
| `src/robot/distance_zuo.py` | `代码/robogame三审速通/pi_current_20261002/distance_zuo.py` | `989FD6F1FC97420FD22A0DD253AC460B1A1E8FE92372B8BE9572042F77A93640` | 原样复制；SHA-256 一致 |
| `src/robot/serial_ports.py` | `代码/robogame三审速通/pi_current_20261002/serial_ports.py` | `1CC1F849500AA681C4BDCBCD37831BF45BA1BD1E23911A7FD26678C8A78BD4C2` | 两处授权 USB 标识脱敏；原件保留 |
| `src/robot/Tuigan.py` | `代码/robogame三审速通/pi_current_20261002/Tuigan.py` | `E0820164BDE3929197A956570293447C2DAC2B793E98DBA637239C42C4281DE0` | 原样复制；SHA-256 一致 |
| `src/robot/test_comm.py` | `代码/robogame三审速通/pi_current_20261002/test_comm.py` | `F1484BC38CB08665C158DE615D0E8FA77A19A7BA46D36323969432A18659406C` | 原样复制；SHA-256 一致 |
| `README.md` | `GitHub发布方案_20261013/README.md` | `5DA01657016AB46BBE5B44E88BC16C61552D2ABADB99686753076AE7D1268531` | 候选文档更新；源文件未改 |
| `.gitignore` | `GitHub发布方案_20261013/.gitignore` | `C276867485D936866561EEB103364D4327C44F8F07F4251BBE78B71F580B3957` | 候选文档更新；源文件未改 |
| `.gitattributes` | 新建（Git 初始化模板） | `1A1DBE176BC233B499D35A57DB7513F2941C99AB9759F177830C9149BE99005B` | 新增；`* text=auto` |
| `RELEASE_CHECKLIST.md` | `GitHub发布方案_20261013/RELEASE_CHECKLIST.md` | `F310D8CA6180741CA71B2C0C284F4FBA3B1F4E260671B53C586D01F82DFA27EF` | 候选文档更新；源文件未改 |
| `FILE_SELECTION.md` | `GitHub发布方案_20261013/FILE_SELECTION.md` | `586A1D142467494F93D286DDCADFE567891FBCC0F0DFF37F6E1255E6B834AEDA` | 候选文档更新；源文件未改 |
| `SOURCE_MANIFEST.json` | `GitHub发布方案_20261013/SOURCE_MANIFEST.json` | `F2BCDFF3A879B7194C9D046E9156B33D2D754A5E0D6178AFA717E6AA86FB4CF5` | 由历史清单派生并更新 |
| `docs/architecture.md` | `GitHub发布方案_20261013/docs/architecture.md` | `9BD52228A358157B2C87E920C0E354CF7E611E3861AB3DB7F961664DEEBC021B` | 候选文档更新；源文件未改 |
| `docs/hardware.md` | `GitHub发布方案_20261013/docs/hardware.md` | `FCD7E1170E53EF1042C4831DF1981A305338D5D14F4473EE91C2B3748B60A48F` | 候选文档更新；源文件未改 |
| `docs/software_structure.md` | `GitHub发布方案_20261013/docs/software_structure.md` | `FA8DC63AE7CD6FF09BBCCAD6AB8C4CB6390C9272FBB830EFA448F12E7322336F` | 候选文档更新；源文件未改 |
| `docs/development_notes.md` | `GitHub发布方案_20261013/docs/development_notes.md` | `65F149A858B1BE076384F7670317921E106A5E772089485C8EF08CAAF11F7B97` | 候选文档更新；源文件未改 |
| `docs/lessons_learned.md` | `GitHub发布方案_20261013/docs/lessons_learned.md` | `9191F3ED6EDE2183A299F36C2A8A065B1578FB7A988D9FF456B78B8E36147044` | 原样复制；SHA-256 一致 |
| `docs/fact_check.md` | `GitHub发布方案_20261013/docs/fact_check.md` | `951DF87D9C93B343F980FC24C272E1140C6DF35194908D3E30C1520FD7A126C0` | 候选文档更新；源文件未改 |
| `media/README.md` | `GitHub发布方案_20261013/media/README.md` | `13E04D318FEC0332A07AD4409885FD97F5EAFC823BC35100139CA3ED11C60156` | 候选文档更新；源文件未改 |

LICENSE 为标准 MIT 文本，SHA-256：`30A1D2F285C2252170C5DF7521E4AE164BDF00DC56F48B1B32F05B61DFE32C54`。其余许可/安全/源码变更文档为整理记录，不冒充原始比赛资料。

## 3. 五张静态图片

| 文件 | 来源 | 原图 SHA256（历史溯源） | 当前 SHA256 | 当前大小 | EOI 后字节 |
| --- | --- | --- | --- | ---: | ---: |
| `media/robot-overview.jpg` | attachment-1 | `21B35EB3FAF5F73726AE464BFE6434CDCD9B69EC0048E73F430E8D5F7495E408` | `BC5DC5FD083B92430C75251C2B1B858744C3DF74E039EE8ACE164C869F296668` | 1436664 | 0 |
| `media/team-award.jpg` | attachment-2 | `2D7096D629E9A8A47F233A0445EB86CE27EB3BB49FA30F690C97FA230513E363` | `7580395965E9D40821297E594B60054720DC9010DBBDBA0E8865E0C251B40946` | 3748634 | 0 |
| `media/competition-stacking.jpg` | attachment-3 | `9D08AF3D78D805665A09E7CA97F5745759C9468314EBF12F08D1ECF2B4B00963` | `182E5CC3A9F50B6080DD29310AE1D955DC90E8AAA81F3F69E4C90A30D789C10B` | 64333 | 0 |
| `media/competition-transport.jpg` | attachment-4 | `6D03216A4D2C9E40572961EC80130D6D3E2A53F2BB23EB9F1078A629A4BAFE19` | `DE4CC8DCDFE5E3572987939E85187978232A7C725C08B82B93C23C2FF06EB950` | 68457 | 0 |
| `media/competition-field.jpg` | attachment-5 | `C47BCD193F74B8C4B5F4CE126D568D045CEC6B5DCB36A80EF83C151CF6D8211A` | `9ABDF53F6D1BA0629A5F01D77FAF528AEF6D593BB7FAF88641D8B47F1D60BD57` | 67833 | 0 |

## 4. 外部审查修正与验证

- team-award.jpg 旧副本的结构性 EOI 后有 4,517,519 字节 MP4，容器记录 3.026 秒并含视频/音频轨。尾部数据已全部移除，未播放、未推断具体内容；APP3/APP4 也已移除。
- 三张比赛截图的 24 字节尾注已移除；五张图 EOI 后均为零字节。
- 其余四张 JPEG 保留 APP0 JFIF；team-award.jpg 无 APP0，仅保留 ICC_PROFILE APP2（含标记共 650 字节，长度字段为 648）色彩配置。全部图片不存在 EXIF/XMP、IPTC、COM、厂商 APP3/APP4 或其他未说明 APP 段。
- Pillow 解码通过，相对修复前候选副本的解码像素、JPEG 图像编码数据与 ICC 配置完全一致。先前把 SOS 后整段（含尾部数据）当作图像数据的哈希口径已替换为结构化图像编码哈希。
- 原始附件哈希仅作历史溯源，本轮没有声称重新读取全部原图。公开清单的来源名中性化为 attachment-1..5，私有文件名映射保留在本地整理记录中。
- README 与文档改为稳定的 historical snapshot / as-is 描述；不再根据远端是否已经上传改变事实含义。
- FACT_CHECK 保留技术原文，并添加 2026-10-23 已确认事项的历史说明。当前白名单公开范围与图片展示权限依据参赛者确认，未独立核验书面权属附件；不扩展到未选 CAD/服务或原始归档。
- 修正媒体说明文档的派生状态、27/32 文件计数、test_comm.py 入口行号，以及 team-award 无 APP0 JFIF 的记录。入口线程参数、Ctrl+C/异常上报、角度 None 等历史缺口已补充披露；机器人源码不作功能修改。
- 相对文档/图片链接与清单哈希复核通过。十个 Python 只做 AST 解析；没有导入模块、运行执行器、安装依赖或测试真机。

## 5. 展示范围与局限

MIT 用于代码与项目说明文档，图片仅确认在本仓库展示，复用范围单独说明。完整视觉服务、依赖版本锁定、接线和完整真机复现仍缺。未纳入比赛规则、培训材料、压缩包、备份、第三方 SDK、原始日志/摄像头数据、完整视频或其他 CAD/媒体。

最初建立代码与文档时为 26 个文件，加入 MIT 时为 27 个（加入图片前），加入五张图片后为 32 个；Git 初始化加入 `.gitattributes` 后为 33 个；2026-10-23 新增 11 份机械说明后为 44 个（39 文本 + 5 JPEG）。审查只覆盖所列快照与必要的来源哈希，不宣称整个原始归档已经净化或整机运行已验收。

## 6. 机械模块补充（2026-10-23）

机械模块为新建说明文档，不是原始归档复制。依据为内部设计记录（2026-04～05）、CAD 文件名/目录清点（2026-05～06 版本、2026-10-01 归档）和五张已确认照片。**CAD 原生文件、工程图、第三方模型和 BOM 均未加入本仓库**；公开权限、体积、Git LFS 和第三方再分发条款仍待确认。**机械说明文档的团队公开范围已由本人确认已获团队授权（2026-10-24，无独立书面材料）；本审计按该确认记录，但不声称独立核验。**详见 [mechanical/README.md](mechanical/README.md)。

| 文件 | SHA256 | 状态 |
| --- | --- | --- |
| `mechanical/README.md` | `1552B4FE21B52103910194C4C85FA3FE3729420621DFA7429D88A5EA40AE3EB3` | 新建；机械系统主页/贡献边界/团队授权已确认 |
| `mechanical/docs/system_overview.md` | `4636D13ACFAEF7C2A7E3C109384CD2B2C298FA817545399FE51C536D0FE6AF67` | 新建；总体布置/尺寸证据 |
| `mechanical/docs/chassis_design.md` | `94DFAFB5235DAFD2C6A3B7BBA5A16C71472706D61C4563DB36E0CAD741BFBCDE` | 新建；麦轮底盘/早期 vs 最终 |
| `mechanical/docs/lifting_mechanism.md` | `7E3DADD0F96A4A9A9CF4F5D15A2509DACFC984B3A505E02FA9F21BAF94DB61A1` | 新建；升降机构/驱动更换 |
| `mechanical/docs/gripper_design.md` | `918FDAB3A12CD807AED47C07028B3420D7F472FE8DF36067C31A27058177D719` | 新建；三层夹爪/本人四项设计 |
| `mechanical/docs/design_iterations.md` | `CD83F85288093BA686AFB1812FBEF350DDC36559B5995AF825BAFF903FBBDE83` | 新建；设计迭代/项目时间线 |
| `mechanical/docs/assembly_notes.md` | `11BA7887F4C96C7AF156DB59B566D17F51BC0AE9ABD66CA6155C42EDA580B8EB` | 新建；装配/调试/制造节点 |
| `mechanical/cad/README.md` | `1EBB8DD5B44A667AE46FC8171D1627817A1592069AA6A1B7598843C4BB5F37BD` | 新建；CAD 清点/授权状态 |
| `mechanical/drawings/README.md` | `081505A01755B9ABABBE90167D1EC861BB3881FD100FBC2FFA8722E8E251163A` | 新建；图纸状态 |
| `mechanical/bom/README.md` | `150A9A756469E420A6E85DCD360E2EA52511DD973308CBEC442527778DA2E2FB` | 新建；BOM 缺失说明 |
| `mechanical/images/README.md` | `E247AFF1332F58FAC467170D2C06056772CBFAA910B44B9B3E80D7045BD9AA6B` | 新建；图片来源 |

机械文档不含新的图片二进制文件；引用的五张图片仍在 `media/`，其展示授权不自动等于 MIT 再授权。机械 CAD/图纸的公开状态为 **BLOCKED（未授权，不公开）**。机械说明文档本身为 **已确认（2026-10-24，本人陈述已获团队授权；无独立书面材料）**。

## 7. Git 元数据、工作区提交状态与远程历史（2026-10-24）

| 检查层 | 结果 |
| --- | --- |
| 工作区文件 | 44 个内容文件（39 文本 + 5 JPEG）；机械文档 11 个；无 CAD/STEP/DWG/DXF/BOM/新增图片 |
| 提交前工作区 Git 状态（本审计编写时） | `HEAD`/`origin/main` 均为 `ca515590aeaeb60854536b7cf1f974445930c88d`（重写后）；`git status` 显示 5 个修改文件 + `?? mechanical/`；44 文件工作区内容尚未提交，远程树仍是重写前的树 |
| Git commit metadata | 重写后的两个提交作者/提交者为 `25-sfdh <25-sfdh@users.noreply.github.com>`；原 QQ 邮箱已从当前提交元数据移除；旧提交对象可能被 GitHub 缓存 |
| 远程历史 | 2026-10-24 按所有者授权执行 `--force-with-lease`；重写后、本次提交前 `origin/main` 为 `ca515590…`；未联网检查 GitHub 旧 SHA 缓存、fork 或 PR |

- 2026-10-24 已在所有者明确选择“改写并 force-push”后改写本地历史并推送；本地 `user.email` 已设为 GitHub noreply。没有把原 QQ 邮箱标记为“已同意公开”。
- **当前远程分支的提交元数据已不再含原 QQ 邮箱**；但 GitHub 可能保留不可达旧提交/缓存，fork、PR 或第三方镜像不受本仓库控制，不能保证所有副本已清除。
- 本审计编写时 44 文件工作区（机械文档、贡献边界、审计修订）仍未提交；因此远程树没有本轮机械文档和最新审计内容。需要所有者另行 Commit/Push 才会公开。
- 机械说明文档的团队授权已由本人确认（2026-10-24，无独立书面材料）；本轮按该确认提交。

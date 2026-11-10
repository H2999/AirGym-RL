# AirGym user160 工程入口

目标：205 g（整机实测已更正）四旋翼真实接触固定装甲钢板，法向速度 5±0.5 m/s，撞后 2 s 内恢复姿态。
本仓库是 AirGym/Isaac Gym 工程；`/home/hzq/RL` 的 Flightmare 和早期 CartPole 工作区不是本项目。

## 开工顺序

1. 阅读 `PROGRESS.md`、`feature_list.json`、`session-handoff.md`。
2. 执行 `bash init.sh`：默认只检查文件/证据，不安装依赖，不启动 GUI、训练或连接飞控。
3. 从功能清单选择一个范围；同时最多一个 `in_progress`。
4. 按需阅读 `docs/harness/architecture.md`、`docs/harness/runbook.md`。
5. 涉及真机时阅读 `docs/hardware/dm_fc01.md` 和 `docs/hardware/data_capture.md`。

## 已知事实与边界

- 实际工程路径 `/home/hzq/RM/AirGym_RL/AirGym`，远程 `https://github.com/H2999/AirGym-RL.git`；当前工作分支 `dev`。
- 现有仿真飞机仍为160g的 `user160_cad_v1`，待205g质量分布更新；任务 schema `armor_impact_attitude_v3_user160`，30 维观测；CTBR 四维动作。
- 2026-11-08用户确认整机称重205g，没有新增部件，原160g不准确；重心偏高但尚未测量，不得虚构高度或把45g放在顶部。
- 参数事实源 `airgym/assets/robots/user160/parameters.json`；起飞质量、电机数据与估计值须区分。
- CAD 外形通过独立 STEP 对照；这不证明碰撞参数、重心/惯量、电机动态已标定。
- 历史 3 epoch 仅验证训练链路；4 回合评估成功率 0。未获得达标恢复策略，未完成真机部署。
- 用户确认 DM-FC01、有 SD 卡；板上固件版本、参数、实际日志能力尚待实测。
- 本地 PX4 源码在 `/home/hzq/RM/px4-autopilot`，独立仓库/分支；它不等于板上当前固件。

## 实现约束

- 碰撞交给 PhysX；不得用手写反弹冲量、接触时摆正机体、参考轨迹或隐式控制接管冒充学会恢复。
- 不能只因距离接近就算命中；碰撞后必须保留完整恢复窗口。
- 不把结构无损与姿态恢复等同；当前没有护框柔性/损坏模型。
- 不默认把 ROS1/MAVROS 例程当作已接通 ROS2；观测真值、坐标系、动作映射需单独适配。
- 训练 URDF 电机关节保持 fixed；网页 continuous 版只用于展示。
- 保留现有未提交修改、权重、原始 CAD 和报告；只修改当前功能需要的文件。
- 本轮用户授权是整理 harness；没有据此改变飞控参数、烧录固件或启动飞行。
- 普通开发使用现成 airgym_dev，禁止默认初始化时运行旧 `configuration.sh` 重建环境。

## 验证与收尾

- 静态：`python3 tools/harness.py check`；状态：`python3 tools/harness.py status`。
- 核心 harness：`python3 tools/harness.py verify H01`；模型/历史 STEP 证据完整性：`verify A01`。
- GPU 检查先激活环境：`source /home/hzq/RM/AirGym_RL/activate_airgym.sh`；再 `python tools/harness.py verify S01`（加载）、`S02`（动力学）、`S03`（接触/零恢复系数）。
- `passing` 只由上述允许列表中的验证器写入，并附带命令、退出码、输入哈希、日志哈希；证据过期则视为未验证。
- 尚未接入自动验证的功能保持 `not_started` 或 `blocked`，不能凭文档/代码存在而 passing。
- 训练目标完成须有当前机型独立评估数据及事先确定的验收条件，不能拿 reward 或旧模型成功率替代。
- 收尾更新进度、功能清单和交接；运行相应检查，说明没跑的部分和原因。不自动 commit/push。

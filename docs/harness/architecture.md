# 代码地图与接口边界

## 现有训练链路（160g旧模型，待205g更新）

`scripts/train_armor_impact.py` → `scripts/config/ppo_armor_impact.yaml` → `lib/torch_runner.py` / `lib/agent/a2c_continuous.py` → `airgym/envs/task/armor_impact.py` → `Hovering` / `rlPx4Controller` → PhysX。

| 位置 | 职责 |
|---|---|
| `airgym/envs/task/armor_impact_config.py` | 160g 机型、目标、碰撞与恢复阈值 |
| `airgym/envs/task/armor_impact.py` | 真实命中、恢复窗口、奖励/终止、30维观测 |
| `airgym/envs/base/hovering.py` | CTBR 内环接口、电机推力映射 |
| `airgym/assets/robots/user160/` | 训练 URDF、网格、参数、构建报告 |
| `airgym/assets/env_assets/arena2027/` | 已导入的蓝方基地/前哨站场景与独立装甲板 actor |
| `tools/import_arena2027.py` / `tools/build_armor_assets.py` | 场景转换、将命中面分离为独立接触对象 |
| `airgym/scripts/arena2027_preview.py` / `tools/check_arena2027.py` | 既有场景预览与固定布局/碰撞检查 |
| `tools/build_user_aircraft.py` 等 | 离线 CAD 转换与核对；训练无需重新转换 |
| `tools/patch_controller_reset.py` | 外部 rlPx4Controller 的按环境重置/可配置增益补丁 |
| `scripts/eval_armor_impact.py` | profile/维度检查、独立评估、逐回合 JSON |

策略是 actor/critic 分离 MLP `[128,128,64]`，PPO。归一化动作前三维映射到机体角速度 ±3 rad/s，最后一维映射为混控前的 0–1 collective。它不是直接的电调 PWM。

## 30维观测的部署缺口

| 切片 | 内容 | 真机所需来源 |
|---|---|---|
| 0:9 | 旋转矩阵 | PX4 姿态估计、坐标/四元数约定转换 |
| 9:12 | 目标减飞机位置 | 装甲板定位 + 同坐标系飞机定位 |
| 12:15 | 世界系线速度 | 有效的状态估计，不能假定单目已有可靠米制速度 |
| 15:18 | 世界系角速度 | 陀螺仪数据经姿态与坐标变换；不能直接塞入机体系数值 |
| 18:21 | 板法向 | 目标姿态估计 |
| 21、22 | 已碰撞、碰后时间 | 实机碰撞检测和单调时钟 |
| 23、24:28 | 要求速度、上次动作 | 任务命令和动作缓存 |
| 28、29 | 合格命中、连续稳定比例 | 可实现的机载判定；目前用仿真真值 |

当前使用的是仿真真值。尚无 ROS2 观测适配、装甲板视觉定位、推理节点、offboard 模式管理和经过验证的接管/退出逻辑。`control_for_gym` 的 ROS1/MAVROS 示例不代表这些接口已经实现。

模型机体系为 FLU（X前、Y左、Z上），PX4 常用机体 FRD/本地 NED；必须逐个量定义变换。原始 sensor_* 数据还涉及板级安装旋转，不应直接当世界坐标。

## 任务约定

固定钢板真实接触；法向速度 4.5–5.5 m/s，切向 ≤1 m/s；接触后运行 2 s，末尾连续 0.25 s 倾角 <30°、角速度 <1.5 rad/s，无触地/越界/数值失败。默认不要求停在固定位置。

模型 profile、schema、动作尺度或观测含义变化后，已有权重与评估须重新核对。`runs/armor_impact_ready.pth` 是旧 33 维版本；先前官方大机型的成功率不适用于 user160。

场景导入与装甲板分离是已有实现；S01/S03会加载当前撞击场景，但本轮没有另外复跑 `arena2027` 独立任务，不把其历史报告升级为本轮结果。

## 其他本地工程

- `/home/hzq/RM/px4-autopilot`：实际 DM-FC01 板级源码；分支/提交见硬件清单。
- `/home/hzq/RL`：Flightmare 实验环境，与本仓库 Conda/仿真器不同。
- `/home/hzq/文档/ChatGPT/New project`：早期 SB3/legged_gym 工作区，不作为本项目状态来源。

此 harness 组织已有工作，不合并这三个工程的依赖，也不修改固件算法。

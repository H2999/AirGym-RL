# 真实接触后的自主姿态恢复任务

> 2026-11-08更正：整机装配后称重205g，没有新增组件；原160g记录不准确。当前URDF/build_report仍是旧160g模型，待质量分布与重心更新。以下160g构建/验证结果保留为历史，不代表205g实机。测量方法见[重心测量](hardware/center_of_mass.md)。

当前任务：`armor_impact`，schema 为 `armor_impact_attitude_v3_user160`，观测 30 维。当前飞机为从用户 CAD 重组的 `user160_cad_v1`，总质量 160g。

目标是让策略自己学会：飞向指定装甲板 → 以约 5 m/s 实际接触 → 在撞击后的 2 秒内稳定姿态并继续飞行。碰撞后不要求返回固定坐标，不给出恢复轨迹，不施加手写反弹冲量，不切换到位置/姿态控制器接管。全程使用相同的 CTBR 接口和底层角速度内环。

## 碰撞和成功标准

- 用独立的装甲板碰撞 actor 和接触力判定命中，靠近或穿过目标坐标不算命中。
- 撞前一控制帧的板面法向接近速度须为 5±0.5 m/s，切向速度不超过 1 m/s。
- 碰撞由 PhysX 求解接触冲量、摩擦和角动量变化。可能被挡住、滑动、转动或自然弹开；成功判定不要求出现反弹。
- 命中后继续运行 2 秒，不能在接触时立即重置。末尾连续 0.25 秒倾角小于 30°、角速度小于 1.5 rad/s，且没有触地风险、越界或数值异常，才算姿态恢复成功。
- 默认 `require_low_speed=False`：稳定姿态时可以仍有水平运动，不要求悬停或回到某个恢复点。线速度有平滑的奖励项，若任务确实需要停稳，可显式启用 `require_low_speed`。
- 没有参考控制轨迹或模仿学习初始化。默认 `warm_start_probability=0`，每回合从静止开始；如自行开启接近起步课程，其只改变 reset 初始状态，不修改碰撞后状态。

任务奖励只评价接近与撞速、撞后的倾角/角速度/速度和终局成功，不规定恢复动作。环境中的 `reset_idx` 只在回合开始或结束时执行；不会在正常撞击时把飞机摆正。

## 安装与训练

当前机器的控制器独立积分重置接口已经安装。其他机器需先激活对应环境，然后运行一次：

```bash
python tools/patch_controller_reset.py
```

该补丁给 rlPx4Controller 增加按环境清零积分状态和配置角速度 PID 参数的接口。未指定参数时保持官方增益；当前 160g 机型使用单独的仿真 PI 参数，reset 后保持这些参数。否则旧的积分状态或默认增益会污染新回合。

当前电脑从随机网络训练：

```bash
source /home/hzq/RM/AirGym_RL/activate_airgym.sh
python scripts/train_armor_impact.py --ctl_mode rate --num_envs 256 --headless --epochs 2000 --seed 61
```

默认网络：独立 actor/critic，MLP [128,128,64]，PPO。归一化动作前三维映射到 ±3 rad/s，最后一维映射到内环混控前的 0–1 collective 指令，再由各电机的静态推力插值映射到牛顿。训练入口默认不加载模型。新权重保存在 `runs/ppo_armor_user160_*/nn/`，包含机型标识；继续训练只能加载该机型、相同 30 维任务的 checkpoint。

也可使用通用入口：

```bash
python scripts/runner.py --task armor_impact --ctl_mode rate --num_envs 256 --headless
```

## 评估与观看

训练后把 `CHECKPOINT` 设置为本次新生成的 `.pth` 文件路径，然后执行：

```bash
python scripts/eval_armor_impact.py --ctl_mode rate --num_envs 16 --headless --seed 77 --episodes 100 --checkpoint "$CHECKPOINT" --report runs/armor_attitude_eval.json
```

观看时去掉 `--headless`，改为 `--num_envs 1`。每个新回合相机会对准选中的装甲板。评估从静止起步，记录命中速度、接触力采样值、末端姿态/角速度/速度、失败判定及成功率。

旧版 `runs/armor_impact_ready.pth` 是 **33 维、参考轨迹初始化版本**，与当前任务不兼容。旧的 30 维官方飞机测试权重同样不适用于新机型。旧版成功率不适用于本版本，训练与评估入口检查维度和机型标识。旧初始化入口 `scripts/bootstrap_armor_impact.py` 已禁用，参考控制器代码已移除；旧文件和报告仅用于保留历史。

## 验证

```bash
python tools/check_armor_impact.py --ctl_mode rate --num_envs 4 --headless --seed 42
python tools/check_armor_impact.py --ctl_mode rate --num_envs 4 --headless --seed 42 --zero-restitution
python tools/check_user_aircraft.py --ctl_mode rate --num_envs 4 --headless --seed 64
```

检查包括：指定装甲板的真实接触、接触引起动量变化、距离近不计命中、足够长的恢复窗口、速度合格判定、姿态稳定不依赖恢复位置、仍在旋转不能成功、触地失败、部分 reset 及控制器独立重置。`--zero-restitution` 验证没有材料弹性恢复也能正常定义和判定任务。

这些是环境/训练链路检查，不代表已训练出达标策略。当前机型验证记录见 [user_aircraft.md](user_aircraft.md)。`armor_attitude_validation.md` 保留的是此前官方机型的历史检查。

## 真实程度的边界

已使用用户 CAD 的防护框、立柱及设备布局，采用 162 个凸碰撞形状，质量设为用户提供的 0.160kg。零件质量分配、质心和惯量仍是估算；电机高度、反扭矩系数、内环增益也未经过实机辨识。建筑/钢板为固定刚体，地面为平面；没有模拟护圈变形、桨叶折断、电机失效或结构损伤。

摩擦范围 0.3–0.7、恢复系数范围 0–0.35 是未标定的材料假设，并不是写好的回弹动作。控制周期 0.01 s，4 个物理子步。物理接触能够产生姿态扰动，但不能据此声称已精确复现真实撞击，更不能把未触地当作结构没有损坏。

进一步贴近实机需要测量或校准质量分布、质心、惯量、电机动态和钢板接触参数。当前观测来自仿真真值，尚未接入真实视觉识别和状态估计；实机迁移还需对应这些接口与延迟。

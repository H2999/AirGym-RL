# 装甲板撞击与恢复任务

分支：`dev`。任务名：`armor_impact`。在已有蓝方前哨站、基地的固定场景中，每个 episode 随机选择一块指定装甲板；策略使用 CTBR（`rate`），全程输出角速度和总推力；前三个归一化动作映射到 ±3 rad/s，推力动作映射到 0–1。接触时没有手动施加反弹冲量，也没有切换到位置控制器。

## 成功标准

1. 指定装甲板的独立碰撞 actor 检测到接触力，且飞机从板面正面接近。单纯距离近不计命中，碰到其他建筑部分不计命中。
2. 接触前一控制帧的板面法向接近速度为 **5 ± 0.5 m/s**，切向速度不超过 **1 m/s**。这是离散采样的速度估计，不是无限精度的接触瞬时测速。
3. 必须在前 **6 秒**内接触目标，撞击后继续物理仿真 **2 秒**。期间高度越界、触地风险、异常角速度等判失败。
4. 2 秒期限末尾连续 **0.25 秒**满足：倾角小于 **30°**、线速度小于 **0.7 m/s**、角速度小于 **1.5 rad/s**，且距板外 0.7 m 的恢复参考位置不超过 **1 m**。

速度、时限、恢复阈值和接触材料参数均在 `airgym/envs/task/armor_impact_config.py` 的 `impact` 中配置。撞击速度按法向定义，不能用高速擦边替代正面命中。

实际候选模型及测试结果见 [本地验证报告](armor_impact_validation.md)。

## 训练

首次使用本任务需要为已安装的控制器增加每个环境独立重置积分状态的接口（当前电脑已安装）：

```bash
source /home/hzq/RM/AirGym_RL/activate_airgym.sh
python tools/patch_controller_reset.py
```

补丁只增加 `reset_idx` Python 接口，不改变控制增益。任务每次重置只清零对应环境的控制器状态，防止回合间积分残留。补丁源代码随仓库提供，供其他电脑复现。

在当前电脑：

```bash
source /home/hzq/RM/AirGym_RL/activate_airgym.sh
python scripts/train_armor_impact.py --ctl_mode rate --num_envs 256 --headless --epochs 2000 --seed 42
```

64 和 256 个环境都满足当前 minibatch 配置。训练产物在 `runs/ppo_armor_impact_*/nn/`；训练日志包含 `impact/hit_rate`、`impact/valid_hit_rate` 和 `impact/success_rate`，这些是训练期间累计值，包含训练专用的接近起步样本，不能当作严格评估成功率。

训练默认有 10% 样本从距板 0.45–0.8 m、5 m/s 的接近状态起步，让策略早期接触恢复阶段；其余从距板 1.5–3 m 的静止状态起步。下述独立评估关闭所有高速起步。

### 可选：先用参考轨迹初始化，再做 PPO

从随机网络直接探索接触和恢复比较困难，可以先使用行为克隆与 DAgger 初始化网络：

```bash
python scripts/bootstrap_armor_impact.py --ctl_mode rate --num_envs 64 --headless --seed 45 --output runs/armor_impact_bootstrap.pth
python scripts/train_armor_impact.py --ctl_mode rate --num_envs 256 --headless --epochs 300 --seed 46 --checkpoint runs/armor_impact_bootstrap.pth
```

初始化会先收集参考控制器轨迹，再收集参考动作与网络动作混合、最后完全由网络执行的轨迹，拟合网络动作。参考控制器仅在这个数据生成脚本内使用；常规训练和独立评估不会调用它。参考速度设为目标的 0.92 倍以补偿有限姿态响应造成的超调，实际命中仍严格按 **5 ± 0.5 m/s** 判定。初始化还会使用演示的折扣回报预拟合独立 critic，降低随机价值估计在微调开始时破坏动作的风险。PPO 使用固定的小学习率；从 checkpoint 微调时冻结观测归一化统计，防止已初始化的输入坐标系发生漂移。初始化后的 checkpoint 是模仿学习结果，继续 PPO 后才是混合训练结果，二者应分别评估。

也可以直接使用通用入口（训练轮数取 YAML）：

```bash
python scripts/runner.py --task armor_impact --ctl_mode rate --num_envs 256 --headless
```

`--checkpoint` 可加载相同网络结构的本任务 checkpoint 继续优化。不能直接使用气球/悬停 checkpoint：观测维度不同。

## 评估和观看

下面路径是示意，必须换成实际生成的 checkpoint：

```bash
python scripts/eval_armor_impact.py --ctl_mode rate --num_envs 16 --headless --seed 77 --episodes 100 --checkpoint "$CHECKPOINT" --report runs/armor_impact_evaluation.json
```

先把 `CHECKPOINT` 设置成真实存在的 `.pth` 文件绝对路径。脚本会在打开仿真前检查文件，避免因路径错误出现黑窗重试。观看时去掉 `--headless`，将 `--num_envs` 改为 `1`。评估从静止状态起步，输出每个 episode 的命中速度、接触力采样峰值、恢复时间、失败标记和成功率，以及基地/前哨站的分组成功率。没有命中的 episode 同样计入分母。

避免同时在 8 GB 显卡上运行训练和评估。无相机的 headless 模式不初始化 Vulkan 渲染器。

## 代码位置

- `airgym/envs/task/armor_impact.py`：33 维状态观测、接触事件、奖励、2 秒恢复阶段和 episode 指标。
- `airgym/envs/task/armor_impact_config.py`：环境及任务阈值、材料随机化、PhysX 子步。
- `scripts/config/ppo_armor_impact.yaml`：参考气球任务的 PPO 配置；actor 与 critic 分离，避免价值网络的梯度直接改动已初始化的 actor。
- `scripts/train_armor_impact.py`：可指定 `--epochs` 的训练入口。
- `scripts/eval_armor_impact.py`：有界评估和 JSON 报告。
- `scripts/bootstrap_armor_impact.py`、`airgym/utils/armor_reference.py`：可选参考轨迹初始化，不参与最终评估控制。
- `tools/build_armor_assets.py`：从已有模型拆分独立装甲板 actor，原模型不修改。前哨站为正面的 `ArmorPanel`，基地为顶部正面的 `SmallArmor`；同板表面的灯条/覆盖件归入相同接触 actor，避免遮挡造成误判。
- `tools/check_armor_impact.py`：GPU 物理集成检查。

重新生成碰撞资产和检查：

```bash
python tools/build_armor_assets.py
python tools/check_armor_impact.py --ctl_mode rate --num_envs 4 --headless --seed 42
```

## 仿真适用范围

- 当前仍使用官方 X152b：机身碰撞几何是半径 **0.2 m 的球**，不是实机护圈模型；模型质量和惯量也未替换为用户飞机。
- 建筑及装甲板固定，支持 PhysX 刚体接触、摩擦和恢复系数产生的回弹；没有旋转装甲板、弹簧安装结构、材料变形、桨叶损伤或电池碰撞损伤。
- 每个环境随机设置恢复系数 0.05–0.35、摩擦 0.3–0.7，数值是训练假设，未经实物辨识。控制周期 0.01 s，物理 4 子步，接触力数值不能解释为实物可承受载荷。
- 为避免向量环境中的全局地形网格相互覆盖，训练场景使用平地，不含原预览场景的 1.5° 坡面。
- 和气球例程一样，观测来自仿真真值，含目标相对位置、姿态、速度、板面法向、碰撞阶段、恢复参考位置和上次动作。没有装甲板视觉检测；真机还需替换为可测量的状态及碰撞事件估计。
- “不炸机”在代码中只能转化为上述仿真成功指标；通过检查、奖励增加或完成若干 epoch 都不等于策略达标，也不能保证真机结构和动力系统不损坏。需要独立评估报告判断模型是否学会任务。

参考控制器使用官方 X152b 的总质量 0.601 kg 和 4×9.59 N 推力模型。替换自己的飞机后必须同步更新这些参数，不能把这套初始化直接当作自己的实机控制器。

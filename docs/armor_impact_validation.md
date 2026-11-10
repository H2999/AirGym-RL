> 历史报告：以下为旧版 33 维、参考轨迹初始化任务的结果，不适用于当前 30 维纯 PPO 姿态恢复任务。当前状态见 `armor_attitude_validation.md`。

# 本地装甲板任务验证结果（2026-11-02）

候选策略：`runs/armor_impact_ready.pth`。该模型先使用参考轨迹进行行为克隆/DAgger 初始化，再完成 30 轮 PPO 微调。评估全程由神经网络给出 CTBR 动作，没有参考控制器接管；高速训练起步已关闭，每回合从静止开始，并独立清零底层控制器积分状态。

- 种子 321，100 回合：命中率 97%，速度合格率 97%，完整成功率 89%，失败判定率 1%；命中平均法向速度 5.283 m/s。
- 种子 77，10 回合：命中率 100%，速度合格率 100%，完整成功率 100%，失败判定率 0%；命中平均法向速度 5.270 m/s。

100 回合测试使用 16 个并行环境；10 回合检查使用单环境，对应观看方式。完整成功要求 5±0.5 m/s 的实际板面接触、之后生存 2 秒，并在期限末尾连续 0.25 秒满足配置中的稳定条件。未命中、撞速不合格、恢复超时和触地等失败均计入分母。完整逐回合数据保留在 `runs/armor_reset_ppo_eval321.json` 与 `runs/armor_reset_ppo_single77.json`，摘要及模型 SHA256 见 `armor_impact_validation.json`。

已通过 GPU 集成检查：指定环境的控制器积分重置不影响其他环境；前哨站/基地的真实接触；约 5 m/s 接近后的正向回弹；无接触不计命中；不合格速度不计成功；完整 2 秒时限；触地失败；部分重置与观测一致性。测试反弹法向速度约 0.17–0.67 m/s，仅对应这组刚体材料假设。

这些结果说明任务和初版策略能够运行，不代表达到 100% 可靠性，也不保证真机不会损坏。当前模型是官方 X152b，尚未辨识用户飞机的质量、惯量、碰撞几何和缓冲结构；装甲板固定，观测来自仿真真值。更早的试训与评估可能使用旧版控制器状态重置、共享网络或不同动作映射，不能与本报告混用。

观看：

```bash
source /home/hzq/RM/AirGym_RL/activate_airgym.sh
python scripts/eval_armor_impact.py --ctl_mode rate --num_envs 1 --seed 77 --episodes 10 --checkpoint runs/armor_impact_ready.pth --report runs/armor_watch.json
```

继续训练：

```bash
python scripts/train_armor_impact.py --ctl_mode rate --num_envs 256 --headless --epochs 300 --seed 53 --checkpoint runs/armor_impact_ready.pth
```

继续训练后必须重新评估，奖励上升或更多 epoch 不保证策略优于此候选；本候选文件不会自动被后续训练覆盖。

# 开发与验证命令

当前实机称重已更正为205g，旧模型仍为160g。S01–S03已标记待更新，验证入口会拒绝质量不一致；旧训练命令仍运行旧模型，不应当作205g训练结果。重心测量见 `docs/hardware/center_of_mass.md`。

## 每次开工

```bash
cd /home/hzq/RM/AirGym_RL/AirGym
bash init.sh
python3 tools/harness.py status
```

`HARNESS_PYTHON` 可选择 stdlib 检查用的 Python。GPU 操作另外激活：

```bash
source /home/hzq/RM/AirGym_RL/activate_airgym.sh
python -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
```

已知环境为 Python3.8 / PyTorch2.0.0+cu118 / Isaac Gym Preview4 / RTX4060 Laptop。不要用系统 Python3.10 替代 GPU 环境，也不要从此 harness 自动执行 `configuration.sh`。

## 验证分层

| 命令 | 证明什么 | 不证明什么 |
|---|---|---|
| `python3 tools/harness.py verify H01` | 状态与证据规则、试验模板、代码地图检查可运行 | 仿真/真机可飞 |
| `python3 tools/harness.py verify A01` | URDF引用/质量/固定关节、历史STEP报告与当前URDF哈希一致 | 重新执行精确CAD核对、材料已标定 |
| `python tools/harness.py verify S01` | 当前机器无窗口加载用户飞机 | 学会飞行 |
| `python tools/harness.py verify S02` | 质量、静推力、角速度响应、真实接触的动力学检查 | 真机增益正确或结构无损 |
| `python tools/harness.py verify S03` | 正常/零恢复系数下的接触与恢复判定 | 策略成功率 |

H01/A01 是 CPU/stdlb；S01–S03 使用当前激活的 GPU Python。S02 原脚本会刷新 `docs/user_aircraft_dynamics_validation.json`，harness 另存本轮 stdout 和输入哈希。没有运行时就保持未验证，不用旧截图代替。

每次验证的日志和 JSON 在 `runs/harness/<UTC时间>-<唯一ID>/`。`feature_list.json` 记录证据指针；修改相关输入或删除证据后，`check` 报告 stale/missing，需重新验证。运行其他机器时，本地忽略目录不随 git 自动迁移。

哈希覆盖的是各验证项声明的项目文件；不覆盖操作系统、GPU驱动或外部rlPx4Controller安装包。环境/外部控制器变更后必须主动重跑S01–S03，不能仅依赖文件检查。初始化打印 passing 表示存在仍匹配这些项目文件的执行证据，不是在每次开工时重新运行GPU检查。

## 预览

```bash
python scripts/view_user_aircraft.py
python scripts/view_user_aircraft.py --scene
```

纯静态模型展示；`--headless` 执行一次加载后退出。展示网页版本：

```bash
python tools/export_user_aircraft_viewer.py --output /tmp/user160_web
```

导入整个文件夹至 https://viewer.robotsfan.com/；continuous 电机关节仅作运动学显示，不替换训练 URDF。

## 训练与独立评估（显式运行）

短链路检查：

```bash
python scripts/train_armor_impact.py --ctl_mode rate --num_envs 64 --headless --epochs 3 --seed 61
```

正式实验例子：

```bash
python scripts/train_armor_impact.py --ctl_mode rate --num_envs 256 --headless --epochs 2000 --seed 61
```

从终端输出取本次生成的实际 checkpoint 路径，设置 `CHECKPOINT` 后评估：

```bash
test -f "$CHECKPOINT"
python scripts/eval_armor_impact.py --ctl_mode rate --num_envs 16 --headless --seed 77 --episodes 100 --checkpoint "$CHECKPOINT" --report runs/user160_eval_seed77.json
```

不要原样输入中文占位路径。报告 checkpoint SHA、profile、schema、每种目标命中/成功率及逐回合结果；跨 seed 与工况验证。尚未约定正式策略的最低成功率门槛，因此训练完成或单次高 reward 不会自动把 T01 标为 passing。

## 数据采集

```bash
python3 tools/harness.py new-trial --id hover_001
python3 tools/harness.py trial-check data/captures/hover_001
```

新目录中填写 `trial.json`，放入固件版本记录、参数、原始 ULog、视频和同步记录；缺失数据时检查必须失败。该检查只证明元数据/文件齐全，不证明传感器采样率、无丢包或拟合质量。

## 收尾

运行对应验证和 `python3 tools/harness.py check`；检查 `git diff --check`；更新进度与交接。基线已有大量未提交工作，不能用 `git reset/clean` 获取所谓干净状态。未经本次授权不自动提交、推送、刷机或开始实机试飞。

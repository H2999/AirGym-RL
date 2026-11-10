# Harness 使用入口

这是 AI 开发工作的上下文、状态和验证入口。飞控接线与飞行状态机仍是独立工程内容。

```bash
cd /home/hzq/RM/AirGym_RL/AirGym
bash init.sh
python3 tools/harness.py status
```

最小工作循环：读状态 → 选择一个功能 → 实现 → 运行指定验证 → 保存证据 → 更新交接。

| 文件 | 用途 |
|---|---|
| `AGENTS.md` | 新会话先读的入口和约束 |
| `feature_list.json` | 功能范围、优先级、验收步骤、状态 |
| `PROGRESS.md` | 项目事实、历史结果和本轮记录 |
| `session-handoff.md` | 下一轮应从哪里继续 |
| `tools/harness.py` | 初始化、证据检查、受限验证入口、试验目录工具 |
| `docs/harness/architecture.md` | 代码地图与真机接口缺口 |
| `docs/harness/runbook.md` | 环境、预览、验证、训练、评估命令 |
| `config/hardware/dm_fc01.json` | 手册/源码证据，板上实际值留空 |
| `docs/hardware/data_capture.md` | 日志与视频采集流程 |

`verify` 只接受内置命令，不执行 JSON 中任意 shell。失败/超时会保留日志并把该项置为 `blocked`；成功后保存输入与日志 SHA256。`check` 会拒绝没有证据的 passing、多个进行中功能、已变更输入或被删除的日志。此机制是本地流程约束，不是防篡改安全边界。

GPU 验证是显式操作；初始化不安装依赖、不启动长训练、不访问串口。原始试验数据和运行日志放在忽略的 `data/captures/`、`runs/harness/`，需要长期共享时另行保存，并连同哈希保留。

设计参考用户指定的 [Learn Harness Engineering](https://walkinglabs.github.io/learn-harness-engineering/zh/)，主要采用：

- [仓库事实来源](https://walkinglabs.github.io/learn-harness-engineering/zh/lectures/lecture-03-why-the-repository-must-become-the-system-of-record/)：把聊天事实落入仓库。
- [分层指令](https://walkinglabs.github.io/learn-harness-engineering/zh/lectures/lecture-04-why-one-giant-instruction-file-fails/)：入口保持短，硬件/运行细节按需读取。
- [功能清单](https://walkinglabs.github.io/learn-harness-engineering/zh/lectures/lecture-08-why-feature-lists-are-harness-primitives/)：用可执行验证控制 passing。
- [初始化](https://walkinglabs.github.io/learn-harness-engineering/zh/lectures/lecture-06-why-initialization-needs-its-own-phase/)与[交接](https://walkinglabs.github.io/learn-harness-engineering/zh/lectures/lecture-12-why-every-session-must-leave-a-clean-state/)：每轮可恢复。

针对现有 CUDA/Isaac Gym 环境，初始化采用检查而非模板中的每次重装依赖；没有引入自动迭代代理或自动飞行循环。

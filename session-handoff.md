# 会话交接：2026-11-08

## 当前已验证与证据入口

查看 `feature_list.json`，然后执行 `bash init.sh`。passing 项须有本轮执行记录、输入哈希和日志；证据缺失或输入变更时检查失败。历史几何/动力学/训练链路报告见 `PROGRESS.md`，不能代替当前运行结论。

## 本轮范围

整理已有 AirGym user160 工程的开发 harness 与 DM-FC01 实测数据入口。未调整训练奖励/网络、未替换物理模型、未改固件/板上参数、未启动实机飞行。保留所有原有未提交修改。

## 已完成的整理

- `AGENTS.md`：项目入口、约束与收尾规则。
- `tools/harness.py` / `init.sh`：静态检查、允许列表验证、日志/输入哈希、试验目录与缺失数据检查。
- `docs/harness/`：代码地图、观测接口缺口、运行与验证说明。
- `docs/hardware/` / `config/hardware/`：DM-FC01手册与本地源码证据、板上待测字段、数据采集流程。
- `data/templates/impact_trial.json`：真实视频/日志/固件/参数/同步元数据模板；默认不会创建假原始数据。

## 仍未验证或待做

板上PX4版本与参数、SD实际记录速率、IMU来源/削顶、视频同步、质心/惯量、电机动态和接触参数；ULog内容审计器、参数拟合、达标策略、ROS2部署均未完成。

## 下一轮最佳动作

请用户按 `docs/hardware/dm_fc01.md` 导出 `ver all`、参数和首段原始ULog。先做日志质量审计，随后辨识普通飞行动力学，再处理真实接触。

## 快速命令

```bash
cd /home/hzq/RM/AirGym_RL/AirGym
bash init.sh
python3 tools/harness.py status
python3 tools/harness.py new-trial --id hover_001
source /home/hzq/RM/AirGym_RL/activate_airgym.sh
python tools/harness.py verify S01
```

不要自动commit/push、不要清理旧工作、不要把网页continuous URDF用于训练。GPU验证默认无窗口；GUI此前影响过外接显示器的操作，不为检查harness主动打开。

## 本轮复验结论

H01（8项测试）、A01（模型/历史报告完整性）、S01（加载）、S02（动力学）、S03（两种接触配置）通过，具体日志见功能清单。空白试验目录检查按预期失败，未伪造原始数据。未新训练或进行实机验证。

## 最新更正优先于上方历史状态

实机装配称重205g，未新增组件，原160g错误；重心未测。参数记录已更正，仿真URDF/build_report仍160g。先指导/取得重心及部件质量数据，再做模型更新；不要把45g当顶部新增点质量。S01–S03已置blocked保留旧证据，质量一致性检查阻止旧模型误验。新增测量指南 `docs/hardware/center_of_mass.md`。

205g记录更新后：H01的9项行为测试及A01历史资产完整性检查通过，静态初始化/差异空白检查通过；未重新运行旧160g模型的GPU检查，也未声称205g动力学已验证。

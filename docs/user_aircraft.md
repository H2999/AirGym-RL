# 用户飞机参数与模型替换状态

> 2026-11-08更正：整机装配后称重205g，没有新增组件；原160g记录不准确。当前URDF/build_report仍是旧160g模型，待质量分布与重心更新。以下160g构建/验证结果保留为历史，不代表205g实机。测量方法见[重心测量](hardware/center_of_mass.md)。

2026-11-02 用户给出：起飞重量 160g，横向/纵向电机间距 81/60mm，1202.5 11500KV 电机配 GF2023 桨叶；7.4V 测试条件下，50% 油门拉力 66g、100% 油门拉力 127g。目标是固定钢板。这里的拉力单位按克力处理。

数据保存在 `airgym/assets/robots/user160/parameters.json`。**当前 `armor_impact` 已加载用户飞机的 URDF**：`airgym/assets/robots/user160/model.urdf`，profile 为 `user160_cad_v1`。其他官方任务仍保留原模型。

## 数据的解释

结合用户的力效表，将拉力作为单电机 GF2023、7.4V 测试数据使用；81/60mm 作为左右/前后电机中心间距，与 CAD 的安装孔布局相符。

- 单电机最大静推力约 1.245N，四电机总计约 4.982N，推重比约 3.175。
- 水平悬停所需单电机拉力约 40 克力。仿真采用 (0,0)、(50%,66gf)、(100%,127gf) 分段线性插值，0 油门推力为 0 是假设；静态悬停指令约 30.3%。该值取决于插值模型，不能当作实机已测得的悬停油门。
- 以机体 x 向前、y 向左、中心对称矩形布局表示，电机中心为 x=±30mm、y=±40.5mm；轴距不等于防护框外尺寸。
- 160g 飞机以 5m/s 飞行时的平动动能为 2J。这不用于推断结构能否承受撞击。

电机外形按厂家直径 15.8mm、高 8.9mm 和每个 4.5g 加入；四个螺旋桨已改为直径约 50.8mm 的三叶显示网格，并提供镜像旋向。叶片弦长、后掠与扭转均为示意，未声称是 GF2023 的精确叶型。该改动仅影响显示，未改变质量、惯量、碰撞体和实测推力映射；未模拟逐叶片气动、叶片损伤或桨盘相互作用。CAD 原装配体没有电机和桨叶零件，电机高度及旋转方向仍为估计。

## CAD 转换与核对

原文件目录：`/home/hzq/下载/3下置无人机`，装配体 `无人之机.SLDASM`。使用其保存的组件 XML 变换矩阵，重组 13 种零件、34 个实例；没有重新求解 SolidWorks 装配约束。显示网格来自零件文件，单位为米。解析器对部分未解码元数据有警告，数量保存在报告中；未把这些警告解释为完整格式支持。

装配网格的 CAD XYZ 尺寸约 139.2×118.2×60mm，与装配文件包围框的最大差异为 0.224mm。另以两份原始 STEP 核对对应板件：包围尺寸差异小于 0.00001mm，网格体积误差分别约 0.43% 和 0.51%。因此保留的是 CAD 的显示网格近似，不是解析曲面的无损转换。

随后使用用户导出的完整 `无人之机.STEP` 独立核对。OpenCascade 检查拓扑有效，包含 40 个实体、1735 个面。将 STEP 按 0.02mm 弦差三角化，与 URDF `base_link` 中实际引用的全部 CAD 可视网格做双向最近表面距离比较，检查全部网格顶点及每方向 8192 个表面采样点。核对报告为 [user_aircraft_assembly_step_validation.json](user_aircraft_assembly_step_validation.json)。检查通过，外形包围尺寸差异约 0.003mm，最大检查点表面距离约 0.20mm，99% 检查点的距离小于 0.08mm；未发现需要修正的装配位置差异。

该核对不包含额外生成的参数化电机/桨盘，也不能用来证明凸碰撞体、质量分布或接触材料已经标定。

机体系为 X 向摄像头所在的 CAD +Y 方向、Y 向 CAD -X、Z 向上。机体原点放在估算的整机重心。模型包含 `base_link` 和 `prop_1`～`prop_4`，保留现有施力索引。

防护框板件使用 CoACD 凸分解，电子器件和立柱使用单个凸包。当前共 162 个碰撞形状，避免把整机直接包成一个大球或一个大凸包。凸分解会近似细小孔洞和边缘；这些碰撞几何仍需与实物接触位置核对。

完整尺寸、零件来源哈希、实例位置、质量估算及惯量见 `airgym/assets/robots/user160/build_report.json`。预览为同目录 `cad_conversion_preview.png`。

## 质量、控制与接触的估计

总质量按用户实测设为 0.160kg；四电机各 0.0045kg，其余 0.142kg 按报告中明确列出的相对质量假设分配。零件质量分布未实测；质心、完整惯量张量因此是估算，不能声称是 CAD 已标定的物理属性。闭合网格采用体积分布，两个非闭合零件采用包围盒分布近似惯量。

模型需要将接触几何放在实际重心周围。接触冲量对重心产生的角动量变化与接触位置有关；只改质量、继续使用官方大球形碰撞体，不能代表图中的防护框撞击。

固定钢板使用固定刚体；钢材和固定安装本身不能确定整套机架与钢板接触时的恢复系数、摩擦或结构柔性。这些参数仍需标定。

当前两个装甲板命中面均使用 `ArmorImpactCfg.steel_plate` 的固定刚体配置，并显示为灰色金属外观；`friction_range=[0.3,0.7]`、`restitution_range=[0,0.35]` 为未标定的接触不确定范围。与原先范围相同，未因改名为钢板就假设特殊回弹。范围采样会与飞机材料共同决定接触响应，没有使用钢材弹性模量来求解结构变形。需要模拟钢板柔性或损坏时，还需厚度、钢种和安装约束等数据及相应结构模型。

新任务已替换官方每电机 9.59N 的推力缩放。反扭矩/推力比暂设为 0.005m，未测量；尚未模拟电机动态延迟、电池电压下降和气动阻力。角速度 PI 依据估计惯量调整到能通过仿真检查的参数，不代表用户飞控的真实增益。该内环在整个回合持续运行，不在撞击时接管策略。

接触由 PhysX 求解，没有手动反弹冲量或碰撞后重置。固定钢板采用固定刚体；没有建模防护框柔性、断裂、桨叶破损和电机损伤。

## 重建与验证

只查看飞机外形（物理暂停，不需要训练权重）：

```bash
source /home/hzq/RM/AirGym_RL/activate_airgym.sh
python scripts/view_user_aircraft.py
```

默认只创建一架飞机，相机近距离对准机体。关闭窗口或按 Esc 退出。`--headless` 可用于无窗口检查加载。

同时查看飞机和钢板场景：

```bash
python scripts/view_user_aircraft.py --scene
```

三叶桨显示网格可单独更新，无需重新进行机架凸分解：

```bash
python tools/update_user_aircraft_propellers.py
```

离线转换使用 [solidworks-converters](https://github.com/blussyya/solidworks-converters)，Apache-2.0，版本 `0ef50223d5884d378fb0cf1f44f017adfd1f35ba`。几何生成工具不在训练时执行。

```bash
node tools/extract_user_aircraft.cjs --reader /path/to/solidworks-converters --source /path/to/3下置无人机 --output /tmp/user160-cad
# 另一个离线 Python 环境，需要 numpy/trimesh/scipy/coacd；无需改训练环境。
python tools/build_user_aircraft.py --cad /tmp/user160-cad --output airgym/assets/robots/user160
# 可选的独立整机 STEP 核对，另需 cadquery-ocp 和 rtree。
python tools/check_user_aircraft_step.py --step /path/to/无人之机.STEP
source /home/hzq/RM/AirGym_RL/activate_airgym.sh
python tools/patch_controller_reset.py
python tools/check_user_aircraft.py --ctl_mode rate --num_envs 4 --headless --seed 64
python tools/check_armor_impact.py --ctl_mode rate --num_envs 4 --headless --seed 42
```

基本动力学、实际接触、零恢复系数接触和 3 轮 PPO 训练/保存检查已通过。动力学结果见 `user_aircraft_dynamics_validation.json`，独立 STEP 对照见 `user_aircraft_step_checks.json`。尚未训练出达标的撞击恢复策略；旧飞机模型的权重不适用于本机型。

## 是否需要 SolidWorks 力学仿真

训练撞击后的姿态恢复可以先使用当前刚体接触环境，结构有限元不是启动 PPO 的前置要求。但当前模型不预测机架、立柱、桨叶或连接件的结构失效。

优先为真实装机组件指定材料或实测质量，再从“工具 → 评估 → 质量属性”导出总质量、重心、惯量矩阵及对应坐标系和单位。[SolidWorks 质量属性说明](https://help.solidworks.com/2022/English/SolidWorks/sldworks/HIDD_MASSPROPERTY_TEXT_DLG.htm)。电池、电路板等简化实体不应直接用均匀实体材料质量替代实际组件质量；仅覆盖整机总质量也不会提供可靠的内部质量分布。

若需判断 5m/s 撞击是否导致变形或破坏，应进行包含接触、实际材料和安装约束的动态冲击分析。SolidWorks 的跌落测试研究求解零件/装配体与刚性或柔性平面的动态碰撞，可分析应力、位移和接触力；具体模型适用性仍需核对。[SolidWorks 跌落测试说明](https://help.solidworks.com/2026/English/SolidWorks/cworks/c_Drop_Test_Studies.htm)。单张静态应力图不能直接导入 Isaac Gym 变成经过标定的柔性碰撞模型。

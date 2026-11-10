# 撞击数据采集与后续辨识

目标是让同一初始状态、同一控制输入下，仿真与真实运动接近。当前只有采集规范和目录检查器；没有声称已实现视频跟踪、ULog 自动质量审计或参数自动拟合。

## 先采一段基线日志

1. 按 `dm_fc01.md` 保存板上版本、logger 状态与参数。核对实际使用的电池、起飞质量、传感器 device_id。
2. 静止记录一段日志，检查是否能下载、能解析、包含需要的主题。板上未解锁时默认可能不记录，先核对实际 `SDLOG_MODE` 与 logger 状态，不假设文件一定已生成。
3. 再采常规悬停日志，观察角速度是否跟随设定、传感器是否正常、存储是否掉数据。
4. 只有采集链路确认后，才逐步覆盖任务撞速/姿态；记录失败样本，不只挑选成功得分样本。

PX4 的 [Logging](https://docs.px4.io/main/en/dev_log/logging) 与 [Flight Review](https://docs.px4.io/main/en/log/flight_review) 可供参考；具体主题和配置行为以下述固定版本源码为准。

## 日志配置必须按版本核对

本地 PX4 `dcc6516e3a...` 的 `src/modules/logger/logged_topics.h` / `module.yaml`：默认 bit0，系统辨识 bit3，高频 bit4，原始 gyro/accel FIFO bit8/9。因此默认+系统辨识+高频的掩码为 25，但这里只是该源码的计算示例，未向飞控应用。

更关键的是该提交的 `initialize_logged_topics()`：如果 SD 卡 `/etc/logging/logger_topics.txt` 成功读到主题，会使用文件集合，跳过配置 profile 的集合。不能假定二者追加。其他 PX4 版本可能不同。

`config/logging/dm_fc01_logger_topics.example.txt` 是依照该源码列出的候选完整采集集合，不是可直接覆盖 SD 卡的指令。字段为 `topic interval_ms instance`，0表示不额外限速，仍受主题发布率、logger 和存储限制。保存原配置并核对实际固件后再配置；日志里没有实际发布的实例不会凭空生成。

## 要记录什么

| 数据 | 主题 / 来源 | 用途与限制 |
|---|---|---|
| 机体角速度、姿态、加速度 | vehicle_angular_velocity / vehicle_attitude / vehicle_acceleration | 碰前后运动、恢复过程；滤波会改变冲击波形 |
| 原始传感器 | sensor_accel / sensor_gyro，分别记录实例和device_id | 识别削顶、板级旋转、IMU来源 |
| 选中IMU和质量状态 | sensor_selection / vehicle_imu_status / sensor_combined | 双IMU不等于都在控制，也不等于全部已记录 |
| 角速度与力矩/推力设定 | vehicle_rates_setpoint / vehicle_torque_setpoint / vehicle_thrust_setpoint | 区分碰撞扰动和控制修正 |
| 电机指令与分配 | actuator_motors / actuator_outputs / control_allocator_status | 指令不是实测RPM/推力 |
| 电池与ESC | battery_status / esc_status | 电压下跌、若存在则记录转速 |
| 位置速度 | vehicle_local_position / vehicle_odometry | 同时保留valid/reset状态；撞击时估计可能失真 |
| 模式、接管 | vehicle_status / vehicle_control_mode / input_rc / manual_control_setpoint | 标记控制阶段 |

后续质量检查应统计：每主题实际采样间隔/长缺口、ULog dropout、IMU clipping/error、device_id对应、时间戳、估计有效性。不能只因为文件能打开就开始拟合。高率 FIFO 对单/双IMU覆盖和存储负载需另行确认，不默认全开。不能随便改变控制滤波/IMU配置来改善日志。

## 视频与同步

固定相机，尽量高帧率、短曝光；提供已知尺度，保证参考物与主要运动平面关系明确。机身设置至少两个可辨标记；侧面单机位无法充分观测出平面运动，条件允许时双机位并标定。

原始视频保留真实帧时间：手机慢动作的播放fps不一定等于采集fps，记录二者和是否发生丢帧/插帧。240fps的采样间隔约4.17ms，5m/s时移动约20.8mm，不保证能分辨完整接触过程或峰值力。

使用视频与日志都能定位的共同事件同步，例如由有时间记录的机载输出控制可见LED。记录偏移、时钟漂移和同步误差；只用碰撞峰值粗对齐时明确误差，不预设零延迟。

## 每次试验的数据包

```bash
python3 tools/harness.py new-trial --id hover_001
# 填写生成的 trial.json，放入实际文件后：
python3 tools/harness.py trial-check data/captures/hover_001
```

目录模板包含 `flight.ulg`、`video.mp4`、`firmware.txt`、`parameters.params`、`sync.json` 的预期路径，不创建假原始文件。`trial.json` 中的未知值保留 null，不能填0冒充测量。原始文件不覆写、不上传；另保留副本。

## 从实测到仿真：后续待实现

先固定已测质量/几何，校准普通飞行的电机响应与控制链路，再重放实测输入拟合撞击。比较碰前后线速度/角速度、接触后的姿态和恢复时间；用不同速度、接触位置与角度的数据作留出验证。

可拟合模型中的有效摩擦/恢复响应和延迟，但通常不能由一次视频唯一分离所有刚度、阻尼和材料系数。若要反推刚度或峰值接触力，需要更高时间分辨率、变形观测或力传感数据；IMU饱和后不能按读数直接计算峰值力。

拟合结果应包含参数范围、哪些参数固定、数据哈希、拟合误差与留出误差，再据此设置域随机化。不能拿未标定范围当实测结论，也不能用姿态恢复代表结构没损坏。

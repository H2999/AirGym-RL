# DM-FC01：硬件、源码和实际固件分别记录

核对日期：2026-11-08。用户确认使用 DM-FC01 且有 SD 卡；没有提供板上 `ver all`、参数导出或原始 `.ulg`。当前不把任何源码默认值当作板上实测值。

## 证据来源

- 用户手册：`/home/hzq/下载/DM-FC01飞控板用户手册V1.0.pdf`，V1.0 / 2026-04-17；SHA256 见 `config/hardware/dm_fc01.json`。这里只提取项目相关事实，不复制手册全文。
- 用户提供的 [Gitee 仓库](https://gitee.com/kit-miao/dm-fc01)：公开目录有手册、3D 模型及固件成品；[PX4目录](https://gitee.com/kit-miao/dm-fc01/tree/master/%E5%9B%BA%E4%BB%B6/PX4)列出 V1.14/V1.15/V1.16 的 `.px4`/`.elf` 与 bootloader。这不是已核对的完整 PX4 源码工程。
- 本地源码：`/home/hzq/RM/px4-autopilot`，远程 `https://github.com/H2999/damiao_px4.git`，分支 `devb`，观察提交 `dcc6516e3a755cca68c7c8a4bcf2fcaccf3e9df9`。
- 板级目录：`boards/damiao/dm-fc01`；编译目标 `damiao_dm-fc01_default`。
- 本地存在 `build/damiao_dm-fc01_default/damiao_dm-fc01_default.px4`，嵌入 git identity `dcc6516e3a`、board_id 7140。存在构建产物不代表板上已刷入；包内 `version: 1.0.0` 也不能单独当作 PX4 发布版本。

## 与采集相关的能力

| 项目 | 手册 / 本地源码证据 | 需要真机确认 |
|---|---|---|
| MCU | STM32H743，手册 p3 | 运行版本、负载 |
| 双IMU | BMI088 + ICM45686，手册 p3–4、p11–12；`rc.board_sensors` 均启动 | 实际发布的 device_id、实例、选中传感器 |
| 存储 | microSD / SDMMC1，手册 p13；板级 defconfig 启用 SDMMC/FAT | 卡挂载、写入、日志丢包 |
| 电机输出 | 8路 PWM/DShot，手册 p11；板配置含 DShot | 实际协议、顺序、旋向、ESC转速遥测 |
| 机载通信 | 板配置含 MAVLink 和 uXRCE-DDS client | 当前选用协议、端口、消息版本/频率 |

物理口与逻辑口注意：手册 p9 中，USART6 的物理 TELEM4 是 PX4 `URT6` / `/dev/ttyS5`；PX4 `TELEM4` / `/dev/ttyS6` 对应 UART7 的 ESC Telemetry。不能仅凭“TELEM4”名字接线或选参数。GPS1 `/dev/ttyS0`，TELEM1 `/dev/ttyS1`，TELEM2 `/dev/ttyS2`，TELEM3 `/dev/ttyS3`，RC `/dev/ttyS4`，DEBUG `/dev/ttyS7`。

## 已发现的默认值差异

`init/rc.board_defaults`：`BAT1_N_CELLS=4`，而用户电机数据使用 7.4V/2S；需导出板上参数核对。`set-default` 可能被已保存参数覆盖，此处不是认定实际配置错误，也没有更改任何参数。

该文件还设置 `IMU_GYRO_RATEMAX=2000`、若干 MC_*RATE_K、`UXRCE_DDS_CFG=201`。这些只证明本地默认值存在，不证明真实响应、串口使用情况或消息兼容性。

## 第一轮只读检查

通过 QGroundControl 的 MAVLink Console，记录以下输出到试验目录 `firmware.txt` / `board_status.txt`：

```text
ver all
logger status
uorb top
listener sensor_selection -n 1
listener vehicle_imu_status -n 1
param show SDLOG_*
param show IMU_GYRO_RATEMAX
param show BAT1_N_CELLS
```

命令支持情况以板上固件为准；不存在的命令也记录错误。参数另通过 QGC 导出为 `parameters.params`，保存完整配置。先验证普通静止/悬停日志，再设计撞击采集。

详细采集流程见 `data_capture.md`。本轮没有连接飞控、配置日志、刷写固件或验证 ROS2 offboard。

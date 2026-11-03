# 时延分支：SSS/PX4 动力学验证结果（2026-11-03）

在 experiment/communication-delay@05f4bd3 上接入此前验证的 swarm_sync_sim 后端。三辆麦轮车、六架带 PX4 控制模块的四旋翼，初值 A，控制参数、拓扑与限幅不变。signal_timeout=false；定位、监督器、FCU、记录器与控制截止时间保护保留。SSS 上游固定 4837b51。

随机工况：均值参数 250 ms、标准差参数 50 ms、截断 0–500 ms；种子 11/29/47，每次正式段 80 秒。时延按 ROS 仿真时间注入。

| 尝试 | 完整80秒 | 领导者 NE 末段最大误差(m) | 编队位置末段最大误差(m) | 编队速度末段最大误差(m/s) |
|---|---|---:|---:|---:|
| baseline_no_delay（首次） | 否 | -- | -- | -- |
| normal250_seed11 | 是 | 8.60323e-05 | 0.000901509 | 0.000260879 |
| normal250_seed29 | 是 | 8.49629e-05 | 0.00108471 | 0.000369345 |
| normal250_seed47 | 是 | 8.30177e-05 | 0.000968274 | 0.000346078 |
| baseline_no_delay（复核） | 是 | 1.47325e-07 | 0.00114269 | 0.000288632 |

误差取最后 10 秒、全部对应实体的最大范数。首次基线在未来 t0 尚未到达时，探测端报告 `Supervisor heartbeat became stale`，没有正式控制样本；此失败原样保留。没有把失败记作收敛，也没有放宽心跳阈值。独立复核的 src/ 控制源码哈希与首轮完全一致；两次结果均保留。

三种随机工况均完成指令 1–7、降落后上锁和九实体遥测/rosbag 完整性校验。正式 UAV 输入掩码均为 2111（纯加速度），通信时钟为 ROS，最大采样间隔约 20 ms。实际生成的时延均值约 249.830–250.100 ms、标准差约 50.083–50.239 ms。三组最高使用数据年龄均约 360 ms，确认未恢复 300 ms 邻居过期判定。

末段最差编队位置误差约 1.085 mm、速度误差约 0.369 mm/s、NE 误差约 0.0860 mm。它们对应真值反馈与当前轻量动力学条件，不是实机精度，也不是任意时延下稳定性证明。

103 项单元测试通过，原 mock 后端 12 秒完整流程兼容性回归也通过。main@70f5fb0 与 PX4_sim@b916da3 保持不变，数学 core.py 未改动。完整五次尝试、配置和原始 rosbag 哈希见 [validation/delay_sss_20261103.json](validation/delay_sss_20261103.json)。

## 本机原始结果

- 首轮四次：`/home/bjl/xgc_ws/delay_px4_experiment_results/20261103_165520_sss_normal250_no_timeout`。
- 独立基线复核：`/home/bjl/xgc_ws/delay_px4_experiment_results/20261103_170232_sss_baseline_repeat_no_timeout`。
- 容器内将 `/home/bjl/xgc_ws` 换为 `/home/xgc_ws`。各工况目录含实际 YAML、smoke-report.json、ROS 日志、output/<session>/ 原始遥测与 bag、picture/<session>/ 13 幅图和 metrics.json。

[构建、手动启动与批量复现说明](DELAY_PX4_TEST.md)。

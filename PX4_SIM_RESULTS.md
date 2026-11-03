# PX4_sim 接入验证（2026-11-02）

环境：现有 xgc / ROS Noetic。SSS 上游固定提交 `4837b51`，应用本分支的落地状态补丁。NE/TVFT core 与 main `70f5fb0` 相同，控制参数及执行限幅未为通过测试而调整。

98 项单元测试通过，原有 ROS mock 12 秒完整流程回归通过；三个正式工况均运行 80 秒，完成指令 1–7、九节点遥测/rosbag 校验及六机接地后上锁。下表在最后 10 秒内、各角色全部实体中取最大值。

| 工况 | 领导者 NE 误差 (m) | UAV 编队位置误差 (m) | UAV 编队速度误差 (m/s) |
|---|---:|---:|---:|
| no_delay | 1.4741e-07 | 0.00109211 | 0.00019207 |
| fixed_50ms | 5.34709e-07 | 0.00091397 | 0.000178705 |
| normal_50ms_std15ms | 5.00971e-07 | 0.000921671 | 0.000224281 |

三组最大采样间隔均约 20 ms。全部正式 UAV 目标采用 acceleration-only、掩码 2111；时延按 ROS 仿真时间调度。保留默认 300 ms 邻居过期保护。

固定 50 ms 配置的 jitter 为零；随机工况采用均值参数 50 ms、标准差参数 15 ms、截断到 0–100 ms 的正态分布，种子 7。此处每个工况仅一次运行，误差差异不能用于证明时延提升控制性能或泛化稳定性。毫米级残差对应当前理想反馈条件，不作为实机精度指标。

RViz 软件渲染已实测，九台机器人和轨迹可见；87 个 URDF 链接到 map 的变换均可查询。原生显卡渲染遇到 nouveau 驱动错误，SSS 启动入口改用软件渲染。

原始记录、图和报告位于本地 output/sss_run_*/，按 .gitignore 不纳入提交。哈希、逐次路径和样本数见 [integration/validation_20261102.json](integration/validation_20261102.json)。

- `no_delay`：`output/sss_run_fvJUIXH4`
- `fixed_50ms`：`output/sss_run_EmZZp1IY`
- `normal_50ms_std15ms`：`output/sss_run_dsMf5eGS`

[启动与复现说明](PX4_SIM.md)。

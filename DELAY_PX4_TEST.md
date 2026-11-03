# 时延分支：swarm_sync_sim/PX4 动力学复测

本次在 `experiment/communication-delay@05f4bd3` 基础上接入此前 `PX4_sim@b916da3` 已验证的 SSS 后端。只移植平台适配，没有切换到 PX4_sim 分支或改变 main/PX4_sim 的提交。数学控制核心 `src/ne_tvft/core.py` 不变。

## 场景和保护设置

- 三个一阶领导者使用 SSS 麦轮 UGV；六个二阶跟随者使用 SSS 的 PX4 控制模块与四旋翼动力学，直接闭环订阅实际仿真状态。
- `config/experiment.yaml` 与 `config/sss_experiment.yaml` 均保留 `runtime.signal_timeout: false`，没有恢复 300 ms 或换成更大数值。
- 保留定位、FCU、监督器、记录器、采样截止时间及执行限幅。首次邻居数据、会话和时间戳合法性检查保留。
- 通信队列在 SSS 模式按 ROS 仿真时间排队，仍保存原始源时间戳和墙钟接收时刻。Python Agent、监督器、记录器状态定时器注册 SSS time client。
- 正式 UAV 控制为三轴 acceleration-only，掩码 2111；高度使用原竖直 PD。落地反馈补丁与 OFFBOARD 降落后上锁适配沿用此前验证版本。
- SSS 上游固定提交 `4837b511c3d2bcaa36448e5e5af51335004bafd8`。这是 PX4 v1.13.3 控制模块的轻量仿真，不是完整 PX4-Autopilot 固件 SITL，也不是实机。

## 复现（xgc 容器内）

先确认当前分支为 experiment/communication-delay。下列安装使用独立 `delay_px4_experiment_ws`，不会覆盖此前两个实验 overlay。

```bash
cd /home/xgc_ws/src/hete_TVFT_NE
bash scripts/setup_sss_workspace.sh /home/xgc_ws
source /home/xgc_ws/delay_px4_experiment_ws/devel/setup.bash
# 手动交互，三车六机和 RViz；按原指令 1–7 操作
roslaunch ne_tvft_experiment sss.launch \
  communication:=$(rospack find ne_tvft_experiment)/config/communication_normal250.yaml
```

批量隔离验证：无时延基线一次，250 ms 正态时延三次，标准差 50 ms、截断 0–500 ms，种子 11/29/47，每次正式运行 80 秒，初值 A。控制参数、拓扑、半径与执行限幅保持原样。

```bash
python3 -B scripts/run_no_timeout_trials.py --backend sss \
  --output /home/xgc_ws/delay_px4_experiment_results/新的日期时间_sss_normal250_no_timeout \
  --duration 80 --seeds 11 29 47
python3 scripts/summarize_no_timeout_trials.py \
  /home/xgc_ws/delay_px4_experiment_results/新的日期时间_sss_normal250_no_timeout
```

批量入口禁用 GUI，每次创建独立 localhost ROS master，并检查图中的传感器/执行接口确由 SSS nodelet_manager 提供、没有启动 mock_fleet。完成指令 1–7 后校验正式段输入类型、九个实体的 JSONL/rosbag 样本及记录完整性，再分析误差。

`--backend mock` 仍为批量脚本默认值，用于保留上一阶段实验；本阶段必须显式传入 `--backend sss` 并 source 本节 overlay。脚本只允许在时延分支运行。

数据按日期和工况命名，目录为 `baseline_no_delay`、`normal250_seed11`、`normal250_seed29`、`normal250_seed47`；每个目录保留配置、ROS 日志、smoke-report.json、output/<会话>/ 原始记录和 picture/<同一会话>/ 分析图。SSS 数据根目录独立于之前的 `delay_experiment_results` 和 `output/sss_run_*`。

103 项单元测试已通过。最终运行状态与结果另行记录，任何失败尝试保留并单独报告，不用调大保护阈值来掩盖失败。

正式结果与所有失败尝试见 [DELAY_PX4_RESULTS.md](DELAY_PX4_RESULTS.md)。

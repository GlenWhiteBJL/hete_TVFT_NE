> 2026-11-03 更新：默认取消邻居通信过期判定（signal_timeout=false），新增独立无保护随机时延试验；下文历史工况与报告保留。

# 通信时延实验分支

分支 `experiment/communication-delay` 从 main 的 `70f5fb0` 延伸，原工程另存为本地 `experiment-delay`。时延仅施加于拓扑允许的智能体间信号：领导者 y/z 与组内 w。自身定位、MAVROS 执行、监督器、指令和记录器通信保持原路径。

ROS 节点在接收后排队，到期才释放数据，不阻塞回调或控制循环。源时间戳和实际接收时刻保持原值，不把陈旧包伪装成新数据；乱序旧包不覆盖新状态。每次 SCHEDULED 清空旧会话队列。队列有上限，溢出会明确报故障。

## ROS mock 用法

正态随机时延工况使用 `communication_normal.yaml`：参数均值 50 ms、标准差 15 ms、在 0–100 ms 之间拒绝采样得到截断正态分布（不是把越界值钳到端点）。固定随机种子保证每条有向边的抽样序列可复现，操作系统调度仍可能使完整轨迹略有不同。

```bash
sudo env NE_TVFT_DELAY_PROFILE=communication_normal.yaml bash cloud/run_docker.sh 80
```

可设置 `distribution: normal`、`delay_s`（分布均值参数）、`std_s`、`min_delay_s`、`max_delay_s`。原 uniform 配置继续表示均匀抖动，fixed 表示恒定值。截断后的实际矩应以日志为准。通信诊断保存每条有向边全部生成注入的计数、一二阶矩、极值和20个区间的直方图，包含后来因乱序而丢弃的包；不能只使用每个控制 tick 最后一个交付包推断抽样分布。

默认 profile 关闭注入。手动启动 50 ms 工况：

```bash
roslaunch ne_tvft_experiment experiment.launch mock:=true communication:=$(rospack find ne_tvft_experiment)/config/communication_50ms.yaml
```

沿用 `/command` 的 1–7 流程。容器一键验证：

```bash
sudo env NE_TVFT_DELAY_PROFILE=communication_50ms.yaml bash cloud/run_docker.sh 80
```

将文件名换为 `communication_none.yaml`、`communication_20ms.yaml`、`communication_100ms.yaml` 即可比较。每次独立创建记录目录；保存配置快照、通信队列诊断、邻居源数据年龄、乱序计数及原始输入。JSONL 和 CSV 的 communication 字段均保存诊断。

自定义 YAML 格式（单位均为秒）：

```yaml
communication:
  enabled: true
  delay_s: 0.05
  jitter_s: 0.01
  seed: 7
  max_pending: 4096
  links:
    ugv1->ugv2: 0.08
    ugv1->uav1: 0.10
```

抖动为每条有向边独立、可复现的均匀分布，实际注入为 max(0, 基础时延 + 抖动)。ROS 到期包在下次控制采样时使用，因此实际数据年龄包含注入时延、原有传输与采样等待。未实现丢包、带宽限制或时钟漂移。

本分支现在按用户要求使用 `runtime.signal_timeout: false`，取消邻居通信数据年龄的过期判定；定位、监督器、记录器、控制截止时间及执行限幅不变。ROS 参数服务器不支持 null，因此使用明确的 false 开关。正数仍可显式启用历史通信保护以复现实验。启动前仍须收到每个必要邻居的有效首包，旧会话、非法时间戳和乱序旧包仍被拒绝。配置、复现与新结果见 [NO_TIMEOUT_RANDOM_TEST.md](NO_TIMEOUT_RANDOM_TEST.md)。

## 离线效果对照

```bash
python scripts/delay_sweep.py --duration 80 --dt 0.005 --delays-ms 0 20 50 100
```

输出到 `output/<时间>_delay_sweep/`：各工况原始 telemetry、配置、数值摘要，以及 comparison.json/CSV/PNG。可用现有 analyze_experiment.py 对每个 case 单独生成完整图像。

离线模型采用常值初始信号作为 t<0 的历史，收到首个延迟包前保持该初值，所有邻居量使用相同的采样前快照。它研究理想采样动力学的时延敏感性；不运行 ROS 看门狗、握手或飞控，不生成 rosbag。时延按采样步长量化，应结合减半步长检查评估数值误差。ROS profile 的全流程结果必须由真实 ROS 运行另行确认。

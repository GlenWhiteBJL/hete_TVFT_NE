# 通信时延实验分支

分支 `experiment/communication-delay` 从 main 的 `70f5fb0` 延伸，原工程另存为本地 `experiment-delay`。时延仅施加于拓扑允许的智能体间信号：领导者 y/z 与组内 w。自身定位、MAVROS 执行、监督器、指令和记录器通信保持原路径。

ROS 节点在接收后排队，到期才释放数据，不阻塞回调或控制循环。源时间戳和实际接收时刻保持原值，不把陈旧包伪装成新数据；乱序旧包不覆盖新状态。每次 SCHEDULED 清空旧会话队列。队列有上限，溢出会明确报故障。

## ROS mock 用法

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

原始 `signal_timeout=0.3 s`、状态机和所有执行限幅保持不变。接近或超过此阈值的时延工况可能无法通过启动屏障或触发故障，这是保护机制结果，不能当作算法已收敛，也不会自动扩大超时来让测试通过。先在 mock 测试后再评估实物，不据此声称原无时延定理涵盖有时延系统。

## 离线效果对照

```bash
python scripts/delay_sweep.py --duration 80 --dt 0.005 --delays-ms 0 20 50 100
```

输出到 `output/<时间>_delay_sweep/`：各工况原始 telemetry、配置、数值摘要，以及 comparison.json/CSV/PNG。可用现有 analyze_experiment.py 对每个 case 单独生成完整图像。

离线模型采用常值初始信号作为 t<0 的历史，收到首个延迟包前保持该初值，所有邻居量使用相同的采样前快照。它研究理想采样动力学的时延敏感性；不运行 ROS 看门狗、握手或飞控，不生成 rosbag。时延按采样步长量化，应结合减半步长检查评估数值误差。ROS profile 的全流程结果必须由真实 ROS 运行另行确认。

# 随机时延复测：取消邻居通信过期判定

本次改动仅在 `experiment/communication-delay` 分支。`main@70f5fb0` 和 `PX4_sim@b916da3` 不变，未合并 PX4/SSS 代码。测试后保持当前检出时延分支。

`config/experiment.yaml` 的 `runtime.signal_timeout` 改为 **false**。这取消邻居信号的源时间年龄与接收年龄上限，不是把 300 ms 改成 600 ms 或其他更大阈值。ROS1 参数服务器不支持 null，因此采用可序列化的 false。

收到首包后保持最后收到的有效邻居数据，不再因邻居数据长时间不更新而退出。启动时仍要求所有必要邻居有有效首包，会话不匹配、非法/未来时间戳和乱序旧包仍拒绝。定位、FCU、监督器、记录器、采样截止时间及执行限幅保持原值。正数 signal_timeout 仍可用于重现历史保护组；默认测试使用 false。

## 工况

保持原初值 A、控制参数和拓扑：无时延基线一次；250 ms 正态随机时延，标准差 50 ms、截断 0–500 ms，种子 11/29/47 各一次。每次正式运行 80 秒，完整执行原 ROS mock 指令 1–7、记录并分析。不是 PX4 或实物试验。

独立 overlay 为宿主机 `~/xgc_ws/delay_experiment_ws`，容器内 `/home/xgc_ws/delay_experiment_ws`。以下在 xgc 中运行：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p /home/xgc_ws/delay_experiment_ws/src
ln -sfn /home/xgc_ws/src/hete_TVFT_NE /home/xgc_ws/delay_experiment_ws/src/ne_tvft_experiment
cd /home/xgc_ws/delay_experiment_ws
catkin_make
source devel/setup.bash
cd /home/xgc_ws/src/hete_TVFT_NE
python3 -B scripts/run_no_timeout_trials.py \
  --output /home/xgc_ws/delay_experiment_results/新的日期时间_normal250_no_timeout \
  --duration 80 --seeds 11 29 47
python3 scripts/summarize_no_timeout_trials.py /home/xgc_ws/delay_experiment_results/新的日期时间_normal250_no_timeout
```

不要 source `sss_experiment_ws` 来运行此分支。测试脚本检查当前 Git 分支，使用独立 localhost master，保留每个尝试和实际配置。结果写入 `xgc_ws/delay_experiment_results`，不混入实验仓库原有的 `output/sss_run_*`。

## 目录规则

```
日期时间_normal250_no_timeout/
  baseline_no_delay/
  normal250_seed11/
  normal250_seed29/
  normal250_seed47/
  batch-status.json
  unit-tests.log
  summary.json
  REPORT.md
  comparison.png
```

每个工况内包含原始记录 output/<北京时间会话编号>/、picture/<同一编号>/、配置、smoke-report.json 及日志。汇总报告统计末 10 秒最大误差、实际生成的时延分布和正式段邻居数据年龄超过 300 ms 的次数。源数据年龄与注入时延分布分别统计。

早期启动入口的失败尝试同样保留：`20261103_015932_normal250_no_timeout` 为 ROS Python 路径问题，`20261103_015951_normal250_no_timeout` 为 ROS1 不接受 null 参数，均不属于控制律性能结果。正式参数为 false，最终结果另列。

最终四次正式试验全部通过。指标见 [NO_TIMEOUT_RANDOM_RESULTS.md](NO_TIMEOUT_RANDOM_RESULTS.md)；原始记录和图位于 `20261103_020135_normal250_no_timeout`。

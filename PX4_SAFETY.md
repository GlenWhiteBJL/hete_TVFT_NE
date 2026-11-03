# PX4_sim 组内避碰

本功能只在 `PX4_sim` 分支接入。每组三个实体中，两架 UAV 主动避让，同组 UGV 作为有高度的移动障碍；不增加组间、墙面或其他障碍物的避让。

## 默认参数与启动

```bash
# xgc 容器内
cd /home/xgc_ws/src/hete_TVFT_NE
source /home/xgc_ws/sss_experiment_ws/devel/setup.bash
roslaunch ne_tvft_experiment sss.launch
```

`sss.launch` 默认 `enable_safety:=true`。关闭对照可用 `enable_safety:=false`；开关会覆盖 YAML 的 `safety.enabled`。`experiment.launch` 的此参数默认 false，原 mock 配置不自动启用避碰。

| 参数 | 当前 SSS 默认值 |
|---|---|
| `formation.radii` | `[0.50, 0.50, 0.50] m`，launch 出生点同步调整 |
| `formation.angular_rates` | `[0.10, 0.10, 0.10] rad/s` |
| `groups[].altitude` | `1.2 m` |
| `limits.ugv_speed` | `0.15 m/s` |
| `limits.uav_speed` | `0.30 m/s`，实测超速故障阈值 |
| `limits.uav_acceleration` | `0.80 m/s²` |
| `safety.uav_min_distance` | `0.45 m`，两机水平中心距离 |
| `safety.state_max_age` | `0.10 s`，独立于算法的 300 ms 邻居信号保护 |
| `safety.barrier_k1 / barrier_k2` | `2.0 / 2.0` |
| `safety.acceleration_error_bound` | `0.15 m/s²`，建模假设，需实测确认 |
| `safety.ugv_acceleration_bound` | `0.50 m/s²`，建模假设，需实测确认 |
| `safety.speed_prediction_horizon` | `0.20 s` |
| `safety.landing_clearance` | `0.05 m`，加到车体椭球水平半轴上生成下降通道 |
| `safety.position_kp / velocity_kp` | `1.0 / 2.0`，参考转换的名义加速度增益；起降位置接口仍保留 |

NE 代价函数、位置参数、`core.py` 控制律和算法通信拓扑不变。该分支仍保留 `runtime.signal_timeout=0.3`，不混入去除该保护的时延分支配置。

## 机体包络

按 20×20 cm 飞机平面包络，取 `uav_body_radius=0.15 m`；车体 30×30×20 cm，取水平外接圆半径约 `0.21213 m`、半高度 `0.10 m`。飞机半高度暂取 `0.08 m`，**这是待实测确认的配置假设**，并非用户提供的测量值。附加 `body_margin=0.05 m`。

UGV 障碍采用中心位于 `UGV_pose + [0,0,0.10]` 的膨胀椭球，水平半轴 `0.48 m`、竖直半轴 `0.55 m`。初始化检查椭球能否包住车体加飞机包络及裕量。真实动捕刚体原点如果不对应当前仿真的车辆底面中心或飞机中心，必须先标定变换，不能直接沿用该偏移。

这允许正常高度下绕车飞行，不因 XY 距离小而持续排斥 UGV；低空则约束机体与车体的接近。

## 输入链路与控制含义

每个 agent 仍是其执行话题的唯一发布者，没有额外节点竞争 MAVROS 输出。原 NE/TVFT 控制器产生名义输入，UAV agent 在发送前调用安全过滤。

安全层另外直接订阅同组另一架 UAV 与 UGV 的源时间戳位姿、速度；这些反馈不走算法时延注入队列。统一到世界 ENU 后，对不超过 `state_max_age` 的短龄位置使用恒速外推。ROS 源时间戳与接收墙钟新鲜度同时检查；要求源时间戳速度 `TwistStamped`，不接受无时间戳速度充当实机安全反馈。

实机规划采用 120 Hz 动捕输入，当前控制及安全过滤周期仍为 50 Hz；8.3 ms 采样周期不等于端到端延迟。若速度由位姿差分得到，需把滤波延迟计入反馈年龄和误差标定。当前 `topics.yaml` 仍是 SSS/MAVROS 接口示例，尚未替换成实际动捕刚体名称。

两机水平距离函数为：

$$h=\|p_1-p_2\|^2-d_{\min}^2.$$

采用二阶约束 `ḧ + (k1+k2)ḣ + k1·k2·h ≥ 鲁棒裕量`。两架 UAV 各承担相同的半份约束，使用同一套参数；它们不是在两个节点里求解一个联合 QP。该互惠形式依赖双方及时执行、状态坐标和参数一致，异步采样与飞控误差须单独验证。

每架机只求解二维最小输入修正，约束包含机间分离、UGV 三维椭球、水平加速度及短时预测速度。二维半平面投影枚举边界与顶点求精确欧氏最近点，无需外部 QP 求解器。范数限制用内接八边形保守表达，求解后不再裁剪破坏安全约束。

名义输入满足分离与执行限制时原样保留。安全层介入会改变实际闭环，原算法的收敛证明不能直接视为加入过滤器后的证明。

## 各阶段的实际保护范围

| 阶段 | 实现 |
|---|---|
| IDLE、准备、解锁 | 检查组内反馈和初始几何；保留原地面位置模式，不自动推开已重叠实体 |
| 起飞、初始点转移 | 保留原 PX4 位置接口；检查参考线段与同组实体包络是否相交，不安全时拒绝并报告故障 |
| RUNNING | 实时加速度安全过滤，正常高度不调整竖直参考来躲避另一架机 |
| 停止/故障制动 | 高度至少 0.60 m 时，零速度参考可在需要时转换为安全加速度；低空保留原速度接口，稳定后保留位置保持；反馈失效或约束不可行时报告故障并回退制动 |
| 降落 | 先在原高度向车外侧移动，确保下降通道；SSS 中满足水平就绪条件后再下降 |

降落通道的水平参考至少距车中心 `ugv_horizontal_radius + landing_clearance = 0.53 m`；当前位置需接近目标 2 cm 内且半径不小于 `0.505 m` 才开始下降。硬件路径在请求 AUTO.LAND 前也先做这一横移准备；进入 AUTO.LAND 后由飞控执行下降，不宣称外部加速度过滤仍拥有控制权。硬件路径尚未实飞验证。

位置模式检查采用同步直线转移的几何模型，**不是完整的动态路径规划器**。遇到相互交换初始点等不安全路径会拒绝，而不是自动生成绕行。现有全局监督器根据 agent 故障使整个实验退出正式控制，没有新增分组状态机。

约束不可行或反馈过期时的零速度回退不等于数学保证仍然成立；初始重叠、过大的接近速度、执行器失效等情况不能靠此机制保证恢复安全。

## 记录与第 14 张图

遥测 JSONL 和 rosbag 中新增 `safety` 字段，包含：

- 水平/空间机间距、机车空间距离、两类屏障值；
- 是否启用、是否健康、是否修正、修正大小；
- 名义/实际水平加速度、最小优化约束余量、求解耗时；
- 人工测试输入是否生效以及故障原因。

分析自动新增 **`14_intra_group_distances.png`**：三个子图对应三组，每组绘制两机间及两条机车空间距离，另绘两机水平距离、0.45 m 阈值和组内安全修正区间。该阈值仅用于两机水平分离，不能套到机车空间距离上。

新图的红色底纹表示组内任意一架 UAV 在该匹配时刻发生安全修正。它覆盖已记录的会话阶段（含停止尾段），不代表包含记录开启前的完整起飞或记录关闭后的完整降落。超时匹配显示为缺失，不填零。`metrics.json` 新增 `intra_group_distances` 与 `safety` 指标；跟踪指标仍只统计 `RUNNING && stamp>=t0`。

`safety` 是 `/ne_tvft/telemetry` 的 String 消息内部 JSON 字段，不是新 ROS 话题。读取时先按 `id` 选择 UAV，再查看 `healthy / active / reason`；UGV 的 `safety.enabled=false` 表示其不执行主动避让，不能据此判断整个组的安全层关闭。出现 `safety reference` 说明位置路径检查拒绝了参考；出现 `safety filter` 说明加速度约束不可行或输入异常。`limited` 同时标记名义限幅或安全修正，具体原因结合 `safety.active` 和原始/最终输入判断。

手动 launch 结束后运行分析，例如：

```bash
python3 scripts/analyze_experiment.py output/<session_id> --picture-root picture
```

一键测试在通过后自动分析，图像位于 `output/sss_run_XXXXXXXX/picture/<session_id>/`。本地开关对比汇总图为 `output/safety_validation_20261104/safety_on_off_comparison.png`；原始记录和 PNG 按 `.gitignore` 不随源码上传。上传的数值、路径及哈希报告见 [PX4_SAFETY_RESULTS.md](PX4_SAFETY_RESULTS.md)。

## 复现测试

```bash
# 80 秒正常闭环
bash scripts/run_sss_test.sh 80

# 50±15 ms 截断正态算法通信时延；安全反馈不注入该时延
bash scripts/run_sss_test.sh 80 communication_normal.yaml

# 人工相向接近，只能用于 SSS
NE_TVFT_SSS_CONFIG=/home/xgc_ws/src/hete_TVFT_NE/config/sss_safety_challenge.yaml \
  bash scripts/run_sss_test.sh 80

# 同一人工工况关闭安全层的对照
NE_TVFT_SAFETY=false \
NE_TVFT_SSS_CONFIG=/home/xgc_ws/src/hete_TVFT_NE/config/sss_safety_challenge.yaml \
  bash scripts/run_sss_test.sh 30
```

挑战配置在正式段第 8–12 秒临时覆盖名义水平加速度，使两架机沿固定相反方向以 0.20 m/s 的目标速度相向接近，随后恢复原控制器。为观察完整响应，该测试的速度故障阈值为 0.60 m/s。**它不是正常实机任务配置**：ROS 层在非 SSS backend 下拒绝启用人工挑战。开/关安全层使用相同挑战输入定义。

`offline_demo.py` 也使用同一纯数学过滤器，可做理想模型对照；它不替代 SSS/PX4 动力学或实机测试。SSS 验证除原有数据和阶段检查外，还核查正式段安全诊断以及实测两机水平间距。

验证结果与实际文件路径见 [PX4_SAFETY_RESULTS.md](PX4_SAFETY_RESULTS.md)。实机开关保持关闭，尚需核对真实尺寸、位姿原点、速度估计、反馈/执行延迟和误差界，再开展实机试验。

# 3 UGV + 6 UAV：环境监测 NE 与圆周 TVFT 实验

当前分支加入可配置通信时延及离线效果对照，用法和验证范围见 [COMMUNICATION_DELAY.md](COMMUNICATION_DELAY.md)。下述既有服务器验证属于 main 的无时延版本，不能作为本分支时延工况已通过 ROS 验证的证据。

本工程依据草稿第二、三章和“拟定技术方案”聊天中的实体配置实现。三台麦克纳姆轮无人车作为一阶领导者，六架无人机作为二阶跟随者；三个小队通过环境监测位置博弈选择工作位置，两架无人机相对本组车辆保持同半径、同角速度、相差 180 度的期望圆周运动。

本次交付包含 ROS Noetic/catkin 工程、真实算法的离线动力学示例、ROS 接口模拟器、状态机、异步数据记录与独立分析程序。工程已在腾讯云 Ubuntu 24.04 宿主的 Ubuntu 20.04 / ROS Noetic Docker 环境中完成 catkin 编译并运行 ROS mock。云服务器用法见 [cloud/TENCENT_SERVER.md](cloud/TENCENT_SERVER.md)。PX4 SITL 和实物飞行尚未执行。

## 1. 配置与实体

| 组 | 一阶领导者 | 二阶跟随者 |
|---|---|---|
| 1 | ugv1 | uav1、uav2 |
| 2 | ugv2 | uav3、uav4 |
| 3 | ugv3 | uav5、uav6 |

`config/experiment.yaml`：任务、拓扑、增益、半径、角速度、初始点、高度、限幅和时序。`config/topics.yaml`：每台机器人的话题、消息类型、坐标变换、偏航及服务名。

领导者采用链式通信 `ugv1 <-> ugv2 <-> ugv3`，因此首尾两车需要间接估计彼此的虚拟参考。每组无人机相互通信，只有第一个跟随者直接接收本组领导者的 w 信号。各节点只订阅拓扑允许的控制信号。

圆周半径默认 0.35 m、角速度 0.10 rad/s、高度 1.2 m，均为仿真初值，需按实物尺寸和定位精度调整。半径与角速度从统一 t0 起保持恒定；从悬停进入运动时的速度暂态被记录，不额外加入参考角速度爬升。两机和车在水平投影上共线，车辆为圆心；本实验并不使用三角形编队。

默认配置的坐标原点、话题及硬件核验标志只是模板；`mock=true` 将外部话题和服务映射到 `/mock/...`，不会向实际的 MAVROS 或底盘执行话题发布命令。

## 2. 先运行离线示例

不需要 ROS：

```bash
python -m pip install -r requirements.txt
python scripts/preflight.py
python scripts/offline_demo.py --duration 80 --refine
python scripts/analyze_experiment.py output/<实验目录名>
```

离线示例直接调用与 ROS 节点相同的 LeaderController/FollowerController。每个步长统一截取邻居信号，再更新各控制器与物理状态，避免顺序更新泄漏本步后续信息。默认是无输入限幅的采样理想积分器；`--apply-limits` 启用与配置相同的平面输入限幅，作为额外的数值检查。两种模式均不是 PX4 仿真。

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
```

Windows PowerShell 的测试命令为：

```powershell
$env:PYTHONPATH = (Join-Path (Get-Location) 'src')
python -B -m unittest discover -s tests -v
```

示例数据、数值报告在 `output/<实验时间>/`；对应图像、分析指标在 `picture/<同一实验时间>/`。离线示例会明确标记 `offline`，不会生成伪装为实机记录的 rosbag。

## 3. Ubuntu 20.04 / ROS Noetic 安装

如在 Ubuntu 云服务器执行，先看 [cloud/TENCENT_SERVER.md](cloud/TENCENT_SERVER.md)。其中使用固定 Noetic 环境的 Dockerfile，自动完成 ROS mock 全流程、数据核验及绘图。Codex 云环境的独立配置方式保留在 [cloud/README.md](cloud/README.md)。

将整个目录作为一个 catkin package 放入工作空间 `src`。例如：

```bash
mkdir -p ~/catkin_ws/src
cp -r experiment-3x3 ~/catkin_ws/src/ne_tvft_experiment
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

消息依赖为标准 ROS、geometry_msgs、sensor_msgs 和 mavros_msgs。内部实验通信使用版本为 1 的 JSON 对象封装在 `std_msgs/String` 中，不需要编译自定义消息。`/command` 使用 `std_msgs/Int32`。

先运行 ROS 接口模拟器：

```bash
roslaunch ne_tvft_experiment experiment.launch mock:=true
rosrun ne_tvft_experiment preflight.py --ros
```

模拟器提供 `/mock/ugv...` 和 `/mock/uav...` 状态、MAVROS 服务与执行接口。它验证 ROS 通信、状态转换、记录和接口通路，物理模型是理想积分器，不能替代飞控 SITL。

## 4. 指令与实际反馈

逐步发布命令，并确认 `/ne_tvft/session` 已达到相应完成状态后再发下一条：

```bash
rostopic pub -1 /command std_msgs/Int32 "data: 1"
rostopic echo /ne_tvft/session
```

| 指令 | 过程 | 完成状态 |
|---|---|---|
| 1 | 检查状态流、FCU 连接、目标流条件 | READY |
| 2 | 预先持续发送目标，再请求 OFFBOARD 和解锁 | ARMED，依据实际 FCU 状态 |
| 3 | 各机保持本地 XY，逐步上升 | AIRBORNE |
| 4 | 车到初始点，机到车初始点+h(0) | POSITIONED，满足位置/速度和驻留时间 |
| 5 | 建立记录、控制器准备、确认启动 | RUNNING，统一未来 t0 到达后执行 |
| 6 | 停止论文控制，冻结自适应，车停、机减速后保持 | STOPPED |
| 7 | 车保持停止，各机 AUTO.LAND | LANDED，实际落地且解除解锁 |

重复指令幂等，不会重复重置参数或创建记录目录；非法跳转被拒绝。启动 5 要经过四个阶段：

1. RECORDING：创建独占的新目录，文件打开，rosbag 进程运行且产生 active 文件后，记录器才 ACK。
2. SCHEDULED：分配未来 t0；所有节点按本次 session 重置、发布初始信号、填充邻居缓存。
3. 全部节点 ACK 后，提前发布 RUNNING 作为提交信号。
4. 各节点在 t0 后第一个控制 tick 执行，记录实际启动偏差。没有及时收到提交、节点未就绪或记录失败时，禁止部分启动。

多主机需要同步 ROS 时间/系统时钟；相同的 t0 字段不能证明物理同步，必须检查 `start_offset_s`。超时使用来源时间戳和本机单调时钟双重检查。记录线程不阻塞目标发布；服务请求也在独立线程执行。

运行期间丢失状态、邻居、监督器或记录器心跳，控制将退出 RUNNING。无人机通过零速度目标减速，再捕获实际位置保持，不能用“零加速度”代替停止。故障后不自动恢复论文控制；显式停止/降落及重新准备流程用于恢复。

## 5. 实机接口配置

部署前先修改两个 YAML，并运行 `preflight.py --hardware`。需在配置中确认 `hardware.enabled`、`frames_verified`、`acceleration_interface_verified`，填写实际飞控版本。通过 launch 的 `mock:=false allow_hardware:=true` 才能启用实际输出。这些开关仅防止模板误连，不表示软件已替代飞控核验。

```bash
roslaunch ne_tvft_experiment experiment.launch mock:=false allow_hardware:=true
```

启动本工程不会自动连接飞控、配置外部定位或修改 PX4 参数。MAVROS、底盘驱动和定位系统应独立启动，且执行话题不能同时由其他控制器争用。

### 状态话题

默认 UAV 候选话题是 `/uav{i}/mavros/local_position/pose`、`/uav{i}/mavros/local_position/velocity_local`、`/uav{i}/mavros/imu/data`。你原来的重映射名称可以直接填入。UGV 默认 `/ugv{i}/pose`、`/ugv{i}/twist`、`/ugv{i}/imu`，位姿可替换为 `/vrpn_client_node/<刚体名>/pose`。

当前支持 PoseStamped 位姿，以及 TwistStamped 或 Twist 速度。无时间戳的 Twist 会明确使用接收时 ROS 时间并记录来源，不能识别上游重复的旧测量。若实际驱动发布 Odometry，需要独立转换为上述消息再输入。IMU 是可选记录流，不参与本次论文控制。

### 坐标

所有控制和记录的物理状态采用共同 ENU/world 坐标。每个机器人的 `transform` 显式定义位姿来源 ENU 到 world 的固定偏航和位移；可独立配置速度来源坐标变换。`setpoint_transform` 定义 MAVROS 本地 ENU 到 world 的变换，发送目标时取逆变换。VRPN 的刚体原点应是对应控制点。

UGV `cmd_frame: body` 时，将世界平面速度旋转到车体系；全向车的 linear.y 必须由底盘驱动实际支持。UAV 默认使用局部世界系速度；body 速度需要完整姿态映射，不能直接用平面偏航近似替代。

MAVROS ROS 接口接收 ENU 数值，并由 MAVROS 内部转换为 MAVLink NED；本工程不会再次手动交换 XY/翻转 Z。可参阅 [MAVROS raw setpoint 源码](https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/setpoint_raw.cpp)。

### 无人机输出

运行论文控制时，只启用 `PositionTarget.acceleration_or_force` 的三个加速度分量，`type_mask=2111`，FORCE 位关闭，偏航有效，坐标字段为 FRAME_LOCAL_NED（ROS 数据仍为 ENU）。水平为论文加速度，竖直为独立限幅 PD 高度控制产生的运动学加速度。不要额外加 g。

位置保持/起飞/到初始点使用 position-only；制动使用 velocity-only。输入限幅保留 raw/applied/limited 和实际 setpoint。超速触发退出控制，不以悄悄截断测得状态的方式冒充满足理论。限幅、采样和飞控内环使实物成为受限近似系统，必须由 SITL/单机测试验证。

PX4 要求在切换前和运行时持续发送目标流；工程默认 50 Hz、预热 2 秒，服务调用不阻塞该流。[PX4 Offboard 文档](https://docs.px4.io/main/en/flight_modes/offboard)

## 6. 数学范围

控制律沿用一阶领导者分支与二阶跟随者分支；本实物配置验证了组间异构，并没有验证二阶领导者或一阶跟随者分支。原有一般异构仿真可用于补充验证这些分支。

三组采用 simulation-2 的 Gaussian 监测成本：

```text
f_i = a_i/2 ||x_i-c_i||^2 + eta_i/2 ||x_i-s_i||^2
      + sum_j gamma_ij exp(-||x_i-x_j||^2/(2 sigma^2))
```

程序计算全局强单调下界，默认约 0.49713。NE 仅在离线分析/预检时独立求解，在线控制节点初始化不求 NE、不读取全局真实位置替代估计。应用覆盖仍为合成成本模型，不宣称真实监测精度或社会最优。

当前圆周参考没有内置碰撞避免或场地边界约束。机器人尺寸、起降区域和飞控 failsafe 参数需要结合平台确定；限幅与故障退出会记录下来，而不是被当作原定理的无条件保证。

## 7. 记录与分析

每次指令 5 使用新的上海时区实验时间名。保存 `metadata.json`、实际配置、telemetry.jsonl、telemetry.csv、events.jsonl、raw.bag 与 rosbag.log。写队列满、序列化失败或磁盘故障会明确使记录器 unhealthy，并触发退出；缺失加速度为 null 且 validity=false。

测量加速度区分：速度差分的世界系加速度估计、IMU 原始机体系数据、理论加速度和下发加速度。IMU 的具体重力处理取决于实际驱动语义，本程序不把原始读数直接当作世界系运动学加速度。

收到停止后，记录制动与保持尾段；分析只对 `phase=RUNNING` 且实际时间戳 `stamp >= t0` 的正式控制段统计，横轴使用 `stamp-t0`。提前提交 RUNNING 时、尚在等待未来 t0 的样本不进入正式统计。程序保存了停止模式下实际 setpoint，`input_applied` 的加速度分量在非加速度模式中为零不表示无人机真实加速度为零。

```bash
python scripts/analyze_experiment.py output/<实验时间> --picture-root picture
```

分析以有限时间差匹配本组领导者的状态，超出间隔标为缺失，不填零。图包含世界轨迹、相对车辆轨迹、半径/相位/绕行角速度、NE/估计误差、编队误差、自适应增益、输入和数据质量。绕行角速度由相对位置和相对速度计算，不使用机体 IMU 角速度代替。

物理可用性评价应按定位和飞控精度设置误差阈值；不要把理想仿真微米级残差当作实机精度承诺。

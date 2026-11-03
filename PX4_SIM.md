# PX4_sim：swarm_sync_sim 三车六机 ROS 闭环

本分支从 `main@70f5fb0` 接入 SSS，并复用通信时延分支的接收队列。`src/ne_tvft/core.py` 的 NE/TVFT 控制律保持不变。实体为三个麦轮 UGV 一阶领导者、六个 UAV 二阶跟随者。

## xgc 内安装与启动

宿主机的 `~/xgc_ws` 映射为容器的 `/home/xgc_ws`。平台与实验各用一个 overlay，均放在 `xgc_ws/src` 之外，避免 `group_matlab2ROS` 的同名包冲突。不要在原整个 xgc_ws 上重新全量 catkin_make 来安装本接入。

```bash
# 宿主机进入现有容器
docker exec -it xgc bash

# 以下在容器内执行
cd /home/xgc_ws/src/hete_TVFT_NE
bash scripts/setup_sss_workspace.sh /home/xgc_ws
source /home/xgc_ws/sss_experiment_ws/devel/setup.bash
roslaunch ne_tvft_experiment sss.launch
```

`setup_sss_workspace.sh` 固定上游版本 `4837b511c3d2bcaa36448e5e5af51335004bafd8`，在 `/home/xgc_ws/swarm_sync_sim` 应用 `integration/swarm_sync_sim.patch`，编译三个仿真包，并将本实验链接到 `/home/xgc_ws/sss_experiment_ws` 后编译。脚本可重复执行，不写 `.bashrc`，不修改原参考包。

`sss.launch` 同时启动一个时钟、三辆 `ugv_type=0` 麦轮车、六架 `local_pos_source=0` 无人机、九个实验节点、监督器、记录器及一个包含全部九台机器人和轨迹的 RViz。`open_rviz:=false` 适合无界面测试。GUI 时钟可用 `open_clock_ui:=true` 打开。默认生成位置与原配置对应；后续仍通过指令 4 到配置的正式初始位置。

在另一个容器终端 source 同一 overlay，再按原有指令 1–7 操作，每一步等待 `/ne_tvft/session` 的目标状态：

```bash
rostopic pub -1 /command std_msgs/Int32 'data: 1'
rostopic echo /ne_tvft/session
# 2 解锁，3 起飞，4 初始位置，5 正式控制，6 制动并保持，7 降落
```

本入口使用 `backend=sss, mock=false, allow_hardware=false`；原来的硬件确认字段仍为 false。它直接连接 SSS 话题，不启动 `mock_fleet`。不要把此启动文件用于实机 ROS master。

## 一键隔离验证

```bash
cd /home/xgc_ws/src/hete_TVFT_NE
bash scripts/run_sss_test.sh 80
bash scripts/run_sss_test.sh 80 communication_50ms.yaml
bash scripts/run_sss_test.sh 80 communication_normal.yaml
```

每次使用独立的本地 ROS master 和新目录 `output/sss_run_*/`，执行 1–7，校验九个节点正式段、启动偏差、记录完整性和 rosbag，再生成独立分析图。包括 `smoke-report.json`、`roslaunch.log`、`output/<session>/raw.bag`、原始遥测和 `picture/<session>/metrics.json`。测试失败返回非零，保留原始日志，并尝试停止、降落后清理进程。脚本不改正在使用的其他 master。

## 接口与仿真专用适配

- UGV 输入为 `/ugvN/cmd_vel` 的车体系 `Twist`；反馈为 `map` 下的 `PoseStamped` 与 `TwistStamped`，速度数值为世界系。
- UAV 继续使用 MAVROS 局部位姿、世界系速度和 `PositionTarget`。正式控制仅下发三轴加速度，掩码 2111，保持原水平控制律及竖直 PD，不额外加重力。
- 上游 `velocity_local` 的消息 frame_id 标为 `base_link`，实际数值由源码转换为 ENU；本配置按接口数值语义处理，没有再次旋转到机体系。
- 固定版本上游缺少 `extended_state` 消息。补丁从模拟器自身 `vehicle_land_detected` 发布真正的接地/在空反馈，未使用恒定的伪落地状态。
- 上游没有完整 Navigator 自动降落任务。SSS 后端的指令 7 保持 OFFBOARD，逐步降低位置目标；只有接地反馈有效、位置在地面附近且速度足够小时才请求解除解锁，并等待实际 FCU 回报。硬件后端仍走 AUTO.LAND。
- Agent、监督器和记录器状态定时器各注册 SSS time client；回调完成才请求下一个仿真时间，防止时钟跨过 Python 控制周期。
- SSS 通信注入默认使用 ROS 时间，原源时间戳和墙钟接收时刻均保留。诊断写入 JSONL/CSV；`communication.clock=wall` 可显式选择墙钟研究。墙钟新鲜度保护保留，运行中暂停后可能故障退出，不保证任意暂停恢复。

默认首先验证 `max_speed_ratio=1`。更高倍率需要重新确认墙钟看门狗、日志和服务响应，不把倍率提高自动视作等价实验。250 ms 随机延迟仍受默认 300 ms 邻居超时保护；超时或启动屏障失败不等于算法不稳定，也不计为收敛。不自动放宽阈值。

## 已完成验证

本机正式测试结果见 [PX4_SIM_RESULTS.md](PX4_SIM_RESULTS.md)。可在一键测试前设置 `NE_TVFT_OPEN_RVIZ=true` 同时查看场景。RViz 默认使用软件渲染以兼容当前容器。

## 验证范围

平台使用改写自 PX4 v1.13.3 的控制模块、真值反馈和四旋翼动力学；不是完整 PX4-Autopilot 固件 SITL。图形显示成功、状态机完整、通信健康和控制误差是分别检查的项目。只有完成闭环并检查原始记录后，才能报告该工况的控制性能。

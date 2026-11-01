# Ubuntu 云服务器运行

本工程可以在 Ubuntu 24.04 云主机上通过 Docker 运行 Ubuntu 20.04 / ROS Noetic。代码不依赖 Codex 云环境，也不需要 GPU。使用具备 sudo 权限的普通 Linux 用户即可，不需要开放 ROS 的公网端口。

## 环境安装

本次服务器使用 Ubuntu 软件源中的 Docker 和 Buildx。没有现有 Docker 安装时可以执行：

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends docker.io docker-buildx
sudo systemctl start docker
sudo docker info
sudo docker buildx version
```

若已经安装 Docker，先检查版本和已有容器，不要用上述操作替换现有业务环境。公网拉取 Docker Hub 超时时，腾讯云 CVM 可按[腾讯云官方说明](https://cloud.tencent.com/document/product/213/8623)配置内网加速源 `https://mirror.ccs.tencentyun.com`，保留原 Docker 配置。重启 Docker 前应确认没有其他运行中的容器。

## 运行

进入包含 `package.xml` 的工程根目录：

```bash
sudo bash cloud/run_docker.sh 20
sudo bash cloud/run_docker.sh 80
```

两次命令分别验证 20 秒和 80 秒的正式控制段，起飞、到位、停止、降落和绘图另计。脚本会构建并编译 catkin，运行测试、配置预检、3 UGV + 6 UAV 的 ROS mock 全流程、九台正式数据和真实 rosbag 核验，最后生成 13 张 PNG。

容器使用 `--network none`，全部 ROS 节点通过内部回环通信。`mock=true`，`allow_hardware=false`；不会连接实际 MAVROS 或底盘。执行完毕后容器自动删除，Docker 环境、镜像、源码与结果保留。

## 结果

`cloud-results/<UTC时间>_ros_mock/` 中包含：

- `run-status.json`：整个流程结果，只有 `success=true` 且 `last_stage=completed` 才是完整成功。
- `unit-tests.log`、`preflight.json`、`roslaunch.log`、`smoke.log`、`smoke-report.json`、`analysis.log`：验证证据。
- `output/<上海时间命名>/`：配置快照、telemetry JSONL/CSV、events、rosbag 和记录完成状态。
- `picture/<同一上海时间>/`：13 张图像、指标和中文分析摘要。

`smoke-report.json` 的成功仅表示 ROS 运行与记录检查通过；仍需检查整个 `run-status.json`，确保绘图也完成。正式分析按 `phase=RUNNING` 且实际时间戳 `stamp >= t0` 选取样本，统计横轴为 `stamp-t0`，排除提前发布 RUNNING 时尚在等待启动的记录。

模拟器使用理想动力学，不包含 PX4 飞控内环、实物定位、通信无线链路和机器人执行误差，不能把这些结果标为 PX4 SITL 或实物实验。

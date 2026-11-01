# Codex 云端 ROS Noetic 验证

这些文件用于将本工程放入 Codex 云环境后，编译并运行 3 台 UGV、6 台 UAV 的 ROS 接口模拟器。当前本地开发环境是 Windows；尚未执行 Docker 构建、云端 ROS 运行或 PX4 SITL。部署文件和本地测试通过，不表示云端验证已经通过。

## 云环境准备

本次指定仓库为 `GlenWhiteBJL/hete_TVFT_NE`，工程位于仓库根目录。具体环境字段和可直接粘贴的准备请求见 [ENVIRONMENT_SETUP.md](ENVIRONMENT_SETUP.md)。

1. 将本工程源文件放到你指定的 GitHub 仓库，保留本目录结构。无需上传 `output/`、`picture/` 或旧的 ZIP 文件。专用仓库的根目录可以直接是本工程。
2. 在 **Settings > Codex Cloud > Environments** 创建环境，选择该仓库。已有环境也必须能读取本工程。
3. 让环境准备阶段提供 Linux、可访问的本地 Docker daemon 和 BuildKit。这里用容器固定 Ubuntu 20.04 / Python 3.8 / ROS Noetic，不能假定云主机自身是 Ubuntu 20.04。
4. 按环境策略允许拉取官方 ROS 镜像和下载 Ubuntu、ROS 软件包。遵循云环境继承的代理和 CA 配置；下载失败时查看具体域名与策略，不关闭 TLS 校验或绕过代理。
5. 环境准备并测试完成后，检查结果并 **Publish**。只有保存配置不足以让新任务使用准备好的环境。

官方操作说明：[Cloud environments](https://learn.chatgpt.com/docs/environments/cloud-environments)。云环境中的代码来自所选仓库，本机桌面文件不会因创建环境而自动进入仓库。

准备环境时可以粘贴：

> 请准备当前仓库的 experiment-3x3 工程，确保本地 Docker daemon 可用，按环境策略配置所需的软件包/镜像下载。先检查托管环境网络策略和 Docker 状态，再在工程根目录执行 `bash cloud/run_docker.sh 20`。根据真实日志修复构建或 ROS mock 集成问题，保留全部报告。不要连接实机，不要修改 hardware 核验开关。完成后报告尚未通过的步骤，供我检查并发布环境。

如果仓库根目录就是本工程，直接在根目录执行；如果工程位于子目录，先进入包含 `package.xml` 的目录。

## 开始运行

环境发布后，在使用该环境的云端任务中执行：

```bash
bash cloud/run_docker.sh 20
```

这会构建镜像、执行单元测试和配置预检，再完成准备、解锁、起飞、初始点、控制、停止、降落流程，检查真实 rosbag 并绘图。20 秒是正式控制段时长，起降、记录收尾和首次构建另计。完整轨迹示例可运行：

```bash
bash cloud/run_docker.sh 80
```

容器运行期间使用 `--network none`，ROS master 和节点在容器内部回环通信，不发布主机端口。默认全部执行接口为 `/mock/...`，脚本也会核对节点参数和话题。无需 GPU。

## 获取结果

结果写入工程根目录的 `cloud-results/`：

- `build_*.log` 和对应 `.image-id`：Docker 构建记录与镜像 ID。
- `<时间>_ros_mock/run-status.json`：整个流程的退出码和最后执行阶段。
- 同目录下 `unit-tests.log`、`preflight.json`、`roslaunch.log`、`smoke.log`、`smoke-report.json`、`analysis.log`：逐阶段证据。
- `output/<实验时间>/`：本次 telemetry、元数据和 `raw.bag`。
- `picture/<同一实验时间>/`：分析图像和指标。

只有构建、测试、ROS 流程、数据核验和绘图全部成功，才把这次运行报告为通过。失败时同样保留日志；不要用旧的离线示例替代本次 ROS 输出。

通过这一流程能验证 ROS 通信、状态机、控制计算、记录与分析。PX4 固件内环、外部定位和实物机器人仍需分别验证。

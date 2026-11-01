请在当前指定仓库运行 3 UGV + 6 UAV 的 NE/TVFT 工程，并给出实际运行证据。

先找到本工程包含 package.xml 的目录，阅读 cloud/README.md。确认当前任务确实绑定到可用的云环境，检查当前网络策略以及托管 Docker daemon 状态；沿用环境提供的代理和 CA，不读取或打印凭证。

先执行 bash cloud/run_docker.sh 20，核实 catkin 编译、单元测试、ROS mock 的 1–7 状态流程、九台机器人本次 session 数据、rosbag 以及绘图。若失败，依据日志定位并修复后重试，不降低就绪或数据检查来掩盖失败，不改实机配置开关。20 秒通过后，再执行 bash cloud/run_docker.sh 80，保留完整示例。

请将 cloud-results/ 中本次结果整理成可下载的归档，包含构建日志、镜像 ID、运行报告、ROS 日志、telemetry、raw.bag、分析指标和图片。报告实际执行了哪些步骤、哪些失败或没有执行。ROS 接口模拟器是理想动力学模型，不要称为 PX4 SITL 或实物实验。

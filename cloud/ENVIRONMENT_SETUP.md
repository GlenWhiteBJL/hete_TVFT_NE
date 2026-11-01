# hete_TVFT_NE 的 Codex 云环境配置

这份配置供环境准备对话直接使用。它记录待应用的设置，不表示云环境已经创建或测试通过。

## 设置值

| 项目 | 设置 |
|---|---|
| 环境名称 | `hete-TVFT-NE-Noetic` |
| GitHub 仓库 | `GlenWhiteBJL/hete_TVFT_NE` |
| 分支 | `main` |
| 工作目录 | 仓库根目录，包含 `package.xml` |
| 运行方式 | Linux 上的本地 Docker daemon；容器固定 Ubuntu 20.04 / ROS Noetic / Python 3.8 |
| GPU、外部机器人、VPN | 本次接口模拟器均不需要 |
| Secrets | 本工程不需要额外密钥 |
| 使用范围 | 保留 `Only me` |

在环境网络设置中启用 Package managers；构建还需要访问 `snapshots.ros.org` 以及 Docker Hub 拉取域名 `auth.docker.io`、`registry-1.docker.io`、`production.cloudfront.docker.com`。如果实际拉取重定向到其他主机，应依据失败日志检查并添加所需域名；不要绕过现有网络策略。

参考：[Codex 云环境](https://learn.chatgpt.com/docs/environments/cloud-environments)、[Docker 官方域名说明](https://docs.docker.com/desktop/enterprise/allow-list/)。

## 在环境准备对话粘贴

```text
请配置当前 GlenWhiteBJL/hete_TVFT_NE 仓库的 ROS Noetic 实验环境。

工作目录是仓库根目录。先读取 cloud/README.md、cloud/Dockerfile 和云环境技能，检查当前托管环境、网络策略及本地 Docker daemon。沿用平台代理与 CA，不打印凭证，不使用远端 Docker daemon。若 Docker 不可用，按平台支持的方式恢复；无权限时报告实际缺少的条件。

不要在云主机默认 Python 中安装一套替代 ROS；使用 cloud/Dockerfile 中的 Ubuntu 20.04、Python 3.8 和 ROS Noetic。保留 Noetic 官方 final 快照源，不关闭 TLS 或软件包签名校验。

先在仓库根目录执行 bash cloud/run_docker.sh 20。这会编译 catkin、执行测试和预检，完成 3 UGV + 6 UAV mock 的 1–7 指令流程，核验本次九台机器人的统一启动数据和真实 rosbag，再绘图。根据真实日志修复错误后重试，不修改 hardware 核验开关，不通过降低检查标准来获得通过。

将经过测试的安装过程保存到环境 Install script，并在 Start skill 中记录本地 Docker 状态检查及 bash cloud/run_docker.sh 20 的用法。不要把本机路径、临时凭证或会话专用证书写入可复用配置。

完成后给出实际执行报告以及所有未通过项，让我检查并发布环境。发布后运行 bash cloud/run_docker.sh 80，整理 cloud-results/ 中本次记录、raw.bag、指标、图像和日志。ROS mock 不是 PX4 SITL 或实物实验。
```

准备阶段真实测试通过后，在配置页面检查结果、保存并 Publish；新任务使用发布后的环境。当前仓库没有已通过的云端 ROS 运行报告。

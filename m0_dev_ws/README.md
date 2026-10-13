# 阿克曼小车 M0 开发环境工作区

本工作区保存当前电脑的最小环境安装记录和验证示例。项目技术路线与验收以原始任务书为依据。这里的 TurtleBot3 演示使用差速底盘，只验证通用环境。

## 本轮结果

基础工具、ROS2 Jazzy Desktop、Gazebo Harmonic、ros_gz、Nav2 和 robot_localization 已安装。ROS 通信、最小 C++ 包编译运行、RViz/Gazebo 图形窗口以及官方差速仿真目标导航通过。Route Server 二进制包可用且已激活，不需要源码构建。

- [最终报告](验证报告/M0-report.md)
- [安装与复现步骤](验证报告/install-steps.md)
- [安装前核验及官方来源](验证报告/preflight.md)
- [实际版本清单](验证报告/installed-versions.tsv)
- [全部 Jazzy 软件包版本](验证报告/ros-installed-all.tsv)
- [验证结果](验证报告/verification-results.json)
- [问题记录](验证报告/issues.md)

## 使用方式

系统 ROS 位于 `/opt/ros/jazzy`，编译工具位于系统标准目录。没有修改 `.bashrc`；每次新开项目终端按需加载 `scripts/session.bash`。该文件只设置当前终端环境，并加载已构建的本工作区，不污染全局环境。

`src/m0_smoke` 是可编译运行的最小 ament C++ 示例。`scripts/verify_ros.py` 使用独立 DDS 域验证官方 C++/Python 发布订阅；`scripts/nav2_goal.py` 必须在已经启动的官方 TB3 仿真环境中运行，且 ROS_DOMAIN_ID 一致。本轮仿真使用 73，发布订阅检查使用 71，最小包检查使用 72。这些是检查隔离参数，不是项目长期配置。

本地 Git 已初始化；尚未配置作者身份和远程仓库，未提交或推送。构建产物和原始运行日志不纳入版本管理；关键结果、截图、版本和报告可纳入版本管理。

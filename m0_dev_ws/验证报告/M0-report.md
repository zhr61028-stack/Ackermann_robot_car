# 阿克曼小车 M0 最小开发环境安装与验证报告

日期：2026-10-03（Asia/Shanghai）。本轮开发电脑环境准备完成，可以进入阿克曼仿真模型与接口准备。通用差速仿真通过不能替代任务书的阿克曼导航验收。

## 已安装组件

| 组件 | 实际版本或状态 |
|---|---|
| Ubuntu / CPU | 24.04 系列 noble amd64；i7-14650HX |
| Git / Python | 2.43.0 / 3.12.3 |
| GCC / G++ | 编译器 13.3.0 |
| CMake / build-essential | 3.28.3 / 12.10ubuntu1 |
| colcon core / common extensions | 0.21.3 / 0.3.0 |
| rosdep / vcstool | 0.27.0 / 0.3.0 |
| ROS2 Desktop | Jazzy，元包 0.11.0 |
| RViz2 | 14.1.23 |
| Gazebo Sim | Harmonic 8.15.0；gz-sim vendor 包 0.0.13 |
| ros_gz | 1.0.24 |
| Nav2 | 1.3.13，含 Route Server、Smac、RPP、Collision Monitor |
| robot_localization | 3.8.3，含 EKF 和 navsat_transform |
| 官方 TB3 最小仿真包 | 1.0.1 |

完整 Debian 版本见 installed-versions.tsv；全部 Jazzy 包见 ros-installed-all.tsv。没有混装其他 ROS 发行版。

## 验证结果

| 验证 | 结果与证据 |
|---|---|
| 编译工具 / ROS CLI | 通过；版本命令和 ros2 --help 正常 |
| rosdep | 通过；初始化、普通用户 Jazzy 缓存更新、示例依赖检查成功 |
| 官方发布订阅 | 通过；C++ talker、Python listener，/chatter 有一个发布者和一个订阅者，实际收到 I heard |
| 独立工作区 | 通过；m0_smoke 编译启动，实际收到 m0_workspace_ok |
| RViz GUI | 通过；检查实际窗口截图，网格正常，OpenGL 4.6 |
| Gazebo GUI | 通过；shapes 场景模型正常显示，仿真 topic 可读取 |
| ROS–Gazebo 桥接 | 通过；官方仿真 /clock、/scan 实际消息，以及 cmd_vel 桥接日志正常 |
| 官方目标点导航 | 通过；TurtleBot3 Waffle 差速底盘，action 返回 SUCCEEDED |
| Route Server 启动 | 通过；默认图加载，生命周期 active [3] |
| Smac Hybrid-A* / RPP | 已安装并核实插件可用；阿克曼参数配置和实际运行未验证 |
| Collision Monitor | 已安装；项目安全接线和故障注入未验证 |
| robot_localization | 已安装；真实 IMU/轮速/GNSS 融合和 navsat_transform 未验证 |
| 阿克曼导航 / 实车 | 未验证；本轮未连接或驱动车辆 |

导航起点 (-2.0, -0.5)，目标 (-1.0, -0.5)，最终 map 到 base_link 的 TF 位置约 (-1.228, -0.555)，距目标约 0.234 m。成功来自官方示例容差；没有修改容差来促成通过。默认例使用自身规划控制配置，未借此宣称任务书 Smac+RPP 已验证。目标结果详见 nav2-goal-result.json。

## Route Server 结论

实际 APT 候选、已安装包和运行状态均已确认：ros-jazzy-nav2-route 1.3.13 可直接安装。无需升级 ROS、混装发行版或源码回移；维护方式与当前 Jazzy 二进制一致。下一阶段仍需建立项目 WGS-84 GeoJSON 路网及坐标转换、接入 GNSS 路段判断、编写局部目标生成器；本轮没有验证校园路网计算或路口处理。

## 网络与系统变更

Ubuntu 软件源的 universe、updates 和 backports 原本启用，未修改其配置。新增 ros2-apt-source 1.3.0~noble 管理的官方 ROS 源和密钥。APT 实际下载及签名验证通过，rosdep 官方规则下载通过。

当前助手进程有代理变量；现有普通终端启动进程没有这些变量，bashrc/profile 未发现代理设置，但不排除某个交互 shell 后续临时 export。APT 未发现显式代理配置，通过 pkexec 执行的实际软件源刷新和包下载成功。未输出代理凭据。ROS 官方 HTTPS 主机名验证失败，使用官方配置包提供的 HTTP 加 APT 签名验证；没有禁用验证。

没有系统整体升级、删除软件包、修改 NVIDIA 驱动或安装 CUDA。ROS Desktop 依赖连带更新 8 个 Mesa 图形库，已在审批中说明；安装后实际图形验证通过。NVIDIA 驱动复核仍为 595.91.07。没有修改 .bashrc 或全局环境；scripts/session.bash 仅供项目终端按需加载。

## 工作区与磁盘

工作区：/home/zhanghaoran/.codex/.chatgpt-projects/g-p-6ac0a6b986d08191a18b6f104acd916d/m0_dev_ws

ROS 安装位置：/opt/ros/jazzy。编译工具使用系统标准位置。安装后系统分区剩余约 105 GB；/home 仍约 47 GB。后续 rosbag 写入 /home，需要单独管理容量。

Git 本地仓库已初始化，作者身份和远程仓库尚未配置，未 commit/push。版本清单、步骤、结果、问题和示例源代码已保存，适合核对后形成首个 M0 提交。未修改 sources/ 参考文件。

## 下一阶段所需资料

仿真与实车参数仍全部待确认：轴距、轮距、轮径、车体外形、转向范围、实测最小转弯半径；底盘接口/CAN 或串口协议、轮速反馈、MCU 超时停车与超声波硬停、遥控急停优先级；激光、双目、IMU、GNSS 型号及同步/外参/驱动；车载机型号和系统，尤其是否 JetPack 6。已有驱动仓库、标定文件、可用测试路线及车辆场地安排也需提供。

## 官方来源

- [ROS Jazzy 官方安装文档源文件](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/Ubuntu-Install-Debs.rst)
- [ROS 官方软件源配置说明](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/_Apt-Repositories.rst)
- [Gazebo 官方 ROS 配对说明](https://gazebosim.org/docs/jetty/ros_installation/)
- [Nav2 Jazzy 安装说明](https://docs.nav2.org/jazzy/getting_started/build_and_install/local_installation/)
- [Nav2 官方 TB3 示例](https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/)
- [官方 Jazzy 发布清单](https://raw.githubusercontent.com/ros/rosdistro/master/jazzy/distribution.yaml)

版本和 Route Server 可用性以本轮实际签名验证后的 APT 索引及已安装状态为准。详细日志、截图和问题记录保存在 验证报告/。

# M0 环境核验记录

检查日期：2026-10-03（Asia/Shanghai）

## 当前阶段
安装前核验完成，管理员认证待完成；组件安装及运行验证尚未完成。

## 官方依据
- https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/Ubuntu-Install-Debs.rst
- https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/_Apt-Repositories.rst
- https://gazebosim.org/docs/jetty/ros_installation/
- https://docs.nav2.org/jazzy/getting_started/build_and_install/local_installation/
- https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/
- https://raw.githubusercontent.com/ros/rosdistro/master/jazzy/distribution.yaml
- http://packages.ros.org/ros2/ubuntu/dists/noble/main/binary-amd64/Packages.gz

## 兼容性和候选组件
Ubuntu noble amd64 支持 Jazzy；Jazzy 官方搭配 Harmonic，通过 ros-jazzy-ros-gz 安装对应 vendor 依赖，无需增加其他 Gazebo 发行版源。
实际官方索引存在 ros-jazzy-nav2-route 1.3.13 二进制包，无需源码构建。Smac Hybrid-A* 属于 nav2_smac_planner；RPP、Collision Monitor 和 robot_localization 均有 Jazzy 二进制。候选完整版本及哈希见 official-package-candidates.json。候选版本不等于已安装版本。

## 电脑状态
Ubuntu 24.04 noble x86_64，i7-14650HX，32 GiB 内存，RTX 4060 系列约 8 GiB 显存，驱动 595.91.07 正常。Git 2.43.0、Python 3.12.3 已安装；ROS/Gazebo/Nav2/编译工具/colcon/rosdep 未发现安装。系统盘剩余 110 GB；home 剩余 47 GB。

## 网络和代理
当前助手进程有代理变量；不记录代理地址或凭据。普通终端代理未确认，bashrc/profile 未发现代理配置；APT 配置未发现显式代理，管理员进程实际网络待验证。
当前进程代理路径和禁用环境代理的直连路径均可访问 Ubuntu HTTP 索引、ROS HTTP InRelease 和 GitHub HTTPS。packages.ros.org HTTPS 两种路径都报告证书主机名不匹配，未关闭证书校验。官方源配置包采用 HTTP 加签名验证；下载包 SHA256 与 GitHub release asset digest 一致。下载索引仅用于候选查询，安装前必须经 APT 签名验证。

## 安装规划
1. 官方 ros2-apt-source 包；apt-get update 必须成功。
2. 基础工具：build-essential cmake ros-dev-tools（含 colcon/rosdep/vcstool 等）及必要辅助工具。先模拟安装确认空间、删除与升级清单。
3. ros-jazzy-desktop；rosdep init（存在则跳过），以普通用户运行 rosdep update。
4. ros-jazzy-ros-gz。
5. ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-route ros-jazzy-robot-localization 和官方 TB3 示例。
不执行系统 upgrade/full-upgrade，不改驱动或安装 CUDA，不追加 shell 启动文件。

## 验证规划
工具版本、ROS CLI、rosdep；官方 C++ talker 与 Python listener；独立 ament 示例构建运行；RViz/Gazebo GUI 实际启动；官方 TurtleBot3 差速仿真导航到目标。所有运行项目当前未验证，差速示例不替代阿克曼验收。

## 后续硬件资料
轴距/轮距/轮径/外形/转向限制/实测转弯半径，底盘接口协议与 MCU 失联停车机制，轮速编码器，雷达/双目/IMU/GNSS 型号与同步标定，安全传感器及独立硬停接线，车载机系统，遥控急停接口均待确认。

## 已完成操作更新
官方 ros2-apt-source 1.3.0~noble 已安装；APT 更新成功并验证软件源签名。通过 pkexec 系统认证执行管理员操作，不读取密码。APT 日志显示 Ubuntu 索引访问 cn.archive 和 mirrors.tuna；未由本任务修改 Ubuntu 源，可能为服务器重定向。基础工具安装模拟 0 升级、93 新装、0 删除，安装成功。GCC/G++ 13.3.0、CMake 3.28.3、colcon CLI、rosdep 0.27.0 验证通过。ROS Desktop 以不安装推荐包模式进行：模拟 902 新装、8 Mesa 依赖库更新、0 删除；无 NVIDIA 软件包变更。

## 运行验证更新
ROS Desktop 安装完成，ROS CLI 正常；官方 C++ talker/Python listener 通信通过，保存 nodes/topics/chatter_info 和收发日志。m0_smoke 构建成功，运行后收到 m0_workspace_ok。rosdep init/update Jazzy 和 rosdep check 均通过。Harmonic/ros_gz 安装完成；gz sim --versions 为 8.15.0。RViz 实际窗口网格渲染正常，OpenGL 4.6；独立启动无 TF 时的警告是预期现象，不等于车辆 TF 已验证。Gazebo shapes 场景 GUI 和仿真 topic 正常，截图已保存。Nav2 安装模拟为 120 新装、0 升级、0 删除。

## 最终状态
本轮全部安装阶段与通用环境验证完成，参见 M0-report.md。Nav2 官方差速目标成功，Route Server 与 planner 生命周期均 active [3]。仿真 /scan 和 /clock 实际读取成功。普通终端启动进程无代理变量。最终系统盘约 105 GB，home 约 47 GB。

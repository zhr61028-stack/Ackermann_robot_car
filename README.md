# 阿克曼小车：M3 仿真交付

**M3 仿真完成，实车验收待做。** 当前仅验证本地平面测试路网、已知测试道路地图、Gazebo真值定位与假设车辆参数；未验证校园WGS-84/GNSS、真实感知/里程计、MCU或实车安全。技术路线以导师任务书为依据，充分复用ROS2/Nav2/Gazebo成熟组件。

本仓库由项目根目录统一管理，m0_dev_ws与ackermann_sim_ws都是普通文件夹，**没有子模块**。旧工作区Git元数据保存在原开发电脑的本地备份，未删除；此前两个工作区均无正式提交历史。仓库为公开，按作者2026-10-13明确授权上传。提交作者zhr61028-stack，邮箱使用作者确认值。

## 开发环境与用途

| 项目 | 本次验证环境 |
|---|---|
| 系统 | Ubuntu 24.04 LTS，x86_64 |
| ROS | ROS 2 Jazzy Desktop，含RViz与演示节点 |
| 仿真 | Gazebo Harmonic；ros_gz桥接 |
| 导航 | Nav2，Route Server、Smac Hybrid-A*、Regulated Pure Pursuit |
| 依赖 | colcon、CMake、编译工具、rosdep；Python YAML/NumPy；分析及录像工具使用Matplotlib/OpenCV/X11 |
| 版本记录 | [M0版本与安装核验](m0_dev_ws/验证报告/M0-report.md) |

m0_dev_ws保存最小环境示例与调查；ackermann_sim_ws保存车辆、导航、测试图、脚本及M3报告。src、launch、config、resource等目录保留ROS约定名称。

## 下载后构建与启动

先具备上述环境。首次安装依赖前检查当前系统状态，参考[已执行安装记录](m0_dev_ws/验证报告/install-steps.md)，不要未经检查整体重装或修改NVIDIA驱动。

在普通终端执行这一组，保存至~/projects；正常结果为出现Ackermann_robot_car目录：

```bash
mkdir -p ~/projects
cd ~/projects
git clone https://github.com/zhr61028-stack/Ackermann_robot_car.git
```

进入仿真工作区，加载Jazzy并构建。正常应显示3个软件包完成；若失败，保留第一条错误，不手改build/install：

```bash
cd ~/projects/Ackermann_robot_car/ackermann_sim_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

仍在该目录，启动原粗路网路线B（只启动，不驾驶实车）：

```bash
bash scripts/start_route.bash start:=B
```

正常打开Gazebo/RViz，并显示Managed nodes are active。保留此终端。新开第二终端，仅提交终点5：

```bash
cd ~/projects/Ackermann_robot_car/ackermann_sim_ws
bash scripts/request_route.bash 5
```

虚拟车先右转、再左转，约一分钟到达并停车。异常时查看启动终端的FAILED/TF/Planner错误，不发人工速度绕过限制。结束按Ctrl+C，等旧实例退出后再开同一模式。脚本只设置当前进程ROS环境，不修改.bashrc。

其他入口：scripts/start.bash基础控制、scripts/start_nav.bash简单场地、scripts/run_route_long_distance.bash长距离、scripts/run_m3_closure_tests.py测试断流。断流工具必须先加载Jazzy和install/setup.bash，先用--dry-run检查新目录。详细解释见[下载复现与附件规则](项目发布说明/下载复现与附件规则.md)。

## 当前有效验收证据

| 内容 | 结果与入口 |
|---|---|
| 小型粗路网闭环 | 两路线、路口/分支与不可达停车：[报告](ackermann_sim_ws/验证报告/当前有效/M3_粗路网仿真/report.md) |
| 稳定性与边界 | 近极限弯道、连续路口、测试地图已知封路：[报告](ackermann_sim_ws/验证报告/当前有效/M3_稳定性与边界/report.md) |
| 长距离 | 先100m，再两次310m图边几何长度；实际303.808/303.810m：[报告](ackermann_sim_ws/验证报告/当前有效/M3_长距离粗路网仿真/report.md) |
| 收尾补测 | 310m场景完整负载，两次输入断流；保护61.7/61.2ms，首次停车200.7/207.9ms，确认400.8/408.0ms：[报告](ackermann_sim_ws/验证报告/当前有效/M3_仿真收尾/report.md) |
| 总览与下一步 | [M3交付总览](ackermann_sim_ws/项目文档/M3_仿真交付总览.md)、[实车接入](ackermann_sim_ws/项目文档/实车接入清单.md)、[节点审查](ackermann_sim_ws/项目文档/M3_自写节点审查.md) |

源码、场景、统计结果和必要小图已纳入版本；原始历史报告保留当时的路径/哈希，不篡改实验结果。当前执行入口使用相对工作区路径，历史命令可能含原开发机绝对路径，请使用本页的新下载流程。

## 限制与本地附件

禁止原地旋转/Spin，当前验证仅前进行驶，保护器保留曲率/速度限制；假设车体参数待实测。道路边界和障碍由已知测试地图提供，不能当真实感知。输入断流实验保持保护节点存活，不等于其崩溃、物理断连或MCU安全；规划失败停车也不等于紧急避障。

**大型视频和rosbag没有上传，也尚未获得GitHub远程备份。** 原开发电脑保留在ackermann_sim_ws/实验记录，各实验的演示视频/与原始数据/bag/需要单独备份；[本地附件清单](项目发布说明/本地附件备份清单.md)列出路径和大小。历史报告中的这些本地文件链接不是GitHub在线附件；图片/统计结果可浏览，视频和bag须另带数据包。sources/、本地Git备份、登录工具/凭据、构建产物、缓存和旧调试原始记录也不上传。

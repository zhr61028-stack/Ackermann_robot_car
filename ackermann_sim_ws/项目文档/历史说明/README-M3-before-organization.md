# M3 前置仿真：简单场地自主导航

本轮已通过：虚拟阿克曼车辆直行、转弯、绕过已知固定障碍到达目标。车辆参数全部是“仿真假设，待实测”；直接使用 Gazebo 模型真实位置，是定位测试模式。没有完成校园粗路网或实车定位、安全验收。

## 打开虚拟车辆

在普通 Ubuntu 终端、任意目录执行这一条命令。脚本自行加载 ROS 2 和项目环境，不需要改 .bashrc，也不需要安装软件：

```bash
bash /home/zhanghaoran/.codex/.chatgpt-projects/g-p-6ac0a6b986d08191a18b6f104acd916d/ackermann_sim_ws/scripts/start_nav.bash
```

正常会打开 Gazebo 和 RViz，车辆静止在原点，场地有一个橙色固定障碍。终端稍后应出现 `Managed nodes are active`。保持这个终端运行。脚本仅对子进程设置 ROS_DOMAIN_ID=82、GZ_PARTITION=ackermann_m3，与基础控制的 81 域分开。

如果窗口或导航节点没有正常出现，保留当前日志，先查看最早的 ERROR / Traceback；不要重复启动多个实例。按 Ctrl+C 关闭本次启动。不要把基础控制脚本的 81 域命令当作导航域控制命令。

## 运行三个目标测试并录包

等上一步正常启动后，打开第二个终端，在任意目录执行：

```bash
bash /home/zhanghaoran/.codex/.chatgpt-projects/g-p-6ac0a6b986d08191a18b6f104acd916d/ackermann_sim_ws/scripts/validate_nav.bash --output 实验记录/my-nav-run --video
```

输出目录必须是一个尚不存在的新目录，脚本拒绝覆盖。测试要求车辆从原点开始；再次测试前，在第一个终端 Ctrl+C 关闭并重新启动。测试依次发送 (4,0,0)、(10,3,0)、(19,3,0) 三个目标，坐标单位为米、角度为弧度，均在 odom 坐标系。

正常会显示 GOAL straight / turn / known_obstacle，各目标 status=4，最后 ALL_PASS True。约一分钟完成；视频、结果和 MCAP 录包保存在工作区内指定的新目录。控制来自 Nav2，测试脚本只发送目标并观察，不代替规划器驾驶。

如果 ALL_PASS False 或导航超时，先查看该目录 goal-results.json、results.json 与终端日志，保留录包。本轮失败记录和处理见报告；不要放宽车辆限制或终点容差来绕过失败。

--video 需要两个图形窗口都打开；隐藏窗口也可由程序读取应用自身画面，但不要关闭或最小化窗口。无图形测试可使用 start_nav.bash gui:=false rviz:=false，并在测试命令中省略 --video。

## 查看已经完成的演示

- 最终视频：实验记录/m3-nav-final-v2/demo.mp4，65.7 秒，左侧 Gazebo、右侧 RViz。红色是 Smac 规划，绿色是实际模型轨迹，蓝色为独立轮里程计。
- 总览轨迹图：实验记录/m3-nav-final-v2/navigation_trajectory.png。
- 最新报告：验证报告/M3-preliminary-report.md。
- 精确结果：实验记录/m3-nav-final-v2/results.json。
- 导航录包：实验记录/m3-nav-final-v2/bag/。
- 完整负载下停车录包：实验记录/m3-watchdog-loaded/bag/，结果同目录 results.json。

视频可在 Ubuntu 文件管理器中打开工作区对应文件。不得将旧失败目录 m3-nav-final 或 m3-nav-headless 作为通过演示。

## 参数在哪里

- src/ackermann_sim/config/vehicle.yaml：统一车辆尺寸、轴距、轮距、转角、速度、动力学和停车保护假设。
- src/ackermann_nav/config/test_scene.yaml：测试边界、固定障碍、目标与预设检查容差。
- src/ackermann_nav/config/nav2.yaml：Smac Hybrid-A* / RPP 的基础配置。
- src/ackermann_nav/config/forward_only.xml：复用 Nav2 官方按路径失效重规划行为树，移除 Spin / BackUp。

启动生成物理场景、占据地图、桥接配置、完整车辆包络和规划半径，避免手工在多个文件重复填车辆参数。快照见 验证报告/m3-generated，实际 ROS 参数见 验证报告/m3-runtime-params。运行时临时生成文件位于 /tmp/ackermann_nav_*，不修改 sources/。

## 修改代码后重新构建

先切换到独立工作区：

```bash
cd /home/zhanghaoran/.codex/.chatgpt-projects/g-p-6ac0a6b986d08191a18b6f104acd916d/ackermann_sim_ws
```

再执行以下两条命令，加载现有 ROS 环境并构建本工作区：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

正常结果是 ackermann_sim 和 ackermann_nav 两个包完成、没有失败包；如果失败，先查看首个错误，不继续启动。配置文件的参数值虽可通过符号链接复用，新文件仍需重新构建以安装到 share 目录。

做有价值的修改后，先 git status、查看 git diff（暂存内容用 git diff --cached），再决定 commit；设置自己的作者身份与远程仓库后再 push。不使用虚构身份自动提交。

## 如需重放录包

先关闭正在运行的仿真，避免同一域出现重复 /clock、/tf 或控制发布者。在新终端按顺序执行一小组：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=83
ros2 bag play /home/zhanghaoran/.codex/.chatgpt-projects/g-p-6ac0a6b986d08191a18b6f104acd916d/ackermann_sim_ws/实验记录/m3-nav-final-v2/bag
```

83 为独立重放域，此时不启动车辆桥接。该设置仅作用于当前终端。录包包含原始 /clock；不额外加 --clock 产生第二个仿真时钟。按 Ctrl+C 结束重放。

# 基础控制：手动驾驶虚拟车

此模式不运行Nav2，ROS域为81；自主导航使用82域，见 [导航说明](m3-navigation.md)。当前启动资源核验结果见 [整理记录](organization.md)。

## 第一个终端：启动

在任意目录执行一条命令，脚本加载现有ROS环境并启动Gazebo、RViz和虚拟车：

```bash
bash /home/zhanghaoran/.codex/.chatgpt-projects/Ackermann_robot_car/ackermann_sim_ws/scripts/start.bash
```

正常应出现两个窗口，车辆静止，RViz状态正常。保持终端打开，Ctrl+C关闭。若失败先保留最早报错，不重复启动或重新安装。

## 第二个终端：控制

先进入工作区：

```bash
cd /home/zhanghaoran/.codex/.chatgpt-projects/Ackermann_robot_car/ackermann_sim_ws
```

再执行一条控制命令：

```bash
bash scripts/control.bash forward
```

正常看到车辆前进。每次只运行一个发布器；Ctrl+C停止发布后保护输出零速度，车辆随后减速。可将 forward 换为 left、right、stop。rotate用于测试原地旋转拒绝，车辆应保持静止。发布频率40Hz，不含实车协议。

保护阈值目前60ms，检查周期2ms；触发与实际停止时间不同，见 [当前报告](../验证报告/当前有效/M3_前置仿真/report.md)。所有物理参数是待实测的仿真假设。

## 自动测试和构建

从原点新场景开始，在工作区目录的第二个终端执行：

```bash
bash scripts/validate.bash 实验记录/M1_底盘与控制/过程记录/仿真_基础控制_新试验
```

新目录必须不存在，脚本拒绝覆盖。约37秒验证基础运动、约束和指令中断，保存视频、rosbag与results.json；检查all_pass，失败先分析证据，不调整阈值。录像要求两个图形窗口存在；X11已验证，Wayland录像未验证。

修改代码或需重新构建时，在工作区执行一小组：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

前者加载当前终端环境，后者构建工作区。正常是ackermann_sim、ackermann_nav两个包完成；失败先查首个错误，不在项目根目录构建。

[旧基础演示](../实验记录/M1_底盘与控制/过程记录/仿真_基础车辆控制旧版本/)保留当时参数及停车结果。基础模式使用原生轮里程计/odom；导航测试模式用真值/odom、原生数据放在/wheel/odom，两种定位含义不能混用。

### 新场景与重启提示

本轮路径修复后已验证干净启动。需要重测时，在原启动终端Ctrl+C，等该终端完全退出、旧窗口关闭后再启动；不要重叠开启相同域实例。不要用Gazebo的Reset按钮替代完整重启：时间重置后真值消息恢复存在未解决问题，本轮没有改算法处理。

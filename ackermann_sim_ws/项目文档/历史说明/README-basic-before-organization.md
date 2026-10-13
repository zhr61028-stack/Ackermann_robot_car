> 2026-10-12：最新 M3 前置导航启动流程与验证见 [README-M3.md](README-M3.md)。本页为基础车辆控制说明；保护参数已更新为 60 ms 提前触发、2 ms 检查，详见新报告。

# 阿克曼小车最小仿真工作区

M0 状态保持为“开发环境与通用仿真已完成，硬件盘点待补”。本工作区完成基础阿克曼仿真，不接入完整导航或实车。

- [本轮报告](验证报告/report.md)
- [假设参数唯一配置入口](src/ackermann_sim/config/vehicle.yaml)
- [演示视频](实验记录/final-run/demo.mp4)
- [轨迹图](实验记录/final-run/trajectory.png)
- [停车计时图](实验记录/final-run/stopping.png)
- [验证数据](实验记录/final-run/results.json)
- [问题记录](验证报告/issues.md)

## 已构建工作区的一条启动命令

可以在任何目录执行：

```bash
bash /home/zhanghaoran/.codex/.chatgpt-projects/g-p-6ac0a6b986d08191a18b6f104acd916d/ackermann_sim_ws/scripts/start.bash
```

它进入本工作区、加载 Jazzy 和本包，仅为该进程设置 ROS_DOMAIN_ID=81 与 GZ_PARTITION=ackermann_m0，启动平坦场地、车辆、桥接、保护节点、TF/轨迹观察、Gazebo 和 RViz。不会安装软件或修改 .bashrc。正常应出现两个窗口，蓝色四轮车初始静止，RViz Global Status 正常。保持此终端打开；结束时 Ctrl+C 关闭本次启动。

若没有窗口或节点退出，先阅读终端报错；查看本包对应 launch 日志、/odom 与 TF，不重新安装环境。启动前确保没有同分区的旧实例，不要同时启动两个 start.bash。

## 另一终端手动控制

在本工作区目录执行，每次只运行一个控制发布器：

```bash
bash scripts/control.bash forward
```

可选动作：forward、left、right、stop、rotate。forward/left/right 连续以 40 Hz 发送仿真指令。rotate 专门测试原地旋转拒绝，车体应保持静止。按 Ctrl+C 停止发布后，保护节点超时输出零速度，车辆随后减速。该命令仅连接虚拟车辆的独立 ROS 域，不含任何底盘协议。

## 自动验证与录像

先用启动命令从新场景开始；在另一终端进入本工作区后执行：

```bash
bash scripts/validate.bash 实验记录/my-new-run
```

输出目录必须不存在，脚本拒绝覆盖。约 37 s 自动依次验证前进、停车、左转、右转、原地旋转拒绝、限幅、指令中断，保存视频、rosbag、原始观测及结果。录像使用 X11 compositor 的程序自身缓冲区，不能回退到桌面截图；本轮 X11 环境已验证，Wayland 环境下录像未验证。车辆可在 Gazebo 固定视角中驶出画面，右侧 RViz 可观察完整轨迹。

最终超时和实际停止时间在 rosbag 关闭后由完整记录计算。录像进程的现场订阅可能漏采样，其日志不能直接用于安全计时。查看 results.json 的 watchdog 和 all_pass；若检查失败，先看该次日志与录包，不调整检查阈值来促成通过。

## 重新构建

本轮已构建，不需重复安装。更换电脑或修改包入口后，在本工作区目录依次执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

第一条只加载当前终端环境，第二条只在本工作区生成 build/install/log。正常显示一个包 ackermann_sim 构建成功；失败先读取该包编译日志。修改 vehicle.yaml 后重启即可由同一配置生成模型、URDF 和桥接，不编辑 /opt/ros 的官方示例。

## 数据与回放

最终 rosbag 目录为 实验记录/final-run/bag，包含 /clock、/cmd_vel、/sim/cmd_vel、/sim/watchdog、/odom、/ground_truth/poses、/joint_states、/tf、/tf_static 和阶段标签。record 采用系统记录时间，消息内部仍保留仿真时间。

回放前停止 live 场景，避免两个时钟/TF 发布者；在另一个独立 ROS 域回放，不启动控制桥接。视频可直接用播放器打开，无需启动 ROS。原始 MCAP 和 observations.json 已保留在磁盘但不纳入 Git，大体积数据后续另行备份。

所有尺寸、惯量、摩擦、转角和动态响应均为“仿真假设，待实测”。保护节点本身失效时不具备独立硬停；本轮测试不能作为实车安全验收。

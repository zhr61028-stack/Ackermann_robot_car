# 本轮安装步骤与复现记录

执行目录为本工作区。系统操作通过 pkexec 正常系统认证完成；用户密码仅输入系统认证窗口。以下是记录，不应在其他机器上未经检查直接全部执行。

1. 读取系统版本、架构、磁盘、GPU、源和代理变量是否存在。确认 Ubuntu noble amd64、universe、updates/backports 已启用。
2. 从官方 GitHub ros-infrastructure/ros-apt-source 的 1.3.0 release 下载 ros2-apt-source_1.3.0.noble_all.deb；SHA256 与 release asset digest 一致，记录于 source-package.json。检查包内容后执行 `pkexec /usr/bin/dpkg -i <下载包绝对路径>`，新增官方 ROS 源与签名密钥。
3. 执行 `pkexec /usr/bin/apt-get update`，成功验证官方源签名。
4. 每批安装先执行相同包列表的 `apt-get -s --no-remove install ...` 或含 `--no-install-recommends` 的模拟，计划分别保存在 stage1/2/3/4-plan.txt。实际安装：

```bash
pkexec /usr/bin/apt-get -y --no-remove install build-essential cmake python3-colcon-common-extensions python3-rosdep python3-vcstool
pkexec /usr/bin/apt-get -y --no-install-recommends --no-remove install ros-jazzy-desktop
```

基础工具验证后进入 ROS 阶段；ROS Desktop 的 8 个 Mesa 库依赖更新已在审批中说明，无 NVIDIA 包变更。实际桌面安装计划为 stage2-minimal-plan.txt。

5. 依赖管理初始化前确认默认文件不存在，执行 `pkexec /usr/bin/rosdep init`；已存在时跳过。普通用户执行 `rosdep update --rosdistro jazzy`，并在工作区执行 `rosdep check --from-paths src --ignore-src --rosdistro jazzy`。
6. 当前终端加载 `/opt/ros/jazzy/setup.bash`。执行 `python3 scripts/verify_ros.py`，验证官方发布订阅；执行 `colcon build --symlink-install --packages-select m0_smoke`；加载工作区环境，启动最小节点并订阅 /m0_smoke，收到 m0_workspace_ok。
7. 上一阶段通过后执行：

```bash
pkexec /usr/bin/apt-get -y --no-install-recommends --no-remove install ros-jazzy-ros-gz
```

8. 启动 RViz 和 `gz sim -r shapes.sdf`；保存窗口截图和日志，确认图形实际显示。Gazebo 版本由 `gz sim --versions` 查询。
9. 上一阶段通过后执行：

```bash
pkexec /usr/bin/apt-get -y --no-install-recommends --no-remove install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-route ros-jazzy-robot-localization ros-jazzy-nav2-minimal-tb3-sim
```

10. 加载 ROS 环境，使用 `ROS_DOMAIN_ID=73 ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False` 启动官方仿真；另一个同域进程执行 `ROS_DOMAIN_ID=73 python3 scripts/nav2_goal.py`。设置官方示例初始位姿与目标，记录 action 成功结果及最终 TF。读取 Route Server/planner 生命周期、/scan、/clock。结束后关闭本轮启动实例。

没有运行系统 upgrade/full-upgrade/autoremove，没有修改 NVIDIA 驱动或安装 CUDA，没有修改 shell 启动文件。未主动安装 FAST-LIO2、KISS-ICP、Patchwork++、stereo_image_proc 或 rtabmap。Desktop 自身依赖包含图像/点云基础库，这不表示双目导航已搭建。

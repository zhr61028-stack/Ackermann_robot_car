# 目录整理记录

本轮只整理说明与证据分类，不修改src、scripts、算法、车辆参数或原始结果，不安装、不提交或推送。

## 移动范围

| 整理前 | 整理后 |
|---|---|
| 工作区README、README-M3 | 原文归项目文档/history；新的统一入口与docs操作说明 |
| 验证报告/M3-*.md、来源 | 验证报告/当前有效/M3_前置仿真/report.md、issues.md、sources.json |
| 验证报告/m3-generated、m3-runtime-params | 验证报告/当前有效/M3_前置仿真/配置快照/生成配置、runtime |
| 验证报告/m3-*-publishers.txt | 验证报告/当前有效/M3_前置仿真/evidence/ |
| records其他非日志记录 | 验证报告/历史记录/基础仿真/ |
| 验证报告/*.log | 验证报告/logs/ |
| 实验记录/m3-nav-final-v2 | 实验记录/latest/navigation/ |
| 实验记录/m3-watchdog-loaded | 实验记录/latest/watchdog/ |
| 实验记录/m3-contact-probe | 实验记录/latest/contact-probe/ |
| artifacts其余旧试验 | 实验记录/history/，状态见实验记录/README |

旧验证报告/artifacts路径通过相对符号链接兼容；README-M3.md指向当前说明。旧README原文与全部119个移动文件保持原字节，校验清单见organization-manifest.json。旧报告数字及来源含义不变。

## 外部目录更名

整理执行期间，原项目目录g-p-6ac0a6b986d08191a18b6f104acd916d从文件系统消失，内容出现在Ackermann_robot_car。该根目录更名不是本轮整理脚本执行的操作。

原install和build存在指向旧根目录的绝对链接，旧链接当时失效。用户随后授权修复：先将两个工作区原build/install备份到项目文档/relocation-backup/，再修正103项生成链接/文本路径，两个工作区重新构建通过。源码、脚本、车辆参数及历史快照原字节未变。新操作说明采用新根目录路径。

## 核验

已核验119个移动文件原字节、51个受保护源码/脚本/参考文件、83个Markdown链接、旧路径兼容入口及脚本语法，全部通过；9个MCAP文件和5个视频均保留。结果见 [核验清单](organization-verification.json)。原58个失效构建链接已修复，现为0个。已实测M0示例收到三条消息；干净启动下Nav2为active，/clock、/map、/odom可读，车体及车轮TF正常，Gazebo/RViz窗口实际显示。基础及导航launch资源另作参数解析检查；没有重跑车辆行驶、目标到达或停车验收。启动资源可用性单独记录，不能用文档链接通过代替真实启动通过。

build、install、log和__pycache__为自动产物，可跳过阅读，不删除。验证快照、视频和bag属于证据，必须保留。

## 启动核验过程与边界

首次窗口出现但场景暂停/时间重置，真值消息没有恢复，坐标检查失败；首次结果和恢复运行后的复查均保留。随后一次快速重启中planner生命周期配置失败，记录为启动竞争的候选原因，未修改Nav2参数。确认旧节点退出后，干净实例全部启动核验通过。

当前有效启动证据见 [干净启动结果](startup-clean-check/verification.json)；前次诊断在relocation-startup-verification.json、startup-recheck/、startup-fresh-check/及对应启动日志。Gazebo内Reset之后的真值恢复问题未修复、未作为已通过项目；需要新场景时Ctrl+C关闭，等旧终端完全退出再重新启动。

构建备份属于自动产物的旧版本，仅为回退保留，平时可跳过；不得拿备份中的旧链接作为当前启动资源。

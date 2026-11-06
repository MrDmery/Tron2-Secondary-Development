# TRON2 双臂动作真机程序

从 `~/Course_Code` 运行 `sdk_examples_real_robot/sdk_examples/choreography_live.py`。本脚本依赖同目录的 `client.py`、`lessons.py`、`fk_ik_two_points_reset.py`，以及 `Course_Code/choreography_candidate.py` 和课程 MJCF；将这些更新后的文件同步到 Ubuntu 后运行。

动作：左臂抬起（FK1，右臂不动）、左臂上下摆两次、双臂胸前造型（FK2）、复位、头部先转到零位一侧 +0.60 rad、再转到另一侧 −0.60 rad（横跨约 1.20 rad）并回到启动角、左右臂交替前后摆动并由真机控制器执行两个 8 mm MoveP 点（IK1、IK2）、最后返回启动关节姿态。头部转动只在双臂准备姿态进行，大角度转动自动延长 MoveH 时长；启动 yaw 必须在零位 ±0.30 rad 内，否则在运动前拒绝执行；仍须检查报告中的三个绝对目标角。夹爪依次开合并在末尾恢复启动开口度。`--send` 一次启动，途中不再要求输入 `y`，每段仍检查 SDK 反馈。

## 与 2 个 FK 点、2 个 IK 点的关系

| 点位 | 动作位置 | 输入与运算 | 报告字段 |
|---|---|---|---|
| FK1 | 左臂举起后 | 给定关节角，课程模型 FK 计算左右腕部位置；同时读取真机 SDK 手部位姿 | `fk_points.FK1`、`fk_observations.FK1` |
| FK2 | 双臂胸前造型后 | 再取一组关节角，做相同 FK 计算和实测记录 | `fk_points.FK2`、`fk_observations.FK2` |
| IK1 | 左臂前摆、右臂后摆后 | 向控制器给定左右末端各 8 mm 的反向 X 位移，由控制器求关节运动 | `ik_points.IK1.controller_ik` |
| IK2 | 左臂后摆、右臂前摆后 | 再给定一组反向位移，由控制器求关节运动 | `ik_points.IK2.controller_ik` |

可向老师解释为：“两次 FK 展示从关节角求腕部位置，两次 IK 展示从目标手部位姿求关节运动；摆手、比心、头部和夹爪动作负责把四个计算点串成一套表演。”`fk_observations` 中保存课程模型腕部 XYZ 与真机 SDK 手部位姿；两者坐标系和测量点不同，尚未标定，不能把绝对数值直接比较。IK 使用真机控制器 MoveP；程序记录实测关节角和到位误差，不声称调用了课程文件中的数值 IK 求解器。

启动姿态需与课程 `prepare` 的 14 个手臂关节相差不超过 0.05 rad。真机上的 `my_robot.json` 目前曾报告 `gripper.enabled=false`；要执行夹爪动作，先备份配置，核对实际 LimX 双指夹爪与状态查询，再将它设为 `true`。只读规划不会移动机器人；配置仍为 `false` 时，真机模式会在发送任何动作前拒绝运行。`--arm-only` 可单独跳过夹爪。

先运行只读规划，检查报告的阶段目标和现场完整路径：

```bash
cd ~/Course_Code
.venv-humble/bin/python ./sdk_examples_real_robot/sdk_examples/choreography_live.py \
  --robot ./sdk_examples_real_robot/sdk_examples/my_robot.json \
  --course-code . --output ./results/choreography_plan_001.json
```

在双腕摄像头、夹爪及完整手臂路径均完成现场检查，且有人守在急停旁后，使用新的报告名启动：

```bash
.venv-humble/bin/python ./sdk_examples_real_robot/sdk_examples/choreography_live.py \
  --robot ./sdk_examples_real_robot/sdk_examples/my_robot.json \
  --course-code . --send --site-checked --duration 5 --speed-factor 4 \
  --output ./results/choreography_live_001.json
```

要减少 MoveJ 段间停顿，先用新报告名做 `--flow` 只读规划，再在现场检查路径后运行：

```bash
.venv-humble/bin/python ./sdk_examples_real_robot/sdk_examples/choreography_live.py \
  --robot ./sdk_examples_real_robot/sdk_examples/my_robot.json \
  --course-code . --flow --output ./results/choreography_flow_plan_001.json

.venv-humble/bin/python ./sdk_examples_real_robot/sdk_examples/choreography_live.py \
  --robot ./sdk_examples_real_robot/sdk_examples/my_robot.json \
  --course-code . --send --site-checked --duration 5 --speed-factor 4 --flow \
  --output ./results/choreography_flow_live_001.json
```

`--flow` 保留相同的阶段端点、关节空间直线路径、FK1/FK2 和控制器 MoveP 的 IK1/IK2，并继续逐段检查反馈。它将理想 HOME 起点的 MoveJ 段数从约 70 降到约 25；单段最多约 0.27 rad，超过设备配置的 `max_segment_delta_rad=0.1`，仅此脚本在显式 `--flow` 时放宽该门限。手腕摄像头的课程模型代理检查会沿新长段以最多约 0.02 rad 的采样间隔执行，仍不能证明真实摄像头避碰。快速模式下每段 MoveJ 按实际角度在 1.25 至 3.75 秒之间缩放，段间到位检查次数减少；总用时不一定缩短。Ctrl+C 能阻止后续指令，但已接受的当前长段仍可能继续，最长名义运动时间由 1.25 秒增至 3.75 秒。建议先按 `--speed-factor 1 --flow` 验收长段路径，再使用四倍模式。`--flow` 不能消除所有停顿：MoveJ 段终点、夹爪事件、三个 MoveH 头部动作、两个 MoveP IK 点仍需等待反馈。

`--speed-factor 4` 只作用于此脚本，将原 5 秒段设为 1.25 秒；不使用该选项仍按旧速运行。头部另有三段 MoveH，整套实际总耗时受状态读取、夹爪和控制器执行影响。厂商 MoveJ 文档展示 2 秒示例，但建议调试时使用至少 5 秒；快速模式的动态避碰仍需真机验证。若控制器拒绝或反馈未到位，程序停止后续指令且不自动复位。

`--site-checked` 表示操作者已检查实体机器人完整路径、双腕摄像头以及新增头部转动区域；它不是软件碰撞证明。程序以课程模型腕中心半径 10 cm 的球形包络和 5 cm 净距阈值检查手臂路径，但实际摄像头、夹爪安装几何未知，模型坐标也未与真机 SDK 标定。默认模式的每段 MoveJ 不超过配置的关节变化上限；`--flow` 的变化见上文。

若要在关节路点之间真正连续过渡，需要另行集成厂商公开的 `tron2_env` ServoJ 发布器；该库不被当前脚本使用，也不会因安装而自动消除停顿。它的公开 WebSocket 传输未提供本脚本的控制器 MoveP 接口，因此保留两个 IK 点时至少需要在 ServoJ 与 MoveP 之间明确交接，且须验证 Ctrl+C、反馈和现场避碰。Ubuntu 可在现有虚拟环境安装基础包并确认导入，安装本身不会触发机器人运动：

```bash
.venv-humble/bin/python -m pip install tron2-env==0.1.0
.venv-humble/bin/python -c 'import tron2_env; print(tron2_env.__file__)'
```

**Ctrl+C 可在任意时刻终止脚本的后续指令**。报告状态变为 `interrupted`，`inflight_command` 记录可能已发出的当前指令；不会自动发送复位。已经被控制器接受的当前 MoveJ/MoveP 可能继续走完，Ctrl+C 不能代替遥控器或物理急停。厂商文档中的 `request_emgy_stop` 仅在空闲模式响应，程序没有把它当作运动中的急停。中断后先查询实际姿态，确认周围和双腕摄像头位置，再决定回程；不能直接重跑整套动作。

正常完成时，报告为 `four_points_and_reset_feedback_confirmed`，仅表示 SDK 反馈确认回到启动姿态附近。通信、反馈、限位或夹爪异常时停止后续命令，不在未知状态下自动复位。本机 mock 测试入口：

```bash
cd sdk_examples_real_robot/sdk_examples
python -m unittest -q test_choreography_live.py
```

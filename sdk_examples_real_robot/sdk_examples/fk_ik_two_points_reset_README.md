# 两个 FK 点、两个 IK 点与回到起始姿态

入口：`fk_ik_two_points_reset.py`。将此文件复制到 Ubuntu 的 `~/Course_Code/sdk_examples_real_robot/sdk_examples/`；沿用已核对的 `my_robot.json`、`client.py`、`lessons.py` 和 `~/Course_Code/students/kinematics.py`。Python 文件无需编译。

动作由程序启动时的真机反馈决定。默认 `--pattern simple` 保留已在实体机器人上完成并复位的轨迹：

1. FK1：左右肘同步伸展两段，每段各 0.08 rad，累计各 0.16 rad；记录第一组 14 维反馈、SDK 末端位姿及课程模型 FK。
2. FK2：左右肘继续同步伸展两段，累计各 0.32 rad（约 18.3°）；记录第二组反馈和 FK。
3. IK1、IK2：左右手 SDK 末端 x 方向分别相对移动 +5 mm、-5 mm，由真机 MoveP 控制器执行逆运动学。每个点最多两次反馈修正。
4. 返回：从 IK2 实际关节角出发，依次经过 FK2、FK1 的实测关节角，最后返回程序启动时的实测关节角；MoveJ 返回段每关节不超过 0.07 rad。末尾复查关节角和双臂 SDK 位姿。

可选 `--pattern varied` 增加姿态变化，仍只有两个 FK 记录点与两个 IK 目标点：

| 阶段 | 左肘相对启动角 | 右肘相对启动角 | 双手 SDK 位移 |
|---|---:|---:|---|
| FK1 | +0.16 rad | +0.16 rad | 记录课程 FK 与 SDK 实测位姿 |
| FK2 | +0.32 rad | 0 rad | 记录课程 FK 与 SDK 实测位姿 |
| IK1 | 保持 FK2 后由控制器求解 | 保持 FK2 后由控制器求解 | 左手 x +8、y +4 mm；右手 x +8、y −4 mm |
| IK2 | 由控制器求解 | 由控制器求解 | 相对 IK1，左手 x +8、y −4 mm；右手 x +8、y +4 mm |

FK1 前两段双肘同向；FK1 到 FK2 的两段左肘继续伸展、右肘回收，每段关节变化均为 0.08 rad。每条 IK 斜向命令的位移约 8.9 mm。IK2 后同样按实测检查点分段返回程序启动姿态。新模式仅在本机 mock 验证，实体机器人上的可达性和避碰仍需现场核对。

`--pattern wide` 用于更大幅度、较快节奏的四点演示：

| 阶段 | 左肘相对启动角 | 右肘相对启动角 | 说明 |
|---|---:|---:|---|
| FK1（第 3 段） | +0.27 rad | +0.27 rad | 双臂同步伸展，记录第一个 FK 点 |
| FK2（第 5 段） | +0.45 rad | +0.09 rad | 左肘继续伸展、右肘回收，记录第二个 FK 点 |
| IK1、IK2 | 控制器求解 | 控制器求解 | 沿用 `varied` 的两个双手斜向目标 |

前进每段肘关节目标变化不超过 0.09 rad，返回每段目标变化不超过 0.09 rad；程序仍逐段核对关节反馈，最终核对起始关节角和 SDK 位姿。与现场成功的 `varied --duration 8` 相比，`wide --duration 5` 的左肘最大角度增加 0.13 rad，右肘峰值增加 0.11 rad。用户已在实体机器人上运行 `wide` 并报告完整动作与复位，终端状态为 `four_points_and_reset_feedback_confirmed`、15 步。

`--pattern sway` 是肩部左右摆动的可选模式，保持两个 FK 点、两个 IK 点：

| 阶段 | 双肘相对启动角（左 / 右） | 双肩 yaw 相对启动角 | 双手 SDK 目标 |
|---|---|---:|---|
| FK1（第 2 段） | +0.18 / +0.18 rad | +0.12 rad | 记录课程 FK 与实际 SDK 位姿 |
| FK2（第 5 段） | +0.45 / +0.09 rad | −0.12 rad | 记录课程 FK 与实际 SDK 位姿 |
| IK1 | 控制器求解 | 控制器求解 | 左右手均 x +4、y +8 mm |
| IK2 | 控制器求解 | 控制器求解 | 相对 IK1，左右手均 x +4、y −8 mm |

FK1 前逐段增加肩 yaw，FK1 到 FK2 逐段转向另一侧。程序在报告的 `fk_point_lateral_displacement_sdk_m` 中记录两手实际 y 位移；正负方向与幅度以现场 SDK 反馈为准。IK2 后从实测当前姿态直接分段返回启动关节角，省去重复经过 FK1 的回程，关节目标每段不超过 0.09 rad。**不能把模型关节角限位当作避碰验证；`sway` 尚未在实体机器人上运行。**

可选 `--with-gripper` 让已安装的 LimX 二指夹爪在 FK1 后闭合到 30、FK2 后张开到 80，手臂复位后恢复程序开始时测得的开口度。左右夹爪依次发命令并逐个核对状态。只有真机 `my_robot.json` 的 `gripper.type` 为 `limx_2fclaw`、`gripper.enabled` 为 `true`，且运行前能够读取左右夹爪状态，程序才会下发任何动作。确认两夹爪为空、周围没有会被夹住的物品后再使用。未带该选项时不触碰夹爪。

课程 FK 使用 `base_Link` 坐标，尚未标定到 SDK 基座坐标，因此两套绝对位姿不能直接做误差评价。IK 两点使用控制器 MoveP，**没有**把课程 `solve_ik_position` 的结果下发真机。程序不具备避碰能力；关节限位内也可能与现场物体干涉。

程序仅从官方 `prepare` 姿态附近（14 关节各相差不超过 0.05 rad）启动。若之前的运行在伸展姿态被中断，先用课程 `run.py prepare` 回到该姿态，并只读查询确认；程序不会把当前伸展姿态当作新的起点继续伸展。

先只读规划（在 `~/Course_Code` 下）：

```bash
.venv-humble/bin/python ./sdk_examples_real_robot/sdk_examples/fk_ik_two_points_reset.py \
  --robot ./sdk_examples_real_robot/sdk_examples/my_robot.json \
  --course-code . --pattern varied \
  --output ./results/fk_ik_varied_plan_$(date +%s).json
```

预期 `status=read_only_plan`，没有运动。核对 `initial`、`fk_targets_q14_rad` 和工作空间。现场运行须使用**新的报告文件名**：

```bash
.venv-humble/bin/python ./sdk_examples_real_robot/sdk_examples/fk_ik_two_points_reset.py \
  --robot ./sdk_examples_real_robot/sdk_examples/my_robot.json \
  --course-code . --pattern varied --send --duration 8 \
  --output ./results/fk_ik_varied_live_$(date +%s).json
```

`--send` 是一次性启动授权；程序启动后不再等待键盘输入，逐段发送、读取反馈，正常完成后自动分段返回。每段 MoveJ/MoveP 仍打印目标并执行客户端限位和反馈检查。正常结束且反馈检查通过时为 `four_points_and_reset_feedback_confirmed`。这表示 SDK 反馈恢复到程序启动姿态附近，不替代实体机器人目视验收。

若用户中断、通信失败、工作空间检查或反馈检查失败，程序会保存已完成步骤并停止，**不会在状态未知时自动发送复位动作**。此时先现场读取实际姿态，再决定恢复路径。重新运行会从新的当前姿态生成新的四点，不能当作续跑。

更大幅度模式的只读规划和真机命令（在 `~/Course_Code` 下，先将更新后的脚本复制到 Ubuntu 同名路径）：

```bash
.venv-humble/bin/python ./sdk_examples_real_robot/sdk_examples/fk_ik_two_points_reset.py \
  --robot ./sdk_examples_real_robot/sdk_examples/my_robot.json \
  --course-code . --pattern wide \
  --output ./results/fk_ik_wide_plan_$(date +%s).json
```

```bash
.venv-humble/bin/python ./sdk_examples_real_robot/sdk_examples/fk_ik_two_points_reset.py \
  --robot ./sdk_examples_real_robot/sdk_examples/my_robot.json \
  --course-code . --pattern wide --send --duration 5 \
  --output ./results/fk_ik_wide_live_$(date +%s).json
```

现场先核对机器人和双臂周围的完整运动空间。设备配置 `my_robot.json` 的 `min_segment_s` 为 5 秒；程序不会发送比该值更短的命令。厂商 SDK 指南也建议调试时设置不少于 5 秒的运动时间。

肩部摆动模式的只读规划与真机命令：

```bash
.venv-humble/bin/python ./sdk_examples_real_robot/sdk_examples/fk_ik_two_points_reset.py \
  --robot ./sdk_examples_real_robot/sdk_examples/my_robot.json \
  --course-code . --pattern sway \
  --output ./results/fk_ik_sway_plan_$(date +%s).json
```

```bash
.venv-humble/bin/python ./sdk_examples_real_robot/sdk_examples/fk_ik_two_points_reset.py \
  --robot ./sdk_examples_real_robot/sdk_examples/my_robot.json \
  --course-code . --pattern sway --send --duration 5 \
  --output ./results/fk_ik_sway_live_$(date +%s).json
```

要加入夹爪，先备份 Ubuntu 上的 `my_robot.json`，并在核对夹爪型号后将其中 `gripper.enabled` 改为 `true`；上述两条命令都再加上 `--with-gripper`。只读规划会查询夹爪开口度，不会发运动命令。若配置仍为 `false` 或夹爪状态查询失败，程序会在手臂运动前停止。`--with-gripper` 的开合动作可能增加总用时。

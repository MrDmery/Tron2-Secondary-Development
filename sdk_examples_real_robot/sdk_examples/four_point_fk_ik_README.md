# TRON2 四点运动学真机练习

`four_point_fk_ik.py` 默认左臂，基于启动时的实测状态选择四个小幅相对点：

- FK1、FK2：左肘每次移动 0.02 rad，分别读取关节反馈，计算课程模型的正运动学，并记录 SDK 的末端反馈。
- IK1、IK2：在 SDK 坐标系中沿 x 方向先移动 5 mm，再移动 -5 mm。调用控制器的 MoveP，由控制器执行逆运动学；每点最多两次闭环修正。

这里的 FK 来自 `Course_Code/students/kinematics.py`，其 `base_Link` 坐标未与真机 SDK 坐标标定。报告同时保存两套结果，但不直接相减评价绝对精度。本程序的 IK 两点是控制器 MoveP 的 IK，不是将课程的 `solve_ik_position` 解下发真机。关节限位也不能代替避碰检查。

## Ubuntu 文件准备

将整个 `sdk_examples_real_robot/sdk_examples` 文件夹复制到 Ubuntu，例如 `~/sdk_examples_real_robot/sdk_examples`，并保留已有的 `~/Course_Code`。在复制后的目录执行：

```bash
cd ~/sdk_examples_real_robot/sdk_examples
cp robot_profile.example.json my_robot.json
```

根据机器人管理页和只读查询填写 `my_robot.json`。不要直接使用随包的 `device_A.json`，它记录的是另一台设备。真机模式、关节顺序及安全操作步骤见 `真机操作步骤.md`。

## 先只读规划

使用已安装课程数学与 WebSocket 依赖的 Python 环境：

```bash
~/Course_Code/.venv-humble/bin/python four_point_fk_ik.py --robot my_robot.json --course-code ~/Course_Code --output four_point_plan.json
```

预期 `status=read_only_plan`，没有运动命令。程序会在关节运动前检查双臂当前末端及首个 MoveP 目标是否在 SDK 工作空间内；垂臂等超界姿态会返回 `stopped`。核对 `initial_q14_rad`、`initial_sdk_pose_xyz_wxyz` 和 `fk_joint_targets_q14_rad`；当前姿态和动作空间仍需现场判断。

## 现场逐点运行

```bash
~/Course_Code/.venv-humble/bin/python four_point_fk_ik.py --robot my_robot.json --course-code ~/Course_Code --send --duration 5 --output four_point_live_001.json
```

每条运动都打印目标并等待操作员输入 `y`。MoveP 修正时也会再次询问。任何未到位、反馈漂移、越界或用户拒绝都会停止后续步骤，并保存已完成步骤。只有报告出现 `completed_four_points` 才表示本次四点流程执行完毕；仍需查看各步反馈，不能把 mock 测试当成真机验收。

每次现场运行使用新的 `--output` 文件名。程序拒绝覆盖已有现场报告；重新运行会从当前反馈再次生成新的相对点，不会续跑上次未完成的 IK 步骤。

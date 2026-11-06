# 双臂可见伸展与返回

`visible_dual_arm_motion.py` 在当前实测姿态基础上逐步伸展左肘和右肘，再按相反顺序回到初始关节角。默认每段仅改变一个肘关节 `0.08 rad`；可选 `--simultaneous` 让两肘在同一段各改变 `0.08 rad`。默认每臂累计 `0.32 rad`（约 18.3°），每段 `8 s`。这不是“水平平展”的姿态求解器，实际空间轨迹需现场观察；程序没有碰撞检测。

从 Ubuntu `~/Course_Code` 运行。先将本脚本从 Windows 工作区复制到 `~/Course_Code/sdk_examples_real_robot/sdk_examples/`，并使用已核对的 `my_robot.json`。

只读计划：

```bash
.venv-humble/bin/python ./sdk_examples_real_robot/sdk_examples/visible_dual_arm_motion.py \
  --robot ./sdk_examples_real_robot/sdk_examples/my_robot.json \
  --output ./results/dual_arm_plan_001.json
```

看清 14 维路点和现场空间后，再由操作员决定是否运行。若希望先做较小的可见试验，可加 `--amplitude 0.08`，正常结束时也会返回起始关节角。现场运行示例：

```bash
.venv-humble/bin/python ./sdk_examples_real_robot/sdk_examples/visible_dual_arm_motion.py \
  --robot ./sdk_examples_real_robot/sdk_examples/my_robot.json \
  --amplitude 0.08 --send --output ./results/dual_arm_live_001.json
```

小幅试验核实实体机械臂确实运动后，可以另用 `--amplitude 0.32` 做完整伸展与返回，并使用新的输出文件名。每段都会打印目标、等待输入 `y`，前半程结束时还会暂停供现场观察。若通信、反馈或到位检查失败，程序停止，不自动发送回退动作；此时应先查看机器人实际姿态和报告，再决定恢复路径。`feedback_confirmed_returned` 只表示 SDK 反馈达到计划初始关节角，实体动作仍以现场观察为准。

双臂同步可选模式：先运行 `--simultaneous --amplitude 0.08` 的只读计划，再用同样参数加 `--send` 和新的输出文件名现场试验。同步模式在同一条 MoveJ 命令中将左右肘各增加 0.08 rad，之后再同步返回；程序逐段检查双肘反馈。0.32 rad 幅度有 4 个伸展段和 4 个返回段。此模式在原有单关节模式现场可见运动后新增，尚未在真机验证。

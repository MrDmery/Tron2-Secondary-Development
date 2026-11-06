# TRON2 动作候选（离线）

从 `Course_Code` 目录运行：

```bash
python choreography_candidate.py --output-dir results/my_choreography_01
python choreography_mujoco_replay.py --fps 8
python -m unittest -q test_choreography_candidate.py
```

第一条命令生成 `candidate.json`、静态预览 PNG 和动态 GIF。第二条命令在本地 MuJoCo 窗口回放同一组关节姿态，不会连接机器人；窗口关闭后退出。`--output-dir` 必须使用新目录，避免覆盖先前报告。

动作顺序：左臂抬起（FK1，右臂保持）、左臂上下摆两次、双臂胸前造型（FK2）、回到起点、左右臂交替前后摆并在各摆位计算一次双臂位置 IK（IK1、IK2）、最终回到起点。JSON 中的夹爪开合只是时序提示，MuJoCo 课程模型没有实体双指夹爪。

双腕摄像头没有包含在课程 MJCF 中。候选生成器暂用以腕关节为球心、半径 10 cm 的摄像头包络，并在两摄像头、固定 D435 和对侧前臂之间检查至少 5 cm 净距。包络尺寸与安装偏移未经真机测量，检查结果不能证明真机避碰。关节模型坐标也没有与真机 SDK 末端坐标标定。本候选不支持 `--send`；需取得实际摄像头和夹爪几何并完成三维碰撞、路径与真机低速验收后，才能制作自动真机动作程序。

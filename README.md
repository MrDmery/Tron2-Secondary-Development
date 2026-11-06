# TRON2 二次开发练习

本仓库整理双臂正逆运动学练习、离线动作候选，以及通过 TRON2 高层 WebSocket 协议执行的真机示例。`sdk_examples_real_robot/sdk_examples/` 是课程适配客户端，不是厂商完整 SDK。

## 内容

| 路径 | 用途 |
|---|---|
| `Course_Code/students/` | PD、正逆运动学与轨迹练习实现 |
| `Course_Code/choreography_candidate.py` | 生成 FK1/FK2、IK1/IK2 的离线动作候选 |
| `Course_Code/choreography_mujoco_replay.py` | 在 MuJoCo 窗口回放候选关节姿态 |
| `sdk_examples_real_robot/sdk_examples/` | WebSocket 客户端、四点演示、双臂动作和 mock 测试 |

## 获取模型与安装

课程 MJCF、网格、课件和仿真安装包未随公开源码发布。经相应权利人许可取得模型后，将 `DA_TRON2A_fixed_no_tool.xml` 放在 `Course_Code/models/`，将模型所引用的网格放在 `Course_Code/assets/DA_TRON2A/meshes/`。离线数值动作候选只读取 XML；MuJoCo 三维回放还需网格。请以实际模型资源的许可和来源说明为准。

在 Ubuntu 22.04 / Python 3.10 环境中安装所需 Python 包（课程环境可能已安装）：

```bash
python3 -m pip install numpy matplotlib mujoco websocket-client websockets
```

离线候选与回放：

```bash
cd Course_Code
python3 choreography_candidate.py --output-dir results/choreography_01
python3 choreography_mujoco_replay.py --fps 8
```

真机四点及完整动作的命令、启动姿态、夹爪和中断条件分别见 `sdk_examples_real_robot/sdk_examples/fk_ik_two_points_reset_README.md` 与 `choreography_live_README.md`。复制 `robot_profile.example.json` 为本地 `my_robot.json`，逐项填写并核对现场设备信息；真实配置已被 Git 忽略。`--send` 会给实体机器人发运动命令。

## 验证

在具备课程模型的环境中：

```bash
cd sdk_examples_real_robot/sdk_examples
python3 -m unittest -q test_contract.py test_fk_ik_two_points_reset.py test_choreography_live.py
```

`test_contract.py` 使用本机 WebSocket mock；通过测试不代表完成真机或动力学验证。课程模型坐标、摄像头代理与真机安装几何尚未标定。

## 发布范围

此仓库仅包含整理后的源码、测试及设备配置模板。原始压缩包、PDF 课件、设备 SN 与配置、运行日志、模型和网格资源均未上传。仓库尚未指定整体开源许可证，第三方材料仍按各自许可使用。

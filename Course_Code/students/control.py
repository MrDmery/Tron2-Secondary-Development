# """Exercise 1. Fill the PD law, then python run_lab.py pd --student."""
# import numpy as np


# def pd_torque(q, dq, q_ref, dq_ref, kp, kd, tau_ff, tau_max):
#     # TODO: position error, velocity error, feedforward, torque clipping.
#     # q and dq are 14D; gains and tau_ff also 14D in this experiment.
#     raise NotImplementedError('TODO: implement PD torque and saturation in students/control.py')

import numpy as np

def pd_torque(q, dq, q_ref, dq_ref, kp, kd, tau_ff, tau_max):
    """
    计算 PD + 前馈力矩，并进行饱和限制。

    参数:
        q (np.ndarray): 当前关节位置 (14D)
        dq (np.ndarray): 当前关节速度 (14D)
        q_ref (np.ndarray): 目标关节位置 (14D)
        dq_ref (np.ndarray): 目标关节速度 (14D)
        kp (np.ndarray): 比例增益 (14D)
        kd (np.ndarray): 微分增益 (14D)
        tau_ff (np.ndarray): 前馈力矩(如重力补偿) (14D)
        tau_max (np.ndarray or float): 最大力矩限制

    返回:
        np.ndarray: 最终计算的关节力矩
    """
    # 1. 计算位置误差和速度误差
    # 根据图片，误差 = q_ref - q
    pos_err = q_ref - q
    vel_err = dq_ref - dq

    # 2. 计算 PD 力矩 + 前馈补偿 (如重力补偿)
    tau = kp * pos_err + kd * vel_err + tau_ff

    # 3. 力矩限幅 (Torque clipping / saturation)
    # np.clip 可以逐元素地对矩阵进行限制
    tau = np.clip(tau, -tau_max, tau_max)

    return tau

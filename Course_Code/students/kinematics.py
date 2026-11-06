"""Exercises 2-3. Complete the functions below during class."""
import numpy as np
from model_info import (arm_chain, axis_rotation, transform,
    joint_limits, q_home, IKResult)


def fk(q, side='left'):
    q = np.asarray(q, dtype=float)
    if q.shape != (7,) or not np.all(np.isfinite(q)):
        raise ValueError('q must contain seven finite joint angles')
    T = np.eye(4)
    for angle, joint in zip(q, arm_chain(side)):
        anchor = joint['anchor']
        T = (T @ joint['fixed'] @ transform(p=anchor)
             @ transform(R=axis_rotation(joint['axis'], angle-joint['ref']))
             @ transform(p=-anchor))
    return T


def jacobian_position(q, side='left'):
    q = np.asarray(q, dtype=float)
    if q.shape != (7,) or not np.all(np.isfinite(q)):
        raise ValueError('q must contain seven finite joint angles')
    T = np.eye(4)
    origins, axes = [], []
    for angle, joint in zip(q, arm_chain(side)):
        T = T @ joint['fixed']
        origins.append((T @ np.r_[joint['anchor'], 1.])[:3])
        axes.append(T[:3, :3] @ (joint['axis']/np.linalg.norm(joint['axis'])))
        T = (T @ transform(p=joint['anchor'])
             @ transform(R=axis_rotation(joint['axis'], angle-joint['ref']))
             @ transform(p=-joint['anchor']))
    p = T[:3, 3]
    return np.column_stack([np.cross(axis, p-origin)
                            for axis, origin in zip(axes, origins)])


def solve_ik_position(target, q0, side='left', damping=.01, alpha=1.,
                      max_iterations=200, tolerance=1e-5):
    target = np.asarray(target, dtype=float)
    q = np.asarray(q0, dtype=float).copy()
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError('target must contain three finite coordinates')
    if q.shape != (7,) or not np.all(np.isfinite(q)):
        raise ValueError('q0 must contain seven finite joint angles')
    if damping <= 0 or alpha <= 0 or max_iterations < 0 or tolerance <= 0:
        raise ValueError('invalid IK iteration parameters')
    limits = joint_limits(side)
    q = np.clip(q, limits[:, 0], limits[:, 1])
    errors, path = [], [q.copy()]
    for _ in range(max_iterations+1):
        error = target-fk(q, side)[:3, 3]
        distance = float(np.linalg.norm(error))
        errors.append(distance)
        if distance <= tolerance:
            return IKResult(q, True, errors, len(path)-1, path)
        if len(path)-1 == max_iterations:
            break
        J = jacobian_position(q, side)
        step = J.T @ np.linalg.solve(J @ J.T+damping**2*np.eye(3), error)
        q = np.clip(q+alpha*step, limits[:, 0], limits[:, 1])
        path.append(q.copy())
    return IKResult(q, False, errors, len(path)-1, path)


def fk_2r(q, lengths=(.3, .25)):
    q = np.asarray(q, dtype=float)
    lengths = np.asarray(lengths, dtype=float)
    if q.shape != (2,) or lengths.shape != (2,) or not np.all(np.isfinite(q)) or not np.all(np.isfinite(lengths)) or np.any(lengths <= 0):
        raise ValueError('2R needs two finite angles and two positive lengths')
    return np.array([lengths[0]*np.cos(q[0])+lengths[1]*np.cos(q.sum()),
                     lengths[0]*np.sin(q[0])+lengths[1]*np.sin(q.sum())])


def ik_2r(target, lengths=(.3, .25), elbow=1):
    target = np.asarray(target, dtype=float)
    lengths = np.asarray(lengths, dtype=float)
    if target.shape != (2,) or lengths.shape != (2,) or not np.all(np.isfinite(target)) or not np.all(np.isfinite(lengths)) or np.any(lengths <= 0) or elbow not in (-1, 1):
        raise ValueError('invalid 2R target, lengths or elbow branch')
    x, y = target
    l1, l2 = lengths
    cosine = (x*x+y*y-l1*l1-l2*l2)/(2*l1*l2)
    if abs(cosine) > 1+1e-12:
        raise ValueError('2R target is unreachable')
    q2 = elbow*np.arccos(np.clip(cosine, -1., 1.))
    q1 = np.arctan2(y, x)-np.arctan2(l2*np.sin(q2), l1+l2*np.cos(q2))
    return np.array([q1, q2])

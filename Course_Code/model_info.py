"""Model parameters and primitive transforms for the course exercises.

The articulated FK, Jacobian, IK and PD implementations belong in students/.
Frames and hinge parameters are read from the supplied public MJCF model.
"""
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

MODEL = Path(__file__).resolve().parent / 'models/DA_TRON2A_fixed_no_tool.xml'


def transform(R=None, p=None):
    """Pack an already-known rotation and translation into a 4x4 matrix."""
    value = np.eye(4)
    if R is not None: value[:3, :3] = R
    if p is not None: value[:3, 3] = p
    return value


def axis_rotation(axis, angle):
    """MuJoCo primitive rotation; does not traverse the articulated chain."""
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if axis.shape != (3,) or not np.isfinite(norm) or norm == 0:
        raise ValueError('axis must contain three finite values with nonzero norm')
    quat, matrix = np.empty(4), np.empty(9)
    mujoco.mju_axisAngle2Quat(quat, axis / norm, float(angle))
    mujoco.mju_quat2Mat(matrix, quat)
    return matrix.reshape(3, 3)


def _numbers(text, fallback):
    return np.asarray(fallback if text is None else list(map(float, text.split())), dtype=float)


@lru_cache(maxsize=2)
def arm_chain(side='left'):
    """Return each hinge's fixed transform, axis, anchor, limits and ref."""
    if side not in ('left', 'right'):
        raise ValueError('side must be left or right')
    suffix = 'L' if side == 'left' else 'R'
    root = ET.parse(MODEL).getroot()
    base = root.find(".//body[@name='base_Link']")
    node = base.find(f"body[@name='proximal_pitch_{suffix}_Link']")
    chain = []
    while node is not None:
        joint = node.find('joint')
        if joint is None: break
        if joint.get('type', 'hinge') != 'hinge':
            raise ValueError('Expected hinge joints')
        quat = _numbers(node.get('quat'), [1, 0, 0, 0])
        matrix = np.empty(9)
        mujoco.mju_quat2Mat(matrix, quat / np.linalg.norm(quat))
        chain.append(dict(name=joint.get('name'),
            fixed=transform(matrix.reshape(3, 3), _numbers(node.get('pos'), [0, 0, 0])),
            axis=_numbers(joint.get('axis'), [0, 0, 1]),
            anchor=_numbers(joint.get('pos'), [0, 0, 0]),
            limits=_numbers(joint.get('range'), [-np.pi, np.pi]),
            ref=float(joint.get('ref', '0'))))
        node = node.find('body')
    if len(chain) != 7:
        raise ValueError(f'Expected 7 hinges, got {len(chain)}')
    return chain


def joint_limits(side='left'):
    return np.asarray([joint['limits'] for joint in arm_chain(side)])


def q_home(side='left'):
    """Initial pose for the unloaded course simulator; not a hardware target."""
    if side not in ('left', 'right'): raise ValueError('side must be left or right')
    return np.array([-.15, .25 if side == 'left' else -.25, 0, -.8, 0, 0, 0])


@dataclass
class IKResult:
    q: np.ndarray
    converged: bool
    errors: list
    iterations: int
    path: list

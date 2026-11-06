"""Fixed-position pick-and-place project scaffold.

q contains 14 arm joints in left-then-right order; times are in seconds.
Gripper commands are independent events. This model has no physical gripper.
"""
import numpy as np
from model_info import q_home, joint_limits


def build_taught_task():
    """Return {'times': [...], 'q': [[14 values], ...], 'events': [...]}.

    Each event contains time, name and opening (0..100 percent).
    Use recorded simulator poses and a stationary target for this exercise.
    """
    # Course-model example only. Replace these with recorded simulator waypoints
    # for an actual fixed-target task; these angles are not hardware targets.
    home = np.r_[q_home('left'), q_home('right')]
    approach = home.copy(); approach[1] += .06; approach[3] += .04
    lower = approach.copy(); lower[3] += .04
    carry = home.copy(); carry[0] += .08; carry[1] += .06
    place = carry.copy(); place[3] += .04
    return {'times': [0., 2., 4., 5., 7., 9., 11., 13., 15.],
            'q': [home.tolist(), approach.tolist(), lower.tolist(),
                  lower.tolist(), approach.tolist(), carry.tolist(),
                  place.tolist(), place.tolist(), home.tolist()],
            'events': [{'time': 1., 'name': 'open', 'opening': 100.},
                       {'time': 4.5, 'name': 'close', 'opening': 0.},
                       {'time': 11.5, 'name': 'release', 'opening': 100.}]}


def validate_task(task):
    """Check finite values, 14-joint vectors, increasing times and event bounds."""
    if not isinstance(task, dict) or not all(key in task for key in ('times', 'q', 'events')):
        raise ValueError('task needs times, q and events')
    try:
        times = np.asarray(task['times'], dtype=float)
        q = np.asarray(task['q'], dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError('times and q must be numeric arrays') from exc
    if times.ndim != 1 or len(times) < 2 or not np.all(np.isfinite(times)) or times[0] != 0 or np.any(np.diff(times) <= 0):
        raise ValueError('times must start at zero and increase strictly')
    if q.shape != (len(times), 14) or not np.all(np.isfinite(q)):
        raise ValueError('q must have one finite 14-joint vector per time')
    limits = np.vstack((joint_limits('left'), joint_limits('right')))
    if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
        raise ValueError('waypoint exceeds course-model joint limits')
    if not isinstance(task['events'], list):
        raise ValueError('events must be a list')
    for event in task['events']:
        if not isinstance(event, dict) or not all(key in event for key in ('time', 'name', 'opening')):
            raise ValueError('each event needs time, name and opening')
        try:
            time, opening = float(event['time']), float(event['opening'])
        except (TypeError, ValueError) as exc:
            raise ValueError('event time and opening must be numeric') from exc
        if not np.isfinite(time) or not 0 <= time <= times[-1] or not np.isfinite(opening) or not 0 <= opening <= 100 or not isinstance(event['name'], str) or not event['name']:
            raise ValueError('invalid gripper event')
    return True


def sample_task(task, time_s):
    """Return q_ref and dq_ref (14 values each) at time_s."""
    time_s = float(time_s)
    if not np.isfinite(time_s):
        raise ValueError('time_s must be finite')
    times = np.asarray(task['times'], dtype=float)
    q = np.asarray(task['q'], dtype=float)
    if time_s <= times[0]:
        return q[0].copy(), np.zeros(14)
    if time_s >= times[-1]:
        return q[-1].copy(), np.zeros(14)
    index = int(np.searchsorted(times, time_s, side='right'))-1
    duration = times[index+1]-times[index]
    velocity = (q[index+1]-q[index])/duration
    return q[index]+(time_s-times[index])*velocity, velocity

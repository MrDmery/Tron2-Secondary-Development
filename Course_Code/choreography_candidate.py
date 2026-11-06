"""Offline FK/IK choreography preview for TRON2; never connects to hardware.

The supplied MJCF has no geometry for the two wrist cameras. Camera proxies
are deliberately conservative estimates, not a real collision certificate.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


MODEL = Path(__file__).resolve().parent / "models" / "DA_TRON2A_fixed_no_tool.xml"
HOME = np.array([0., .25, 0., -1.57, .236, 0., 0.,
                 0., -.25, 0., -1.57, -.236, 0., 0.])
JOINT_STEP_RAD = .08
WRIST_CAMERA_PROXY_RADIUS_M = .10  # Assumed camera volume around each wrist.
FIXED_CAMERA_PROXY_RADIUS_M = .08
REQUIRED_CLEARANCE_M = .05
FOREARM_PROXY_RADIUS_M = .03


def _numbers(value, default):
    return np.fromstring(value, sep=" ") if value else np.array(default, dtype=float)


def _rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    cross = np.array([[0., -axis[2], axis[1]],
                      [axis[2], 0., -axis[0]],
                      [-axis[1], axis[0], 0.]])
    return np.eye(3)+math.sin(angle)*cross+(1.-math.cos(angle))*(cross@cross)


def _quaternion_rotation(wxyz):
    w, x, y, z = wxyz/np.linalg.norm(wxyz)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def load_chains(model_path=MODEL):
    root = ET.parse(model_path).getroot()
    base = root.find(".//body[@name='base_Link']")
    chains = {}
    for side, suffix in (("left", "L"), ("right", "R")):
        node = base.find(f"body[@name='proximal_pitch_{suffix}_Link']")
        chain = []
        while node is not None:
            joint = node.find("joint")
            if joint is None:
                break
            chain.append({"pos": _numbers(node.get("pos"), [0., 0., 0.]),
                          "quat": _numbers(node.get("quat"), [1., 0., 0., 0.]),
                          "axis": _numbers(joint.get("axis"), [0., 0., 1.]),
                          "anchor": _numbers(joint.get("pos"), [0., 0., 0.]),
                          "limits": _numbers(joint.get("range"), [-math.pi, math.pi])})
            node = node.find("body")
        if len(chain) != 7:
            raise ValueError(f"{side} arm must have seven MJCF joints")
        chains[side] = chain
    camera = base.find("body[@name='d435_Link']")
    chains["fixed_camera_center"] = _numbers(camera.get("pos"), [0., 0., 0.])
    return chains


def arm_points(chain, q7):
    transform = np.eye(4)
    points = []
    for joint, angle in zip(chain, q7):
        fixed = np.eye(4)
        fixed[:3, :3] = _quaternion_rotation(joint["quat"])
        fixed[:3, 3] = joint["pos"]
        transform = transform @ fixed
        before = np.eye(4)
        before[:3, 3] = joint["anchor"]
        after = np.eye(4)
        after[:3, 3] = -joint["anchor"]
        rotated = np.eye(4)
        rotated[:3, :3] = _rotation(joint["axis"], float(angle))
        transform = transform @ before @ rotated @ after
        points.append(transform[:3, 3].copy())
    return np.array(points)


def wrist_xyz(chains, q14, side):
    offset = 0 if side == "left" else 7
    return arm_points(chains[side], q14[offset:offset+7])[-1]


def position_ik(chains, q7, side, target, tolerance=1e-4):
    """Course-model position IK for a small preview offset, never a hardware command."""
    q = np.array(q7, dtype=float)
    limits = np.array([joint["limits"] for joint in chains[side]])
    for iteration in range(80):
        current = arm_points(chains[side], q)[-1]
        error = target-current
        if np.linalg.norm(error) <= tolerance:
            return q, float(np.linalg.norm(error)), iteration
        jacobian = np.empty((3, 7))
        for index in range(7):
            shifted = q.copy()
            shifted[index] += 1e-5
            jacobian[:, index] = (arm_points(chains[side], shifted)[-1]-current)/1e-5
        step = jacobian.T @ np.linalg.solve(jacobian@jacobian.T+1e-4*np.eye(3), error)
        q = np.clip(q+np.clip(step, -.08, .08), limits[:, 0], limits[:, 1])
    raise ValueError(f"course-model {side} IK did not reach target")


def _point_segment_distance(point, start, end):
    direction = end-start
    fraction = np.clip(np.dot(point-start, direction)/max(np.dot(direction, direction), 1e-12), 0., 1.)
    return float(np.linalg.norm(point-(start+fraction*direction)))


def camera_clearances(chains, q14, camera_radius=WRIST_CAMERA_PROXY_RADIUS_M):
    left = arm_points(chains["left"], q14[:7])
    right = arm_points(chains["right"], q14[7:])
    fixed = chains["fixed_camera_center"]
    values = {
        "between_wrist_camera_proxies_m": float(np.linalg.norm(left[-1]-right[-1])-2*camera_radius),
        "left_to_fixed_camera_proxy_m": float(np.linalg.norm(left[-1]-fixed)-camera_radius-FIXED_CAMERA_PROXY_RADIUS_M),
        "right_to_fixed_camera_proxy_m": float(np.linalg.norm(right[-1]-fixed)-camera_radius-FIXED_CAMERA_PROXY_RADIUS_M),
        "left_to_right_forearm_proxy_m": _point_segment_distance(left[-1], right[3], right[-1])-camera_radius-FOREARM_PROXY_RADIUS_M,
        "right_to_left_forearm_proxy_m": _point_segment_distance(right[-1], left[3], left[-1])-camera_radius-FOREARM_PROXY_RADIUS_M,
    }
    return values


def build_candidate(chains, camera_radius=WRIST_CAMERA_PROXY_RADIUS_M):
    q = HOME.copy()
    frames, milestones, gripper_events = [], {}, []

    def pose(name, target, milestone=None):
        nonlocal q
        target = np.asarray(target, dtype=float)
        if target.shape != (14,) or not np.all(np.isfinite(target)):
            raise ValueError("every pose needs fourteen finite joint angles")
        for side, offset in (("left", 0), ("right", 7)):
            for value, joint in zip(target[offset:offset+7], chains[side]):
                if not joint["limits"][0] <= value <= joint["limits"][1]:
                    raise ValueError(f"{name}: {side} target exceeds MJCF limits")
        count = max(1, math.ceil(float(np.max(np.abs(target-q)))/JOINT_STEP_RAD))
        start = q.copy()
        for number in range(1, count+1):
            current = start+(target-start)*number/count
            clearance = camera_clearances(chains, current, camera_radius)
            frames.append({"phase": name, "q14_rad": current.tolist(),
                           "left_joint_xyz_m": arm_points(chains["left"], current[:7]).tolist(),
                           "right_joint_xyz_m": arm_points(chains["right"], current[7:]).tolist(),
                           "camera_proxy_clearance_m": clearance})
        q = target.copy()
        if milestone:
            milestones[milestone] = {"q14_rad": q.tolist(),
                                      "left_wrist_model_xyz_m": wrist_xyz(chains, q, "left").tolist(),
                                      "right_wrist_model_xyz_m": wrist_xyz(chains, q, "right").tolist()}

    pose("start", HOME)
    high = HOME.copy()
    high[0] = -1.2  # Candidate height only; not the mechanical or verified safe maximum.
    pose("left_raise", high, "FK1")
    gripper_events.append({"after": "FK1", "left_opening": 80., "right_opening": None})
    low_wave = high.copy()
    low_wave[0] = -1.0
    pose("left_wave_down_1", low_wave)
    gripper_events.append({"after": "left_wave_down_1", "left_opening": 30., "right_opening": None})
    pose("left_wave_up_1", high)
    gripper_events.append({"after": "left_wave_up_1", "left_opening": 80., "right_opening": None})
    pose("left_wave_down_2", low_wave)
    pose("left_wave_up_2", high)

    heart = HOME.copy()
    # Elbows move outward and wrists rise inward, leaving a camera-sized gap.
    heart[:7] = [-1.425, .586, -.813, -2., .079, .414, 0.]
    heart[7:] = [-1.425, -.586, .813, -2., -.079, .414, 0.]
    pose("chest_heart_candidate", heart, "FK2")
    gripper_events.append({"after": "FK2", "left_opening": 30., "right_opening": 30.})
    gripper_events.append({"after": "FK2_display", "left_opening": 80., "right_opening": 80.})
    pose("reset_after_heart", HOME)

    forward = HOME.copy()
    forward[0], forward[7] = -.25, +.25
    pose("left_forward_right_back", forward)
    ik1 = forward.copy()
    for side, offset, dx in (("left", 0, +.008), ("right", 7, -.008)):
        seed = forward[offset:offset+7]
        target = arm_points(chains[side], seed)[-1]+np.array([dx, 0., 0.])
        ik1[offset:offset+7], error, iterations = position_ik(chains, seed, side, target)
        milestones.setdefault("IK1", {})[side] = {"target_model_xyz_m": target.tolist(),
                                                   "solver_error_m": error,
                                                   "iterations": iterations}
    pose("IK1_left_forward_right_back", ik1)

    backward = HOME.copy()
    backward[0], backward[7] = +.25, -.25
    pose("left_back_right_forward", backward)
    ik2 = backward.copy()
    for side, offset, dx in (("left", 0, -.008), ("right", 7, +.008)):
        seed = backward[offset:offset+7]
        target = arm_points(chains[side], seed)[-1]+np.array([dx, 0., 0.])
        ik2[offset:offset+7], error, iterations = position_ik(chains, seed, side, target)
        milestones.setdefault("IK2", {})[side] = {"target_model_xyz_m": target.tolist(),
                                                   "solver_error_m": error,
                                                   "iterations": iterations}
    pose("IK2_left_back_right_forward", ik2)
    pose("final_reset", HOME)
    gripper_events.append({"after": "final_reset", "restore_initial_opening": True})

    minimum = min(min(frame["camera_proxy_clearance_m"].values()) for frame in frames)
    return {"kind": "course_model_kinematic_candidate_only",
            "hardware_motion_enabled": False,
            "camera_proxy_radius_m": camera_radius,
            "required_camera_clearance_m": REQUIRED_CLEARANCE_M,
            "camera_geometry_verified": False,
            "camera_proxy_clearance_passed": minimum >= REQUIRED_CLEARANCE_M,
            "minimum_camera_proxy_clearance_m": minimum,
            "highest_pose_note": "FK1 is a candidate raised pose, not the robot's absolute maximum",
            "heart_pose_note": "FK2 is a symmetric chest-heart candidate; appearance needs visual review",
            "milestones": milestones, "gripper_events": gripper_events,
            "frames": frames}


def render_preview(report, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    names = ["start", "left_raise", "left_wave_down_1", "chest_heart_candidate",
             "left_forward_right_back", "left_back_right_forward", "final_reset"]
    fig, axes = plt.subplots(2, len(names), figsize=(20, 7), sharex="row", sharey="row")
    fixed = load_chains()["fixed_camera_center"]
    for column, name in enumerate(names):
        frame = next(item for item in reversed(report["frames"]) if item["phase"] == name)
        left = np.array(frame["left_joint_xyz_m"])
        right = np.array(frame["right_joint_xyz_m"])
        for row, (a, b, title) in enumerate(((1, 2, "front: y / z"), (0, 2, "side: x / z"))):
            ax = axes[row, column]
            ax.plot(left[:, a], left[:, b], "o-", color="#c33", linewidth=2, markersize=3)
            ax.plot(right[:, a], right[:, b], "o-", color="#36c", linewidth=2, markersize=3)
            ax.add_patch(Circle((left[-1, a], left[-1, b]),
                                report["camera_proxy_radius_m"]+REQUIRED_CLEARANCE_M/2,
                                fill=False, color="#c33", alpha=.5))
            ax.add_patch(Circle((right[-1, a], right[-1, b]),
                                report["camera_proxy_radius_m"]+REQUIRED_CLEARANCE_M/2,
                                fill=False, color="#36c", alpha=.5))
            ax.plot(fixed[a], fixed[b], "kx", markersize=7)
            ax.set_aspect("equal", adjustable="box")
            ax.grid(alpha=.2)
            if column == 0:
                ax.set_ylabel(title + " (m)")
            if row == 0:
                ax.set_title(name.replace("_", " "), fontsize=9)
    axes[0, 0].set_xlim(-.45, .45)
    axes[0, 0].set_ylim(-.45, .35)
    axes[1, 0].set_xlim(-.1, .7)
    axes[1, 0].set_ylim(-.45, .35)
    fig.suptitle("TRON2 choreography candidate: red left, blue right, circles wrist-camera proxies")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def render_animation(report, output):
    """Render a portable kinematic GIF; this is not a MuJoCo dynamics replay."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Circle

    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    radius = report["camera_proxy_radius_m"]+REQUIRED_CLEARANCE_M/2
    fixed = load_chains()["fixed_camera_center"]
    drawings = []
    for ax, (a, b, label, xlim) in zip(
            axes, ((1, 2, "front: y / z", (-.45, .45)),
                   (0, 2, "side: x / z", (-.1, .7)))):
        ax.set(xlim=xlim, ylim=(-.45, .35), xlabel=label+" (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=.2)
        ax.plot(fixed[a], fixed[b], "kx", markersize=7)
        left_line, = ax.plot([], [], "o-", color="#c33", linewidth=2)
        right_line, = ax.plot([], [], "o-", color="#36c", linewidth=2)
        left_circle = Circle((0, 0), radius, fill=False, color="#c33", alpha=.5)
        right_circle = Circle((0, 0), radius, fill=False, color="#36c", alpha=.5)
        ax.add_patch(left_circle)
        ax.add_patch(right_circle)
        drawings.append((a, b, left_line, right_line, left_circle, right_circle))

    frames = report["frames"]

    def update(index):
        frame = frames[index]
        left = np.asarray(frame["left_joint_xyz_m"])
        right = np.asarray(frame["right_joint_xyz_m"])
        fig.suptitle(f"TRON2 candidate {index+1}/{len(frames)}: "
                     f"{frame['phase'].replace('_', ' ')}")
        for a, b, left_line, right_line, left_circle, right_circle in drawings:
            left_line.set_data(left[:, a], left[:, b])
            right_line.set_data(right[:, a], right[:, b])
            left_circle.center = (left[-1, a], left[-1, b])
            right_circle.center = (right[-1, a], right[-1, b])

    animation = FuncAnimation(fig, update, frames=len(frames), interval=125)
    animation.save(output, writer=PillowWriter(fps=8), dpi=100)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--camera-proxy-radius", type=float,
                        default=WRIST_CAMERA_PROXY_RADIUS_M)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error("output directory already exists; use a fresh path")
    if not .05 <= args.camera_proxy_radius <= .2:
        parser.error("camera proxy radius must be 0.05..0.20 m")
    report = build_candidate(load_chains(), args.camera_proxy_radius)
    args.output_dir.mkdir(parents=True)
    (args.output_dir / "candidate.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    render_preview(report, args.output_dir / "candidate_preview.png")
    render_animation(report, args.output_dir / "candidate_animation.gif")
    print(json.dumps({"output": str(args.output_dir),
                      "milestones": list(report["milestones"]),
                      "frames": len(report["frames"]),
                      "camera_proxy_clearance_passed": report["camera_proxy_clearance_passed"],
                      "minimum_camera_proxy_clearance_m": report["minimum_camera_proxy_clearance_m"]}))


if __name__ == "__main__":
    main()

"""Two FK observations and two controller-IK (MoveP) points on a TRON2 arm.

Default is read-only planning. --send explicitly enables four small movements.
Course-model FK is reported separately from SDK pose; their frames are not calibrated.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from client import SimClient, load_robot_profile, map_state, pose_pair
from lessons import pose_offset_corrected, stationary_q


def confirm(summary):
    print(summary, flush=True)
    try:
        answer = input("Clear the arm workspace; operator at E-stop. Type y to send: ")
    except EOFError:
        answer = ""
    if answer.strip().lower() != "y":
        raise RuntimeError("operator declined; no further motion sent")


def model_fk(course_code, q7, side):
    sys.path.insert(0, str(course_code))
    try:
        from students.kinematics import fk
        return fk(q7, side).tolist()
    finally:
        sys.path.pop(0)


def json_value(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    raise TypeError(type(value).__name__)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", required=True, type=Path, help="reviewed real-robot profile")
    parser.add_argument("--course-code", type=Path,
                        default=Path(__file__).resolve().parents[2] / "Course_Code")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--send", action="store_true", help="enable physical motion; each command asks y")
    parser.add_argument("--duration", type=float, default=5.)
    parser.add_argument("--output", type=Path, default=Path("four_point_report.json"))
    args = parser.parse_args()
    if args.send and args.output.exists():
        parser.error("--output already exists; choose a new live report path to preserve the previous run")
    if not (args.course_code / "students" / "kinematics.py").is_file():
        parser.error("--course-code must point to the Course_Code directory")
    robot = load_robot_profile(args.robot)
    if args.duration < robot["min_duration"] or args.duration > 30:
        parser.error(f"--duration must be {robot['min_duration']}..30 seconds")

    client = SimClient(robot=robot)
    client.confirm = confirm
    report = {"mode": "hardware_live" if args.send else "read_only_plan",
              "status": "in_progress", "robot_profile": str(args.robot),
              "side": args.side, "course_model_frame": "base_Link to wrist joint origin",
              "sdk_frame": "base bottom center to last-joint end frame",
              "frame_calibrated": False, "physical_motion_verified": False, "steps": []}
    arm_start = 0 if args.side == "left" else 7
    elbow_index = arm_start + 3
    try:
        with client:
            q_initial = map_state(client.joint_state(), names=client.joint_names)
            pose_initial = client.move_pose()
            report["initial_q14_rad"] = q_initial
            report["initial_sdk_pose_xyz_wxyz"] = pose_initial
            report["initial_course_fk"] = model_fk(args.course_code, q_initial[arm_start:arm_start+7], args.side)
            report["initial_left_xyz_m"] = pose_initial[:3]
            report["initial_right_xyz_m"] = pose_initial[7:10]
            # MoveP validates both arms, even when only one arm is selected.
            # Check this before either FK motion so a hanging-arm start fails safely.
            pose_pair(pose_initial[:7], pose_initial[7:])
            first_pose_target = pose_initial.copy()
            first_pose_target[arm_start] += .005
            pose_pair(first_pose_target[:7], first_pose_target[7:])

            # Select two 0.02-rad steps toward whichever side has more room.
            room_up = client.upper[elbow_index] - q_initial[elbow_index]
            room_down = q_initial[elbow_index] - client.lower[elbow_index]
            direction = 1 if room_up >= .04 else -1 if room_down >= .04 else 0
            if not direction:
                raise ValueError("elbow has less than 0.04 rad of safe profile range on both sides")
            targets = []
            for number in (1, 2):
                target = q_initial.copy()
                target[elbow_index] += direction * number * .02
                client.arm_target(target)
                targets.append(target)
            report["fk_joint_targets_q14_rad"] = targets
            report["ik_sdk_offsets_m"] = [+.005, -.005]
            if not args.send:
                report["status"] = "read_only_plan"
            else:
                for number, target in enumerate(targets, 1):
                    # Re-read before every move; unexpected drift invalidates the plan.
                    actual = stationary_q(client)
                    expected = q_initial if number == 1 else targets[number-2]
                    if max(abs(a-b) for a, b in zip(actual, expected)) > .02:
                        raise RuntimeError("joint feedback changed since the previous point; stopped")
                    accepted = client.movej(target, args.duration)
                    arrival = client.wait_joints(target, timeout=args.duration+10.,
                                                 min_duration=args.duration)
                    result = {"target": target, "accepted": accepted, "arrival": arrival}
                    observed = map_state(client.joint_state(), names=client.joint_names)
                    pose = client.move_pose()
                    joint_progress = observed[elbow_index]-actual[elbow_index]
                    joint_error = abs(observed[elbow_index]-target[elbow_index])
                    report["steps"].append({"point": f"FK{number}", "movej": result,
                                            "observed_q14_rad": observed,
                                            "observed_elbow_change_rad": joint_progress,
                                            "elbow_target_error_rad": joint_error,
                                            "course_fk": model_fk(args.course_code, observed[arm_start:arm_start+7], args.side),
                                            "sdk_pose_xyz_wxyz": pose})
                    # A 0.02-rad command must not pass solely because the
                    # configured arrival tolerance is also 0.02 rad.
                    if direction*joint_progress < .012 or joint_error > .005:
                        raise RuntimeError("MoveJ feedback did not confirm the small elbow movement; stopped")
                    pose_pair(pose[:7], pose[7:])
                for number, offset in enumerate((+.005, -.005), 1):
                    # The robot's MoveP controller performs IK in its own SDK frame.
                    result = pose_offset_corrected(client, args.side, 0, offset,
                                                   args.duration, corrections=2)
                    observed = map_state(client.joint_state(), names=client.joint_names)
                    report["steps"].append({"point": f"IK{number}", "movep": result,
                                            "observed_q14_rad": observed,
                                            "course_fk": model_fk(args.course_code, observed[arm_start:arm_start+7], args.side),
                                            "sdk_pose_xyz_wxyz": client.move_pose()})
                report["status"] = "completed_four_points"
    except Exception as exc:
        report.update(status="stopped", error=f"{type(exc).__name__}: {exc}")
    finally:
        report["notifications"] = client.notifications
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=json_value), encoding="utf-8")
        if args.send:
            client.save_events(args.output.with_suffix(".events.json"))
    print(json.dumps({"status": report["status"], "steps": len(report["steps"]),
                      "output": str(args.output), "error": report.get("error")}, ensure_ascii=False))
    return int(report["status"] == "stopped")


if __name__ == "__main__":
    raise SystemExit(main())

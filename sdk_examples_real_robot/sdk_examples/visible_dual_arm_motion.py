"""Visible two-arm elbow extension and return, using the reviewed real-robot profile.

Read-only plan by default. --send asks the operator before every MoveJ segment.
By default one elbow moves at a time; --simultaneous moves both in each segment.
Neither mode is a horizontal-arm pose solver.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from client import SimClient, load_robot_profile, map_state


STEP_RAD = .08
ELBOWS = ((3, "left"), (10, "right"))


def plan_targets(initial, amplitude, simultaneous=False):
    """Generate a reversible elbow route from measured joint angles."""
    count = round(amplitude / STEP_RAD)
    if count not in (1, 2, 3, 4) or not math.isclose(count * STEP_RAD, amplitude, abs_tol=1e-9):
        raise ValueError("amplitude must be 0.08, 0.16, 0.24 or 0.32 rad")
    if simultaneous:
        targets = []
        for phase, steps in (("extend", range(1, count+1)),
                             ("return", range(count-1, -1, -1))):
            for step in steps:
                target = initial.copy()
                for joint_index, _ in ELBOWS:
                    target[joint_index] += step * STEP_RAD
                targets.append({"phase": phase, "side": "both",
                                "joint_indices": [index for index, _ in ELBOWS],
                                "target_q14_rad": target})
        return targets
    targets = []
    offsets = [0., 0.]
    for arm_index, sequence in ((0, range(1, count+1)),
                                (1, range(1, count+1)),
                                (1, range(count-1, -1, -1)),
                                (0, range(count-1, -1, -1))):
        joint_index, side = ELBOWS[arm_index]
        for step in sequence:
            offsets[arm_index] = step * STEP_RAD
            target = initial.copy()
            target[ELBOWS[0][0]] += offsets[0]
            target[ELBOWS[1][0]] += offsets[1]
            targets.append({"phase": "extend" if step and len(targets) < 2*count else "return",
                            "side": side, "joint_index": joint_index,
                            "target_q14_rad": target})
    return targets


def confirm(summary):
    print(summary, flush=True)
    try:
        answer = input("Check the physical path and keep an operator at E-stop. Type y to send: ")
    except EOFError:
        answer = ""
    if answer.strip().lower() != "y":
        raise RuntimeError("operator declined; no further motion sent")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", required=True, type=Path)
    parser.add_argument("--send", action="store_true", help="send physical MoveJ segments")
    parser.add_argument("--simultaneous", action="store_true",
                        help="move both elbows in each small segment instead of one at a time")
    parser.add_argument("--amplitude", type=float, default=.32,
                        help="total elbow extension per arm: 0.08, 0.16, 0.24 or 0.32 rad")
    parser.add_argument("--duration", type=float, default=8., help="seconds per segment, at least 5")
    parser.add_argument("--output", type=Path, default=Path("dual_arm_motion_report.json"))
    args = parser.parse_args()
    if args.output.exists():
        parser.error("--output already exists; choose a new report name")
    robot = load_robot_profile(args.robot)
    if not robot["min_duration"] <= args.duration <= 30.:
        parser.error(f"--duration must be {robot['min_duration']}..30 seconds")
    if robot["max_joint_delta"] < STEP_RAD + .01:
        parser.error("profile max_segment_delta_rad is too small for 0.08-rad segments plus feedback tolerance")
    # Check the amplitude before connecting, without making assumptions about pose.
    plan_targets([0.] * 14, args.amplitude, args.simultaneous)

    client = SimClient(robot=robot)
    client.confirm = confirm
    report = {"mode": "hardware_live" if args.send else "read_only_plan",
              "status": "in_progress", "physical_motion_verified": False,
              "robot_profile": str(args.robot), "amplitude_rad": args.amplitude,
              "simultaneous": args.simultaneous,
              "duration_per_segment_s": args.duration, "steps": []}
    try:
        with client:
            initial = map_state(client.joint_state(), names=client.joint_names)
            client.arm_target(initial)
            report["initial_q14_rad"] = initial
            report["initial_sdk_pose_xyz_wxyz"] = client.move_pose()
            targets = plan_targets(initial, args.amplitude, args.simultaneous)
            for stage in targets:
                client.arm_target(stage["target_q14_rad"])
            report["plan"] = targets
            if not args.send:
                report["status"] = "read_only_plan"
            else:
                previous = initial
                halfway = len(targets) // 2
                for number, stage in enumerate(targets, 1):
                    actual = map_state(client.joint_state(), names=client.joint_names)
                    if max(abs(a-b) for a, b in zip(actual, previous)) > .02:
                        raise RuntimeError("feedback differs from the previous target; stopped")
                    target = stage["target_q14_rad"]
                    indices = stage.get("joint_indices") or [stage["joint_index"]]
                    accepted = client.movej(target, args.duration)
                    arrived = client.wait_joints(target, timeout=args.duration+15.,
                                                 min_duration=args.duration)
                    observed = map_state(client.joint_state(), names=client.joint_names)
                    progress = {client.joint_names[index]: observed[index] - actual[index]
                                for index in indices}
                    error = {client.joint_names[index]: abs(observed[index] - target[index])
                             for index in indices}
                    report["steps"].append({"number": number, "phase": stage["phase"],
                                            "side": stage["side"], "accepted": accepted,
                                            "arrival": arrived, "observed_q14_rad": observed,
                                            "sdk_pose_xyz_wxyz": client.move_pose(),
                                            "selected_joint_change_rad": progress,
                                            "selected_joint_target_error_rad": error})
                    if any(abs(value) < .05 for value in progress.values()) or \
                            any(value > .01 for value in error.values()):
                        raise RuntimeError("feedback did not confirm this visible joint step; stopped")
                    previous = target
                    if number == halfway:
                        print("Both elbows reached the planned extended pose.", flush=True)
                        input("Inspect the physical arms. Press Enter to begin the return, or Ctrl+C to stop: ")
                report["status"] = "feedback_confirmed_returned"
    except KeyboardInterrupt:
        report.update(status="interrupted", error="operator interrupted; no automatic recovery motion")
    except Exception as exc:
        report.update(status="stopped", error=f"{type(exc).__name__}: {exc}")
    finally:
        report["notifications"] = client.notifications
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        if args.send:
            client.save_events(args.output.with_suffix(".events.json"))
    print(json.dumps({"status": report["status"], "steps": len(report["steps"]),
                      "output": str(args.output), "error": report.get("error")}, ensure_ascii=False))
    return int(report["status"] in ("stopped", "interrupted"))


if __name__ == "__main__":
    raise SystemExit(main())

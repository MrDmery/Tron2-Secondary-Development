"""Real TRON2: two FK poses, two controller-IK poses, then return to start.

Read-only planning is the default. --send runs the checked sequence without prompts.
FK uses the course model for reporting; IK motion uses the robot's MoveP solver.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

from client import SimClient, load_robot_profile, map_state, pose_pair, pose_errors
from lessons import HOME, gripper_opening, stationary_q, wait_pose_settled


ELBOWS = (3, 10)
SHOULDER_YAWS = (2, 9)
EXTENSION_RAD = .32
RETURN_STEP_RAD = .07
FK_OFFSETS = {
    "simple": ((.08, .08), (.16, .16), (.24, .24), (.32, .32)),
    "varied": ((.08, .08), (.16, .16), (.24, .08), (.32, 0.)),
    "wide": ((.09, .09), (.18, .18), (.27, .27), (.36, .18), (.45, .09)),
    "sway": ((.09, .09), (.18, .18), (.27, .18), (.36, .09), (.45, .09)),
}
FK_POINT_SEGMENTS = {"simple": (2, 4), "varied": (2, 4),
                     "wide": (3, 5), "sway": (2, 5)}
SHOULDER_YAW_OFFSETS = {"sway": (.06, .12, .04, -.04, -.12)}
IK_DELTAS = {
    "simple": (((.005, 0., 0.), (.005, 0., 0.)),
               ((-.005, 0., 0.), (-.005, 0., 0.))),
    "varied": (((.008, .004, 0.), (.008, -.004, 0.)),
               ((.008, -.004, 0.), (.008, .004, 0.))),
    "wide": (((.008, .004, 0.), (.008, -.004, 0.)),
             ((.008, -.004, 0.), (.008, .004, 0.))),
    "sway": (((.004, .008, 0.), (.004, .008, 0.)),
             ((.004, -.008, 0.), (.004, -.008, 0.))),
}


def log_motion(summary):
    print(summary, flush=True)


def read_gripper_openings(client):
    state = client.request("request_get_limx_2fclaw_state")
    openings = {side: float(state[f"{side}_opening"]) for side in ("left", "right")}
    if any(not math.isfinite(value) or not 0. <= value <= 100.
           for value in openings.values()):
        raise ValueError("gripper feedback opening must be in 0..100")
    return openings


def move_grippers(client, targets, profile, report, output, label):
    for side in ("left", "right"):
        result = gripper_opening(client, side, targets[side],
                                 speed=profile["speed"], force=min(profile["force"], 20.),
                                 timeout=10.)
        report["gripper_steps"].append({"label": label, "side": side,
                                         "target_opening": targets[side], "result": result})
        save(output, report)


def move_both_pose_offset(client, delta, duration, corrections=2, tolerance=.003):
    """Small SDK-frame MoveP for both hands with measured half-error corrections."""
    stationary_q(client)
    initial = client.move_pose()
    target = initial.copy()
    deltas = ((delta, 0., 0.),) * 2 if isinstance(delta, (int, float)) else delta
    if len(deltas) != 2 or any(len(shift) != 3 or math.dist((0., 0., 0.), shift) > .01
                               for shift in deltas):
        raise ValueError("MoveP needs two XYZ offsets of at most 10 mm")
    for index, shift in zip((0, 7), deltas):
        target[index:index+3] = [value+change for value, change in
                                 zip(target[index:index+3], shift)]
    pose_pair(target[:7], target[7:])
    command, steps = target.copy(), []
    for number in range(corrections+1):
        accepted = client.movep(command[:7], command[7:], duration)
        measured, trace = wait_pose_settled(client, duration, duration+15.)
        position, angle = pose_errors(target, measured)
        steps.append({"number": number, "command": command.copy(),
                      "accepted": accepted, "measured": measured,
                      "position_error_m": position, "orientation_error_rad": angle,
                      "settle_samples": len(trace)})
        if position <= tolerance:
            return {"status": "arrived", "initial": initial, "target": target,
                    "delta_xyz_m": deltas,
                    "steps": steps, "corrections_used": number}
        if number and position >= steps[-2]["position_error_m"]:
            raise RuntimeError("MoveP correction did not reduce the pose error")
        for offset in (0, 7):
            command[offset:offset+3] = [c+.5*(t-m) for c, t, m in
                                        zip(command[offset:offset+3],
                                            target[offset:offset+3], measured[offset:offset+3])]
        if max(math.dist(command[o:o+3], target[o:o+3]) for o in (0, 7)) > .01:
            raise ValueError("MoveP correction exceeds the 10 mm target envelope")
    raise TimeoutError("MoveP target not reached after two corrections")


def fk_matrix(course_code, q7, side):
    sys.path.insert(0, str(course_code.resolve()))
    try:
        from students.kinematics import fk
        return fk(q7, side).tolist()
    finally:
        sys.path.pop(0)


def observed(client, course_code):
    q = map_state(client.joint_state(), names=client.joint_names)
    return {"q14_rad": q, "sdk_pose_xyz_wxyz": client.move_pose(),
            "course_fk_left": fk_matrix(course_code, q[:7], "left"),
            "course_fk_right": fk_matrix(course_code, q[7:], "right")}


def save(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")


def movej_checked(client, target, duration, course_code, report, label, previous):
    actual = stationary_q(client)
    if max(abs(a-b) for a, b in zip(actual, previous)) > .02:
        raise RuntimeError(f"{label}: feedback differs from previous waypoint; stopped")
    accepted = client.movej(target, duration)
    arrival = client.wait_joints(target, timeout=duration+15., min_duration=duration)
    state = observed(client, course_code)
    error = max(abs(a-b) for a, b in zip(state["q14_rad"], target))
    changed = [i for i, (a, b) in enumerate(zip(actual, target)) if abs(a-b) >= .05]
    progress = {client.joint_names[i]: state["q14_rad"][i]-actual[i] for i in changed}
    report["steps"].append({"label": label, "target_q14_rad": target,
                            "accepted": accepted, "arrival": arrival,
                            "max_target_error_rad": error,
                            "selected_joint_change_rad": progress, "observed": state})
    if error > .01 or any(abs(value) < .04 for value in progress.values()):
        raise RuntimeError(f"{label}: joint feedback did not confirm waypoint; stopped")
    return state


def reset_waypoints(start, goal, step_rad=RETURN_STEP_RAD):
    distance = max(abs(a-b) for a, b in zip(start, goal))
    if distance < .005:
        return []
    count = max(1, math.ceil(distance/step_rad))
    return [[a+(b-a)*step/count for a, b in zip(start, goal)]
            for step in range(1, count+1)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", required=True, type=Path)
    parser.add_argument("--course-code", required=True, type=Path)
    parser.add_argument("--send", action="store_true")
    parser.add_argument("--pattern", choices=tuple(FK_OFFSETS), default="simple",
                        help="sway adds shoulder yaw and a shorter return; other routes stay available")
    parser.add_argument("--with-gripper", action="store_true",
                        help="open and close installed LimX two-finger grippers; requires enabled profile")
    parser.add_argument("--duration", type=float, default=8., help="seconds per MoveJ/MoveP segment")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("--output already exists; choose a new report name")
    if not (args.course_code / "students" / "kinematics.py").is_file():
        parser.error("--course-code must point to Course_Code")
    robot = load_robot_profile(args.robot)
    if args.with_gripper and not robot["gripper"].get("enabled"):
        parser.error("--with-gripper requires a verified LimX two-finger gripper and profile.gripper.enabled=true")
    if not robot["min_duration"] <= args.duration <= 30.:
        parser.error(f"--duration must be {robot['min_duration']}..30 seconds")
    offsets = FK_OFFSETS[args.pattern]
    prior = (0., 0.)
    largest_step = 0.
    for pair in offsets:
        largest_step = max(largest_step, *(abs(a-b) for a, b in zip(pair, prior)))
        prior = pair
    yaw_offsets = SHOULDER_YAW_OFFSETS.get(args.pattern, (0.,)*len(offsets))
    largest_step = max(largest_step, *(abs(a-b) for a, b in
                                      zip(yaw_offsets, (0.,)+yaw_offsets[:-1])))
    if robot["max_joint_delta"]+1e-9 < largest_step+.01:
        parser.error("profile joint step limit is too small for this pattern plus feedback margin")
    return_step_rad = .09 if args.pattern in ("wide", "sway") else RETURN_STEP_RAD

    client = SimClient(robot=robot)
    # --send is the operator's one-time authorization for this bounded sequence.
    # The client still checks every target and prints each command before sending.
    client.confirm = log_motion
    report = {"mode": "hardware_live" if args.send else "read_only_plan",
              "status": "in_progress", "physical_motion_verified": False,
              "robot_profile": str(args.robot), "course_model_frame_calibrated_to_sdk": False,
              "pattern": args.pattern, "ik_step_deltas_sdk_m": IK_DELTAS[args.pattern],
              "fk_point_segments": FK_POINT_SEGMENTS[args.pattern],
              "shoulder_yaw_offsets_rad": yaw_offsets,
              "reset_max_segment_rad": return_step_rad,
              "return_direct": args.pattern == "sway",
              "with_gripper": args.with_gripper, "gripper_steps": [],
              "steps": []}
    if args.pattern == "simple":
        report.update(fk_extension_rad_per_arm=EXTENSION_RAD,
                      ik_sdk_x_offsets_m=[+.005, -.005])
    try:
        with client:
            initial = observed(client, args.course_code)
            start_q = initial["q14_rad"]
            report["initial"] = initial
            initial_gripper = read_gripper_openings(client) if args.with_gripper else None
            if initial_gripper is not None:
                report["initial_gripper_openings"] = initial_gripper
            client.arm_target(start_q)
            if max(abs(a-b) for a, b in zip(start_q, HOME)) > .05:
                raise ValueError("start pose differs from official prepare pose by >0.05 rad; run prepare first")
            pose_pair(initial["sdk_pose_xyz_wxyz"][:7], initial["sdk_pose_xyz_wxyz"][7:])
            # FK1 is symmetric; the optional patterns make FK2 asymmetric.
            fk_targets = []
            for arm, index in enumerate(ELBOWS):
                if start_q[index]+max(pair[arm] for pair in offsets) > client.upper[index]-.01:
                    raise ValueError(f"{client.joint_names[index]} lacks room for extension")
            for pair, yaw_offset in zip(offsets, yaw_offsets):
                q = start_q.copy()
                for index, offset in zip(ELBOWS, pair):
                    q[index] = start_q[index]+offset
                for index in SHOULDER_YAWS:
                    q[index] = start_q[index]+yaw_offset
                client.arm_target(q)
                fk_targets.append(q)
            first_point, second_point = FK_POINT_SEGMENTS[args.pattern]
            report["fk_targets_q14_rad"] = {"FK1": fk_targets[first_point-1],
                                             "FK2": fk_targets[second_point-1]}
            report["fk_point_lateral_displacement_sdk_m"] = {}
            report["ik_targets_note"] = "MoveP targets are relative to measured SDK pose after FK2; cannot be fixed before motion"
            route = "directly to initial joint state" if args.pattern == "sway" else \
                    "through measured FK2, measured FK1, then initial joint state"
            report["reset_route_note"] = f"return {route}; <={return_step_rad:.2f} rad per reset segment"
            if args.with_gripper:
                report["gripper_plan"] = {"FK1": 30., "FK2": 80.,
                                           "return": initial_gripper}
            save(args.output, report)
            if not args.send:
                report["status"] = "read_only_plan"
            else:
                checkpoints = []
                previous = start_q
                for number, target in enumerate(fk_targets, 1):
                    label = f"FK segment {number}/{len(fk_targets)}"
                    state = movej_checked(client, target, args.duration, args.course_code,
                                          report, label, previous)
                    save(args.output, report)
                    previous = target
                    if number in FK_POINT_SEGMENTS[args.pattern]:
                        checkpoints.append(state["q14_rad"])
                        report["fk_point_lateral_displacement_sdk_m"][f"FK{len(checkpoints)}"] = {
                            "left": state["sdk_pose_xyz_wxyz"][1]-initial["sdk_pose_xyz_wxyz"][1],
                            "right": state["sdk_pose_xyz_wxyz"][8]-initial["sdk_pose_xyz_wxyz"][8],
                        }
                        save(args.output, report)
                        if args.with_gripper:
                            opening = 30. if number == first_point else 80.
                            move_grippers(client, {"left": opening, "right": opening},
                                          robot["gripper"], report, args.output,
                                          f"after FK{len(checkpoints)}")
                    pose = state["sdk_pose_xyz_wxyz"]
                    pose_pair(pose[:7], pose[7:])
                for number, deltas in enumerate(IK_DELTAS[args.pattern], 1):
                    result = move_both_pose_offset(client, deltas, args.duration,
                                                   corrections=2)
                    state = observed(client, args.course_code)
                    report["steps"].append({"label": f"IK{number}",
                                            "controller_ik": result, "observed": state})
                    save(args.output, report)
                    projected = []
                    for index, requested in zip((0, 7), deltas):
                        actual = [end-start for end, start in
                                  zip(state["sdk_pose_xyz_wxyz"][index:index+3],
                                      result["initial"][index:index+3])]
                        projected.append(sum(a*d for a, d in zip(actual, requested)) /
                                         math.dist((0., 0., 0.), requested))
                    if any(distance < .002 for distance in projected):
                        raise RuntimeError(f"IK{number}: SDK pose feedback did not show requested motion")
                # The sway route returns directly to start to avoid a repeated FK1 arc.
                return_goals = (("initial", start_q),) if args.pattern == "sway" else \
                               (("FK2", checkpoints[1]), ("FK1", checkpoints[0]),
                                ("initial", start_q))
                for label, goal in return_goals:
                    current = map_state(client.joint_state(), names=client.joint_names)
                    for number, target in enumerate(reset_waypoints(current, goal, return_step_rad), 1):
                        client.arm_target(target)
                        state = movej_checked(client, target, args.duration, args.course_code,
                                              report, f"return toward {label} segment {number}", current)
                        save(args.output, report)
                        current = target
                final = observed(client, args.course_code)
                q_error = max(abs(a-b) for a, b in zip(final["q14_rad"], start_q))
                position_error, angle_error = pose_errors(initial["sdk_pose_xyz_wxyz"],
                                                          final["sdk_pose_xyz_wxyz"])
                report["final"] = final
                report["reset_error"] = {"max_joint_rad": q_error,
                                         "max_position_m": position_error,
                                         "max_orientation_rad": angle_error}
                if q_error > .01 or position_error > .015 or angle_error > .05:
                    raise RuntimeError("return to startup pose not confirmed by SDK feedback")
                if args.with_gripper:
                    move_grippers(client, initial_gripper, robot["gripper"], report,
                                  args.output, "restore initial opening")
                report["status"] = "four_points_and_reset_feedback_confirmed"
    except KeyboardInterrupt:
        report.update(status="interrupted", error="operator interrupted; no automatic recovery motion")
    except Exception as exc:
        report.update(status="stopped", error=f"{type(exc).__name__}: {exc}")
    finally:
        report["notifications"] = client.notifications
        save(args.output, report)
        if args.send:
            client.save_events(args.output.with_suffix(".events.json"))
    print(json.dumps({"status": report["status"], "steps": len(report["steps"]),
                      "output": str(args.output), "error": report.get("error")}, ensure_ascii=False))
    return int(report["status"] in ("stopped", "interrupted"))


if __name__ == "__main__":
    raise SystemExit(main())

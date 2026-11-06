"""TRON2 choreography: FK1/FK2, controller IK1/IK2, grippers, and reset.

Read-only by default. --send --site-checked starts one automatic hardware run.
The camera test uses an unverified wrist-centered proxy, not real geometry.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

from client import SimClient, load_robot_profile, map_state, pose_errors
from fk_ik_two_points_reset import (move_both_pose_offset, read_gripper_openings,
                                     save)
from lessons import HOME, gripper_opening, stationary_q


PHASES = ("left_raise", "left_wave_down_1", "left_wave_up_1",
          "left_wave_down_2", "left_wave_up_2", "chest_heart_candidate",
          "reset_after_heart", "left_forward_right_back",
          "left_back_right_forward", "final_reset")
IK_DELTAS = {"IK1": ((.008, 0., 0.), (-.008, 0., 0.)),
             "IK2": ((-.008, 0., 0.), (.008, 0., 0.))}


def load_candidate(course_code):
    sys.path.insert(0, str(course_code.resolve()))
    try:
        from choreography_candidate import (REQUIRED_CLEARANCE_M, build_candidate,
                                            camera_clearances, load_chains,
                                            wrist_xyz)
    finally:
        sys.path.pop(0)
    chains = load_chains(course_code / "models" / "DA_TRON2A_fixed_no_tool.xml")
    candidate = build_candidate(chains)
    if not candidate["camera_proxy_clearance_passed"]:
        raise ValueError("offline candidate failed assumed camera proxy clearance")
    endpoints = {frame["phase"]: frame["q14_rad"] for frame in candidate["frames"]}
    return (candidate, chains, endpoints, camera_clearances,
            REQUIRED_CLEARANCE_M, wrist_xyz)


def fk_observation(client, chains, wrist_fk):
    q = stationary_q(client)
    return {"q14_rad": q, "sdk_pose_xyz_wxyz": client.move_pose(),
            "course_wrist_fk_xyz_m": {
                side: wrist_fk(chains, q, side).tolist()
                for side in ("left", "right")}}


def checked_waypoints(start, goal, step_rad, client, chains, camera_check, clearance):
    distance = max(abs(a-b) for a, b in zip(start, goal))
    count = max(1, math.ceil(distance/step_rad))
    waypoints = [[a+(b-a)*number/count for a, b in zip(start, goal)]
                 for number in range(1, count+1)]
    previous = start
    for target in waypoints:
        client.arm_target(target)
        samples = max(5, math.ceil(max(abs(a-b) for a, b in
                                       zip(previous, target))/.02))
        for substep in range(samples+1):
            fraction = substep/samples
            q = [a+(b-a)*fraction for a, b in zip(previous, target)]
            minimum = min(camera_check(chains, q).values())
            if minimum < clearance:
                raise ValueError(f"camera proxy clearance {minimum:.3f} m below "
                                 f"{clearance:.3f} m; stopped")
        previous = target
    return waypoints


def move_phase(client, label, goal, duration, step_rad, chains, camera_check,
               clearance, report, output, allow_large_move=False):
    start = stationary_q(client)
    waypoints = checked_waypoints(start, goal, step_rad, client, chains,
                                  camera_check, clearance)
    for number, target in enumerate(waypoints, 1):
        before = stationary_q(client)
        client.arm_target(target)
        delta = max(abs(a-b) for a, b in zip(before, target))
        segment_duration = max(client.robot["min_duration"],
                               round(duration*min(delta/step_rad, 1.), 6))
        report["inflight_command"] = {"type": "MoveJ", "phase": label,
                                      "segment": number, "target_q14_rad": target,
                                      "duration_s": segment_duration}
        save(output, report)
        accepted = client.movej(target, segment_duration,
                                large_move=allow_large_move)
        arrival = client.wait_joints(target, timeout=segment_duration+15.,
                                     min_duration=segment_duration)
        actual = stationary_q(client)
        error = max(abs(a-b) for a, b in zip(actual, target))
        report["steps"].append({"phase": label, "segment": number,
                                "segments_in_phase": len(waypoints),
                                "before_q14_rad": before,
                                "target_q14_rad": target,
                                "duration_s": segment_duration,
                                "accepted": accepted,
                                "arrival": arrival, "actual_q14_rad": actual,
                                "max_error_rad": error})
        report.pop("inflight_command", None)
        save(output, report)
        if error > .02:
            raise RuntimeError(f"{label}: joint feedback differs from target by {error:.3f} rad")
        if max(abs(a-b) for a, b in zip(actual, before)) < .005 and \
                max(abs(a-b) for a, b in zip(target, before)) > .025:
            raise RuntimeError(f"{label}: joint feedback did not show commanded motion")
    return stationary_q(client)


def set_grippers(client, targets, profile, report, output, label):
    for side, opening in targets.items():
        report["inflight_command"] = {"type": "gripper", "after": label,
                                      "side": side, "target_opening": opening}
        save(output, report)
        result = gripper_opening(client, side, opening, speed=profile["speed"],
                                 force=min(profile["force"], 20.), timeout=10.)
        report["gripper_steps"].append({"after": label, "side": side,
                                         "target_opening": opening, "result": result})
        report.pop("inflight_command", None)
        save(output, report)


def move_head_checked(client, label, target, duration, report, output):
    before = stationary_q(client, include_head=True)
    if max(abs(a-b) for a, b in zip(before[:14], HOME)) > .025:
        raise RuntimeError("head turn requires both arms at the prepare pose")
    duration = max(duration, round(duration*abs(target[1]-before[15])/.5, 6))
    if duration > 30.:
        raise ValueError("head turn duration exceeds 30 seconds")
    report["inflight_command"] = {"type": "MoveH", "phase": label,
                                  "target_head_rad": target, "duration_s": duration}
    save(output, report)
    accepted = client.moveh(target, duration)
    started = time.monotonic()
    deadline = started + max(duration+15., client.robot["arrival_timeout_s"])
    stable = 0
    while time.monotonic() < deadline:
        state = client.joint_state()
        actual = map_state(state, include_head=True, names=client.joint_names)
        error = max(abs(a-b) for a, b in zip(actual[14:], target))
        elapsed = time.monotonic()-started
        stable = stable+1 if elapsed >= duration and error <= .02 else 0
        if stable >= 3:
            if max(abs(a-b) for a, b in zip(actual[:14], HOME)) > .025:
                raise RuntimeError("arms changed unexpectedly during head turn")
            report["head_steps"].append({"phase": label, "target_head_rad": target,
                                         "actual_head_rad": actual[14:],
                                         "accepted": accepted, "duration_s": duration,
                                         "max_error_rad": error})
            report.pop("inflight_command", None)
            save(output, report)
            return actual[14:]
        time.sleep(.05)
    raise TimeoutError(f"{label}: head arrival not confirmed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", required=True, type=Path)
    parser.add_argument("--course-code", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--send", action="store_true")
    parser.add_argument("--site-checked", action="store_true",
                        help="operator checked full physical path, including both wrist cameras")
    parser.add_argument("--arm-only", action="store_true",
                        help="omit gripper events; otherwise verified enabled LimX grippers are required")
    parser.add_argument("--duration", type=float, default=5.)
    parser.add_argument("--speed-factor", type=int, choices=(1, 4), default=1,
                        help="4 sets each 5 s movement segment to 1.25 s for this script only")
    parser.add_argument("--flow", action="store_true",
                        help="fewer MoveJ endpoints on the same checked joint path; still stops at each endpoint")
    parser.add_argument("--head-yaw-amplitude", type=float, default=.6)
    parser.add_argument("--heart-hold", type=float, default=2.)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("--output already exists; choose a fresh report path")
    if args.send and not args.site_checked:
        parser.error("--send requires --site-checked after physical camera/path inspection")
    if not math.isfinite(args.heart_hold) or not 0. <= args.heart_hold <= 10.:
        parser.error("--heart-hold must be 0..10 seconds")
    if not math.isfinite(args.head_yaw_amplitude) or not 0. < args.head_yaw_amplitude <= .75:
        parser.error("--head-yaw-amplitude must be in (0, 0.75] rad")
    if not (args.course_code / "choreography_candidate.py").is_file():
        parser.error("--course-code must point to the updated Course_Code directory")
    robot = load_robot_profile(args.robot)
    if args.send and not args.arm_only and not robot["gripper"].get("enabled"):
        parser.error("gripper.enabled=false; verify installed LimX grippers and enable the profile first")
    profile_min_duration = robot["min_duration"]
    if not profile_min_duration <= args.duration <= 30.:
        parser.error(f"--duration must be {profile_min_duration}..30 seconds before speed scaling")
    motion_duration = args.duration/args.speed_factor
    if motion_duration < 1.25:
        parser.error("fast motion segments must be at least 1.25 seconds")
    if args.speed_factor == 4:
        # Narrow, explicit override for this previously checked choreography.
        # Other scripts keep the profile's >=5 s hardware gate.
        robot = dict(robot, min_duration=motion_duration)
    base_step_rad = min(.09, robot["max_joint_delta"]-.01)
    if base_step_rad <= .01:
        parser.error("profile max_segment_delta_rad too small for this sequence")
    step_rad = min(.27, 3*base_step_rad) if args.flow else base_step_rad
    arm_duration = round(motion_duration*step_rad/base_step_rad, 6)
    if arm_duration > 30.:
        parser.error("flow MoveJ duration exceeds 30 seconds")

    candidate, chains, endpoints, camera_check, clearance, wrist_fk = \
        load_candidate(args.course_code)
    client = SimClient(robot=robot)
    client.confirm = lambda message: print(message, flush=True)
    report = {"mode": "hardware_live" if args.send else "read_only_plan",
              "status": "in_progress", "robot_profile": str(args.robot),
              "site_checked_by_operator": args.site_checked,
              "physical_motion_verified": False,
              "camera_geometry_verified": False,
              "camera_proxy_radius_m": candidate["camera_proxy_radius_m"],
              "required_camera_clearance_m": clearance,
              "camera_proxy_minimum_offline_m": candidate["minimum_camera_proxy_clearance_m"],
              "course_model_frame_calibrated_to_sdk": False,
              "fk_points": {}, "fk_observations": {}, "ik_points": {},
              "steps": [], "gripper_steps": [],
              "arm_only": args.arm_only, "duration_s": motion_duration,
              "profile_min_duration_s": profile_min_duration,
              "speed_factor": args.speed_factor, "head_steps": [],
              "flow_enabled": args.flow, "arm_duration_s": arm_duration,
              "profile_max_segment_delta_rad": robot["max_joint_delta"],
              "step_limit_rad": step_rad, "planned_phase_endpoints_q14_rad":
              {name: endpoints[name] for name in PHASES}}
    try:
        with client:
            initial16 = stationary_q(client, include_head=True)
            start, initial_head = initial16[:14], initial16[14:]
            client.arm_target(start)
            if max(abs(a-b) for a, b in zip(start, HOME)) > .05:
                raise ValueError("start differs from official prepare pose by >0.05 rad")
            initial_pose = client.move_pose()
            if abs(initial_head[1]) > .3:
                raise ValueError("head yaw route exceeds safe centered-start window; "
                                 "start with head yaw within 0.3 rad of zero")
            head_targets = ([initial_head[0], args.head_yaw_amplitude],
                            [initial_head[0], -args.head_yaw_amplitude],
                            initial_head)
            if any(not -.78 <= target[0] <= 1.04 or not -1.57 <= target[1] <= 1.57
                   for target in head_targets):
                raise ValueError("head yaw route exceeds documented MoveH limits")
            initial_grippers = (read_gripper_openings(client) if not args.arm_only
                                and robot["gripper"].get("enabled") else None)
            report["initial_q14_rad"] = start
            report["initial_head_rad"] = initial_head
            report["planned_head_targets_rad"] = head_targets
            report["initial_sdk_pose_xyz_wxyz"] = initial_pose
            report["initial_gripper_openings"] = initial_grippers
            # Validate every planned joint path before the first hardware command.
            planned = start
            segment_count = 0
            for phase in PHASES:
                segment_count += len(checked_waypoints(planned, endpoints[phase],
                                     step_rad, client, chains, camera_check,
                                     clearance))
                planned = endpoints[phase]
            report["planned_movej_segments"] = segment_count
            save(args.output, report)
            if not args.send:
                report["status"] = "read_only_plan"
            else:
                for phase in PHASES:
                    actual = move_phase(client, phase, endpoints[phase], arm_duration,
                                        step_rad, chains, camera_check, clearance,
                                        report, args.output,
                                        allow_large_move=args.flow)
                    if phase == "left_raise":
                        report["fk_points"]["FK1"] = actual
                        report["fk_observations"]["FK1"] = \
                            fk_observation(client, chains, wrist_fk)
                        if not args.arm_only:
                            set_grippers(client, {"left": 80.}, robot["gripper"],
                                         report, args.output, phase)
                    elif phase == "left_wave_down_1" and not args.arm_only:
                        set_grippers(client, {"left": 30.}, robot["gripper"],
                                     report, args.output, phase)
                    elif phase == "left_wave_up_1" and not args.arm_only:
                        set_grippers(client, {"left": 80.}, robot["gripper"],
                                     report, args.output, phase)
                    elif phase == "chest_heart_candidate":
                        report["fk_points"]["FK2"] = actual
                        report["fk_observations"]["FK2"] = \
                            fk_observation(client, chains, wrist_fk)
                        if not args.arm_only:
                            set_grippers(client, {"left": 30., "right": 30.},
                                         robot["gripper"], report, args.output, phase)
                        time.sleep(args.heart_hold)
                        if not args.arm_only:
                            set_grippers(client, {"left": 80., "right": 80.},
                                         robot["gripper"], report, args.output,
                                         "chest_heart_display")
                    elif phase == "reset_after_heart":
                        for label, target in zip(("head_left", "head_right", "head_restore"),
                                                 head_targets):
                            move_head_checked(client, label, target, motion_duration,
                                              report, args.output)
                    elif phase in ("left_forward_right_back", "left_back_right_forward"):
                        name = "IK1" if phase == "left_forward_right_back" else "IK2"
                        report["inflight_command"] = {"type": "MoveP", "point": name,
                                                      "sdk_delta_xyz_m": IK_DELTAS[name]}
                        save(args.output, report)
                        result = move_both_pose_offset(client, IK_DELTAS[name],
                                                       motion_duration, corrections=2)
                        after_ik_q = stationary_q(client)
                        minimum = min(camera_check(chains, after_ik_q).values())
                        report["ik_points"][name] = {"controller_ik": result,
                                                      "actual_q14_rad": after_ik_q,
                                                      "camera_proxy_clearance_m": minimum}
                        report.pop("inflight_command", None)
                        save(args.output, report)
                        if minimum < clearance:
                            raise RuntimeError(f"{name}: camera proxy clearance below 5 cm")
                final = stationary_q(client)
                final_head = stationary_q(client, include_head=True)[14:]
                final_pose = client.move_pose()
                q_error = max(abs(a-b) for a, b in zip(final, start))
                pos_error, angle_error = pose_errors(initial_pose, final_pose)
                report["final_q14_rad"] = final
                report["final_head_rad"] = final_head
                report["final_sdk_pose_xyz_wxyz"] = final_pose
                report["reset_error"] = {"max_joint_rad": q_error,
                                         "max_head_rad": max(abs(a-b) for a, b in
                                                             zip(final_head, initial_head)),
                                         "max_position_m": pos_error,
                                         "max_orientation_rad": angle_error}
                if q_error > .02 or report["reset_error"]["max_head_rad"] > .02 or \
                        pos_error > .015 or angle_error > .05:
                    raise RuntimeError("final reset not confirmed by SDK feedback")
                if initial_grippers is not None:
                    set_grippers(client, initial_grippers, robot["gripper"],
                                 report, args.output, "restore_initial_openings")
                report["status"] = "four_points_and_reset_feedback_confirmed"
    except KeyboardInterrupt:
        report.update(status="interrupted", error="Ctrl+C stopped future commands; "
                      "an already accepted segment may continue, and no automatic reset was sent")
        print("Ctrl+C: no further waypoints will be sent. The current segment may still move; "
              "use the physical stop/remote if immediate stopping is required.", flush=True)
    except Exception as exc:
        report.update(status="stopped", error=f"{type(exc).__name__}: {exc}")
    finally:
        report["notifications"] = client.notifications
        save(args.output, report)
        if args.send:
            client.save_events(args.output.with_suffix(".events.json"))
    print(json.dumps({"status": report["status"], "steps": len(report["steps"]),
                      "output": str(args.output), "error": report.get("error")},
                     ensure_ascii=False))
    return int(report["status"] in ("stopped", "interrupted"))


if __name__ == "__main__":
    raise SystemExit(main())

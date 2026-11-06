"""Replay the offline choreography in the course MuJoCo model, never on hardware."""
from __future__ import annotations

import argparse
import time

from choreography_candidate import MODEL, build_candidate, load_chains


JOINT_NAMES = [
    f"{part}_{side}_Joint"
    for side in ("L", "R")
    for part in ("proximal_pitch", "proximal_roll", "proximal_yaw", "elbow",
                 "wrist_yaw", "wrist_pitch", "wrist_roll")
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fps", type=float, default=8.)
    parser.add_argument("--repeats", type=int, default=1)
    args = parser.parse_args()
    if not 1 <= args.fps <= 30 or not 1 <= args.repeats <= 10:
        parser.error("--fps must be 1..30 and --repeats must be 1..10")

    import mujoco
    import mujoco.viewer

    report = build_candidate(load_chains())
    if not report["camera_proxy_clearance_passed"]:
        raise RuntimeError("candidate fails the assumed camera clearance check")
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    addresses = []
    for name in JOINT_NAMES:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"model lacks {name}")
        addresses.append(model.jnt_qposadr[joint_id])

    print("Course-model replay only. Wrist cameras and grippers are absent from this MJCF.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        for _ in range(args.repeats):
            for frame in report["frames"]:
                if not viewer.is_running():
                    return
                with viewer.lock():
                    for address, angle in zip(addresses, frame["q14_rad"]):
                        data.qpos[address] = angle
                    mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(1. / args.fps)
        print("Replay finished at the initial arm pose. Close the viewer to exit.")
        while viewer.is_running():
            time.sleep(.1)


if __name__ == "__main__":
    main()

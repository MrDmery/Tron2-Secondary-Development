"""Checks for the offline choreography candidate, not hardware safety tests."""
import unittest

import numpy as np

from choreography_candidate import (HOME, JOINT_STEP_RAD, REQUIRED_CLEARANCE_M,
                                    build_candidate, load_chains)


class ChoreographyCandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = build_candidate(load_chains())

    def test_four_fk_ik_markers_and_gripper_cues(self):
        self.assertEqual(set(self.report["milestones"]), {"FK1", "FK2", "IK1", "IK2"})
        self.assertTrue(any(event.get("left_opening") is not None
                            for event in self.report["gripper_events"]))
        self.assertTrue(any(event.get("right_opening") is not None
                            for event in self.report["gripper_events"]))
        for name in ("IK1", "IK2"):
            for side in ("left", "right"):
                self.assertLess(self.report["milestones"][name][side]["solver_error_m"], 1e-4)

    def test_right_arm_stays_at_home_during_left_raise_and_wave(self):
        for frame in self.report["frames"]:
            if frame["phase"].startswith("left_raise") or frame["phase"].startswith("left_wave"):
                np.testing.assert_allclose(frame["q14_rad"][7:], HOME[7:], atol=1e-12)

    def test_interpolation_reset_and_proxy_clearance(self):
        frames = self.report["frames"]
        self.assertFalse(self.report["hardware_motion_enabled"])
        self.assertTrue(self.report["camera_proxy_clearance_passed"])
        self.assertGreaterEqual(self.report["minimum_camera_proxy_clearance_m"],
                                REQUIRED_CLEARANCE_M)
        np.testing.assert_allclose(frames[-1]["q14_rad"], HOME, atol=1e-12)
        for before, after in zip(frames, frames[1:]):
            self.assertLessEqual(np.max(np.abs(np.array(after["q14_rad"])
                                                - np.array(before["q14_rad"]))),
                                 JOINT_STEP_RAD+1e-12)


if __name__ == "__main__":
    unittest.main()

"""Mock checks for the hardware choreography; no physical safety claim."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from client import SimClient
import choreography_live as exercise
from test_contract import MockController


class ChoreographyLiveTests(unittest.TestCase):
    def setUp(self):
        self.mock = MockController()
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.profile = self.root / "robot.json"
        settings = json.loads(Path(__file__).with_name("robot_profile.example.json").read_text())
        settings.update(url=f"ws://127.0.0.1:{self.mock.port}", accid="TEST_TRON2",
                        robot_model="TRON2 mock", firmware_version="mock",
                        mode_confirmed=True, state_order="left7_right7_head2")
        self.profile.write_text(json.dumps(settings))
        self.course = Path(__file__).resolve().parents[2] / "Course_Code"

    def tearDown(self):
        self.mock.close()
        self.temp.cleanup()

    def run_program(self, name, send=False, gripper=False, checked=False,
                    speed_factor=1, flow=False):
        output = self.root / f"{name}.json"
        if gripper:
            settings = json.loads(self.profile.read_text())
            settings["gripper"]["enabled"] = True
            self.profile.write_text(json.dumps(settings))
        argv = ["choreography_live.py", "--robot", str(self.profile),
                "--course-code", str(self.course), "--output", str(output),
                "--duration", "5", "--speed-factor", str(speed_factor),
                "--heart-hold", "0"]
        if send:
            argv.append("--send")
        if checked:
            argv.append("--site-checked")
        if flow:
            argv.append("--flow")
        with patch.object(sys, "argv", argv), \
             patch.object(SimClient, "wait_joints", return_value={"status": "arrived"}), \
             patch.object(exercise, "move_both_pose_offset",
                          side_effect=self.mock_ik), \
             patch("builtins.input", side_effect=AssertionError("unexpected prompt")):
            code = exercise.main()
        return code, json.loads(output.read_text())

    def mock_ik(self, client, deltas, duration, corrections=2):
        pose = client.move_pose()
        target = pose.copy()
        for index, delta in zip((0, 7), deltas):
            target[index:index+3] = [a+b for a, b in zip(target[index:index+3], delta)]
        client.movep(target[:7], target[7:], duration)
        return {"initial": pose, "target": target, "status": "arrived"}

    def test_read_only_sends_no_motion(self):
        code, report = self.run_program("plan")
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "read_only_plan")
        self.assertFalse(any(call["title"] in ("request_movej", "request_movep",
                                                 "request_set_limx_2fclaw_cmd")
                             for call in self.mock.calls))

    def test_send_requires_site_check_and_enabled_grippers(self):
        argv = ["choreography_live.py", "--robot", str(self.profile),
                "--course-code", str(self.course),
                "--output", str(self.root / "blocked.json"), "--send"]
        with patch.object(sys, "argv", argv), self.assertRaises(SystemExit):
            exercise.main()
        argv.append("--site-checked")
        with patch.object(sys, "argv", argv), self.assertRaises(SystemExit):
            exercise.main()
        self.assertFalse(any(call["title"] == "request_movej" for call in self.mock.calls))

    def test_automatic_sequence_grippers_ik_and_final_reset(self):
        start = self.mock.q[:14].copy()
        code, report = self.run_program("live", send=True, gripper=True,
                                        checked=True, speed_factor=4)
        self.assertEqual(code, 0, report.get("error"))
        self.assertEqual(report["status"], "four_points_and_reset_feedback_confirmed")
        self.assertEqual(set(report["fk_points"]), {"FK1", "FK2"})
        self.assertEqual(set(report["fk_observations"]), {"FK1", "FK2"})
        for point in report["fk_observations"].values():
            self.assertEqual(len(point["q14_rad"]), 14)
            self.assertEqual(len(point["sdk_pose_xyz_wxyz"]), 14)
            self.assertEqual(len(point["course_wrist_fk_xyz_m"]["left"]), 3)
            self.assertEqual(len(point["course_wrist_fk_xyz_m"]["right"]), 3)
        self.assertEqual(set(report["ik_points"]), {"IK1", "IK2"})
        self.assertEqual(report["duration_s"], 1.25)
        self.assertEqual(report["speed_factor"], 4)
        self.assertEqual([step["phase"] for step in report["head_steps"]],
                         ["head_left", "head_right", "head_restore"])
        self.assertAlmostEqual(report["planned_head_targets_rad"][0][1], .6)
        self.assertAlmostEqual(report["planned_head_targets_rad"][1][1], -.6)
        self.assertEqual([step["duration_s"] for step in report["head_steps"]],
                         [1.75, 3., 1.25])
        self.assertEqual(report["final_head_rad"], report["initial_head_rad"])
        self.assertEqual(self.mock.q[:14], start)
        movej = [call["data"]["joint"] for call in self.mock.calls
                 if call["title"] == "request_movej"]
        self.assertTrue(movej)
        self.assertTrue(all(call["data"]["time"] == 1.25 for call in self.mock.calls
                            if call["title"] in ("request_movej", "request_movep")))
        for step in report["steps"]:
            if step["phase"].startswith(("left_raise", "left_wave")):
                self.assertEqual(step["target_q14_rad"][7:], start[7:])
        self.assertEqual(len([call for call in self.mock.calls
                              if call["title"] == "request_movep"]), 2)
        self.assertEqual(len([call for call in self.mock.calls
                              if call["title"] == "request_moveh"]), 3)
        self.assertGreaterEqual(len(report["gripper_steps"]), 7)
        self.assertEqual(self.mock.opening, {"left": 80., "right": 80.})

    def test_flow_reduces_stops_preserves_checkpoints_and_reset(self):
        start = self.mock.q[:14].copy()
        code, report = self.run_program("flow", send=True, gripper=True,
                                        checked=True, speed_factor=4, flow=True)
        self.assertEqual(code, 0, report.get("error"))
        self.assertTrue(report["flow_enabled"])
        self.assertEqual(report["arm_duration_s"], 3.75)
        self.assertLess(report["planned_movej_segments"], 40)
        self.assertEqual(set(report["fk_points"]), {"FK1", "FK2"})
        self.assertEqual(set(report["ik_points"]), {"IK1", "IK2"})
        self.assertEqual(len(report["head_steps"]), 3)
        self.assertEqual(self.mock.q[:14], start)
        arm_calls = [call for call in self.mock.calls if call["title"] == "request_movej"]
        durations = [call["data"]["time"] for call in arm_calls]
        self.assertTrue(all(1.25 <= value <= 3.75 for value in durations))
        self.assertTrue(any(value > 1.25 for value in durations))
        self.assertEqual(len(arm_calls), len(report["steps"]))
        self.assertEqual(durations, [step["duration_s"] for step in report["steps"]])
        self.assertTrue(all(max(abs(a-b) for a, b in zip(step["target_q14_rad"],
                                                        step["before_q14_rad"]))
                            <= .27+1e-9 for step in report["steps"]))

    def test_frozen_feedback_stops_without_running_later_phases(self):
        self.mock.freeze = True
        code, report = self.run_program("frozen", send=True, gripper=True, checked=True)
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "stopped")
        self.assertEqual(len([call for call in self.mock.calls
                              if call["title"] == "request_movej"]), 1)
        self.assertEqual(next(call["data"]["time"] for call in self.mock.calls
                              if call["title"] == "request_movej"), 5.)
        self.assertFalse(any(call["title"] == "request_set_limx_2fclaw_cmd"
                             for call in self.mock.calls))

    def test_head_limit_rejected_before_motion(self):
        self.mock.q[15] = 1.5
        code, report = self.run_program("head_limit", send=True, gripper=True,
                                        checked=True, speed_factor=4)
        self.assertEqual(code, 1)
        self.assertIn("head yaw route exceeds", report["error"])
        self.assertFalse(any(call["title"] in ("request_movej", "request_moveh")
                             for call in self.mock.calls))

    def test_ctrl_c_during_first_segment_sends_no_further_motion(self):
        output = self.root / "interrupted.json"
        settings = json.loads(self.profile.read_text())
        settings["gripper"]["enabled"] = True
        self.profile.write_text(json.dumps(settings))
        argv = ["choreography_live.py", "--robot", str(self.profile),
                "--course-code", str(self.course), "--output", str(output),
                "--send", "--site-checked", "--heart-hold", "0"]
        with patch.object(sys, "argv", argv), \
             patch.object(SimClient, "wait_joints", side_effect=KeyboardInterrupt), \
             patch("builtins.input", side_effect=AssertionError("unexpected prompt")):
            self.assertEqual(exercise.main(), 1)
        report = json.loads(output.read_text())
        self.assertEqual(report["status"], "interrupted")
        self.assertEqual(report["inflight_command"]["type"], "MoveJ")
        self.assertEqual(len([call for call in self.mock.calls
                              if call["title"] == "request_movej"]), 1)
        self.assertFalse(any(call["title"] in ("request_movep",
                                                 "request_set_limx_2fclaw_cmd")
                             for call in self.mock.calls))


if __name__ == "__main__":
    unittest.main()

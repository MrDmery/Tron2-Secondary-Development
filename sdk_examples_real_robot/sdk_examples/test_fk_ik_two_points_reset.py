"""Mock protocol checks for the bounded four-point hardware sequence."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from client import SimClient
import fk_ik_two_points_reset as exercise
from test_contract import MockController


class FourPointSequenceTests(unittest.TestCase):
    def setUp(self):
        self.mock = MockController()
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        profile = json.loads(Path(__file__).with_name("robot_profile.example.json").read_text())
        profile.update(url=f"ws://127.0.0.1:{self.mock.port}", accid="TEST_TRON2",
                       robot_model="TRON2 mock", firmware_version="mock",
                       mode_confirmed=True, state_order="left7_right7_head2")
        (self.root / "robot.json").write_text(json.dumps(profile))
        self.course_code = Path(__file__).resolve().parents[2] / "Course_Code"

    def tearDown(self):
        self.mock.close()
        self.temp.cleanup()

    def run_sequence(self, label, pattern="simple", send=True, with_gripper=False,
                     enable_gripper=False):
        output = self.root / f"{label}.json"
        if enable_gripper:
            profile_path = self.root / "robot.json"
            profile = json.loads(profile_path.read_text())
            profile["gripper"]["enabled"] = True
            profile_path.write_text(json.dumps(profile))
        argv = ["fk_ik_two_points_reset.py", "--robot", str(self.root / "robot.json"),
                "--course-code", str(self.course_code), "--pattern", pattern,
                "--duration", "5",
                "--output", str(output)]
        if send:
            argv.append("--send")
        if with_gripper:
            argv.append("--with-gripper")
        with patch.object(sys, "argv", argv), \
             patch.object(SimClient, "wait_joints", return_value={"status": "arrived"}), \
             patch.object(exercise, "fk_matrix", return_value=[[1., 0., 0., 0.],
                                                                [0., 1., 0., 0.],
                                                                [0., 0., 1., 0.],
                                                                [0., 0., 0., 1.]]), \
             patch.object(exercise, "wait_pose_settled", side_effect=lambda client, *_: (client.move_pose(), [])), \
             patch("builtins.input", side_effect=AssertionError("unexpected operator prompt")):
            code = exercise.main()
        return code, json.loads(output.read_text())

    def test_two_arm_motion_and_reset_without_prompts(self):
        code, report = self.run_sequence("complete")
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "four_points_and_reset_feedback_confirmed")
        self.assertEqual(len(report["steps"]), 12)
        commands = [call for call in self.mock.calls if call["title"] == "request_movej"]
        self.assertEqual(len(commands), 10)
        for number, call in enumerate(commands[:4], 1):
            q = call["data"]["joint"]
            self.assertAlmostEqual(q[3], self.mock.q[3] + number * .08)
            self.assertAlmostEqual(q[10], self.mock.q[10] + number * .08)
        self.assertAlmostEqual(self.mock.q[3], report["initial"]["q14_rad"][3])
        self.assertAlmostEqual(self.mock.q[10], report["initial"]["q14_rad"][10])

    def test_frozen_feedback_stops_after_first_command(self):
        self.mock.freeze = True
        code, report = self.run_sequence("frozen")
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "stopped")
        self.assertEqual(len([call for call in self.mock.calls if call["title"] == "request_movej"]), 1)

    def test_extended_start_is_rejected_without_motion(self):
        self.mock.q[3] += .32
        self.mock.q[10] += .32
        code, report = self.run_sequence("extended")
        self.assertEqual(code, 1)
        self.assertIn("run prepare first", report["error"])
        self.assertFalse(any(call["title"] in ("request_movej", "request_movep")
                             for call in self.mock.calls))

    def test_reset_includes_joint_change_caused_by_ik(self):
        original_movep = SimClient.movep

        def move_ik_and_change_joints(client, left, right, duration):
            accepted = original_movep(client, left, right, duration)
            self.mock.q[3] += .012
            self.mock.q[10] += .012
            return accepted

        with patch.object(SimClient, "movep", move_ik_and_change_joints):
            code, report = self.run_sequence("ik_joint_change")
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "four_points_and_reset_feedback_confirmed")
        self.assertEqual(len(report["steps"]), 13)
        self.assertEqual(report["reset_error"]["max_joint_rad"], 0.)

    def test_varied_route_has_two_distinct_fk_and_ik_points_then_resets(self):
        start_pose = self.mock.pose.copy()
        original_movej = SimClient.movej

        def movej_with_mock_fk_pose(client, target, duration, large_move=False):
            accepted = original_movej(client, target, duration, large_move)
            if all(abs(a-b) < 1e-9 for a, b in zip(target, report_start_q)):
                self.mock.pose = start_pose.copy()
            return accepted

        report_start_q = self.mock.q[:14].copy()
        with patch.object(SimClient, "movej", movej_with_mock_fk_pose):
            code, report = self.run_sequence("varied", pattern="varied")
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "four_points_and_reset_feedback_confirmed")
        movej = [call["data"]["joint"] for call in self.mock.calls
                 if call["title"] == "request_movej"]
        self.assertEqual([(round(q[3]-report_start_q[3], 2),
                           round(q[10]-report_start_q[10], 2)) for q in movej[:4]],
                         [(0.08, 0.08), (0.16, 0.16), (0.24, 0.08), (0.32, 0.0)])
        movep = [call["data"]["pos"] for call in self.mock.calls
                 if call["title"] == "request_movep"]
        self.assertEqual(len(movep), 2)
        self.assertAlmostEqual(movep[0][0]-start_pose[0], .008)
        self.assertAlmostEqual(movep[0][1]-start_pose[1], .004)
        self.assertAlmostEqual(movep[0][8]-start_pose[8], -.004)
        self.assertAlmostEqual(movep[1][0]-start_pose[0], .016)
        self.assertAlmostEqual(movep[1][1]-start_pose[1], 0.)
        self.assertAlmostEqual(movep[1][8]-start_pose[8], 0.)
        self.assertEqual(self.mock.q[:14], report_start_q)
        self.assertEqual(self.mock.pose, start_pose)

    def test_varied_read_only_plan_sends_no_motion(self):
        code, report = self.run_sequence("varied_plan", pattern="varied", send=False)
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "read_only_plan")
        self.assertEqual(report["pattern"], "varied")
        self.assertFalse(any(call["title"] in ("request_movej", "request_movep")
                             for call in self.mock.calls))

    def test_wide_route_extends_both_elbows_and_resets(self):
        start_q = self.mock.q[:14].copy()
        start_pose = self.mock.pose.copy()
        original_movej = SimClient.movej

        def movej_with_mock_fk_pose(client, target, duration, large_move=False):
            accepted = original_movej(client, target, duration, large_move)
            if all(abs(a-b) < 1e-9 for a, b in zip(target, start_q)):
                self.mock.pose = start_pose.copy()
            return accepted

        with patch.object(SimClient, "movej", movej_with_mock_fk_pose):
            code, report = self.run_sequence("wide", pattern="wide")
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "four_points_and_reset_feedback_confirmed")
        self.assertEqual(report["fk_point_segments"], [3, 5])
        self.assertEqual(set(report["fk_targets_q14_rad"]), {"FK1", "FK2"})
        self.assertEqual([step["label"] for step in report["steps"] if step["label"].startswith("IK")],
                         ["IK1", "IK2"])
        self.assertEqual(report["reset_max_segment_rad"], .09)
        movej = [call["data"] for call in self.mock.calls
                 if call["title"] == "request_movej"]
        self.assertEqual([(round(cmd["joint"][3]-start_q[3], 2),
                           round(cmd["joint"][10]-start_q[10], 2)) for cmd in movej[:5]],
                         [(.09, .09), (.18, .18), (.27, .27), (.36, .18), (.45, .09)])
        self.assertTrue(all(cmd["time"] == 5 for cmd in movej))
        self.assertEqual(self.mock.q[:14], start_q)
        self.assertEqual(self.mock.pose, start_pose)

    def test_wide_read_only_plan_sends_no_motion(self):
        code, report = self.run_sequence("wide_plan", pattern="wide", send=False)
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "read_only_plan")
        self.assertEqual(report["fk_point_segments"], [3, 5])
        self.assertFalse(any(call["title"] in ("request_movej", "request_movep")
                             for call in self.mock.calls))

    def test_sway_moves_both_shoulders_then_returns_directly(self):
        start_q = self.mock.q[:14].copy()
        start_pose = self.mock.pose.copy()
        original_movej = SimClient.movej

        def movej_with_mock_fk_pose(client, target, duration, large_move=False):
            accepted = original_movej(client, target, duration, large_move)
            if all(abs(a-b) < 1e-9 for a, b in zip(target, start_q)):
                self.mock.pose = start_pose.copy()
            return accepted

        with patch.object(SimClient, "movej", movej_with_mock_fk_pose):
            code, report = self.run_sequence("sway", pattern="sway")
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "four_points_and_reset_feedback_confirmed")
        self.assertTrue(report["return_direct"])
        self.assertEqual(report["fk_point_segments"], [2, 5])
        q_targets = [call["data"]["joint"] for call in self.mock.calls
                     if call["title"] == "request_movej"]
        self.assertEqual([round(q[2]-start_q[2], 2) for q in q_targets[:5]],
                         [.06, .12, .04, -.04, -.12])
        self.assertEqual([round(q[9]-start_q[9], 2) for q in q_targets[:5]],
                         [.06, .12, .04, -.04, -.12])
        self.assertEqual([step["label"] for step in report["steps"] if step["label"].startswith("IK")],
                         ["IK1", "IK2"])
        self.assertTrue(all("return toward initial" in step["label"]
                            for step in report["steps"] if step["label"].startswith("return")))
        self.assertEqual(self.mock.q[:14], start_q)

    def test_sway_grippers_close_open_and_restore_when_enabled(self):
        start_q = self.mock.q[:14].copy()
        start_pose = self.mock.pose.copy()
        original_movej = SimClient.movej

        def movej_with_mock_fk_pose(client, target, duration, large_move=False):
            accepted = original_movej(client, target, duration, large_move)
            if all(abs(a-b) < 1e-9 for a, b in zip(target, start_q)):
                self.mock.pose = start_pose.copy()
            return accepted

        with patch.object(SimClient, "movej", movej_with_mock_fk_pose):
            code, report = self.run_sequence("sway_gripper", pattern="sway",
                                             with_gripper=True, enable_gripper=True)
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "four_points_and_reset_feedback_confirmed")
        self.assertEqual(len(report["gripper_steps"]), 6)
        self.assertEqual([step["target_opening"] for step in report["gripper_steps"]],
                         [30., 30., 80., 80., 80., 80.])
        self.assertEqual(self.mock.opening, {"left": 80., "right": 80.})

    def test_gripper_option_requires_enabled_profile_before_connection(self):
        argv = ["fk_ik_two_points_reset.py", "--robot", str(self.root / "robot.json"),
                "--course-code", str(self.course_code), "--pattern", "sway",
                "--with-gripper", "--send", "--output", str(self.root / "disabled.json")]
        with patch.object(sys, "argv", argv), self.assertRaises(SystemExit):
            exercise.main()
        self.assertFalse(any(call["title"] in ("request_movej", "request_movep",
                                                 "request_set_limx_2fclaw_cmd")
                             for call in self.mock.calls))

    def test_sway_gripper_read_only_plan_queries_without_motion(self):
        code, report = self.run_sequence("sway_gripper_plan", pattern="sway",
                                         send=False, with_gripper=True,
                                         enable_gripper=True)
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "read_only_plan")
        self.assertEqual(report["initial_gripper_openings"], {"left": 80., "right": 80.})
        self.assertFalse(any(call["title"] in ("request_movej", "request_movep",
                                                 "request_set_limx_2fclaw_cmd")
                             for call in self.mock.calls))

    def test_gripper_state_failure_stops_before_arm_motion(self):
        with patch.object(exercise, "read_gripper_openings", side_effect=RuntimeError("no gripper state")):
            code, report = self.run_sequence("sway_no_gripper_state", pattern="sway",
                                             with_gripper=True, enable_gripper=True)
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "stopped")
        self.assertFalse(any(call["title"] in ("request_movej", "request_movep",
                                                 "request_set_limx_2fclaw_cmd")
                             for call in self.mock.calls))

    def test_sway_frozen_arm_does_not_start_gripper_sequence(self):
        self.mock.freeze = True
        code, report = self.run_sequence("sway_frozen", pattern="sway",
                                         with_gripper=True, enable_gripper=True)
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "stopped")
        self.assertEqual(len([call for call in self.mock.calls
                              if call["title"] == "request_movej"]), 1)
        self.assertFalse(any(call["title"] == "request_set_limx_2fclaw_cmd"
                             for call in self.mock.calls))


if __name__ == "__main__":
    unittest.main()

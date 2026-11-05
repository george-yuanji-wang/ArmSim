"""Common joint-state references, reach extrema and durable readable export."""

import csv
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from armsim.catalog import layout_by_number
from optimization.poses import (DEFAULT_POSE_COUNT, REFERENCE_POSE_COUNT,
                                SHOULDER_ORIGIN_M, generate_poses,
                                resolve_joint_positions, write_poses)
from optimization.reach import extended_reach_m


def requirement(**changes):
    values = dict(reach_lower_mm=450., reach_upper_mm=600.)
    values.update(changes)
    return SimpleNamespace(**values)


class PoseTests(unittest.TestCase):
    def test_common_suite_has_1008_states_and_spans_every_joint(self):
        poses = generate_poses(requirement())
        self.assertEqual(len(poses), DEFAULT_POSE_COUNT)
        self.assertEqual(sum(p.pose_kind == "halton_joint_state" for p in poses),
                         DEFAULT_POSE_COUNT-REFERENCE_POSE_COUNT)
        self.assertEqual(len({p.pose_id for p in poses}), len(poses))
        self.assertEqual(sum(p.pose_kind == "fully_extended" for p in poses), 6)
        ordinary = np.array([p.joint_positions_rad for p in poses
                             if p.joint_positions_rad is not None])
        self.assertEqual(len(np.unique(np.round(ordinary, 11), axis=0)), len(ordinary))
        self.assertTrue(np.all(ordinary.min(axis=0) < -.95*np.pi))
        self.assertTrue(np.all(ordinary.max(axis=0) > .95*np.pi))
        self.assertAlmostEqual(sum(p.hold_weight for p in poses), 1.)

    def test_basis_does_not_change_with_reach_bounds_or_fixed_reach(self):
        first = generate_poses(requirement())
        for other in (requirement(reach_lower_mm=500, reach_upper_mm=500),
                      requirement(reach_lower_mm=200, reach_upper_mm=2000)):
            with self.subTest(other=other):
                second = generate_poses(other)
                for a, b in zip(first, second):
                    self.assertEqual((a.pose_kind, a.reference_name, a.extension_direction),
                                     (b.pose_kind, b.reference_name, b.extension_direction))
                    if a.joint_positions_rad is not None:
                        np.testing.assert_array_equal(a.joint_positions_rad, b.joint_positions_rad)

    def test_explicit_references_include_signed_pitch_roll_and_zero(self):
        references = generate_poses(requirement())[:REFERENCE_POSE_COUNT]
        np.testing.assert_array_equal(references[0].joint_positions_rad, np.zeros(6))
        for j in range(6):
            for sign in (-1, 1):
                expected = np.eye(6)[j]*sign*np.pi/2
                self.assertTrue(any(p.joint_positions_rad is not None and
                                    np.array_equal(p.joint_positions_rad, expected)
                                    for p in references))

    def test_extension_references_achieve_exact_radius_in_six_world_directions(self):
        directions = {"+X": [1, 0, 0], "-X": [-1, 0, 0], "+Y": [0, 1, 0],
                      "-Y": [0, -1, 0], "+Z": [0, 0, 1], "-Z": [0, 0, -1]}
        for number in (7, 8, 9):
            layout = layout_by_number(number)
            candidate = SimpleNamespace(layout_number=number,
                                        lengths_m=(.32, layout.l2_length, layout.l3_length))
            model = mujoco.MjModel.from_xml_path(str(layout.path))
            data = mujoco.MjData(model)
            for pose in generate_poses(requirement()):
                if not pose.extension_direction:
                    continue
                with self.subTest(layout=number, direction=pose.extension_direction):
                    data.qpos[:] = resolve_joint_positions(pose, candidate)
                    mujoco.mj_forward(model, data)
                    vector = data.site("tool_frame").xpos-SHOULDER_ORIGIN_M
                    expected = np.array(directions[pose.extension_direction])*extended_reach_m(candidate)
                    np.testing.assert_allclose(vector, expected, atol=1e-10)

    def test_subset_is_deterministic_spread_out_and_export_is_exclusive(self):
        full = generate_poses(requirement())
        subset = generate_poses(requirement(), limit=8)
        self.assertEqual(subset[0].pose_id, full[0].pose_id)
        self.assertEqual(subset[-1].pose_id, full[-1].pose_id)
        self.assertEqual([p.pose_id for p in subset],
                         [p.pose_id for p in generate_poses(requirement(), limit=8)])
        self.assertAlmostEqual(sum(p.hold_weight for p in subset), 1.)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"poses.csv"
            write_poses(path, full)
            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), DEFAULT_POSE_COUNT)
            self.assertEqual(float(rows[0]["q1_rad"]), 0.)
            extension = next(row for row in rows if row["pose_kind"] == "fully_extended")
            self.assertEqual(extension["q3_rad"], "")
            self.assertTrue(extension["extension_direction"])
            self.assertNotIn("radius_mm", rows[0])
            with self.assertRaises(FileExistsError):
                write_poses(path, subset)

    def test_invalid_reach_ranges_and_limit_are_rejected(self):
        for changes in (dict(reach_upper_mm=100), dict(reach_lower_mm=-1),
                        dict(reach_upper_mm=float("nan"))):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                generate_poses(requirement(**changes))
        for limit in (0, -1, True, 1.5):
            with self.subTest(limit=limit), self.assertRaises(ValueError):
                generate_poses(requirement(), limit=limit)


if __name__ == "__main__":
    unittest.main()

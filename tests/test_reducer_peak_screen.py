"""Separate holding and repeated-peak constraints using prescribed load bases."""

from dataclasses import replace
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from optimization.design import Candidate
from optimization.inputs import (DATA_DIR, InputError, Inputs, LinkBounds, Motor,
                                 Reducer, Requirement, STRUCTURAL_BODIES, load_inputs)
from optimization.physics import evaluate_candidate
from optimization.poses import generate_poses


def fixture(*, motor_torque=10., rated=3., repeated_peak=5., input_limit=None):
    motor = Motor("M", "Policy test motor", motor_torque, 6000., None, None, None,
                  .2, 20., 18., 1e-8, capacity_model="rated_continuous")
    reducer = Reducer("G", "Policy test reducer", 30., .1, 25., 20., rated,
                      repeated_peak, input_limit, 6000.)
    links = tuple(LinkBounds(name, .01, 1., .1, .9, .01, .2, 10.)
                  for name in STRUCTURAL_BODIES)
    inputs = Inputs((motor,), (reducer,), (), links)
    candidate = Candidate(8, (.55-.15-np.hypot(.09, .16), .09, .16),
                          (.1,)*6, (.5,)*6, (.1,)*6, (0.,)*4,
                          ("M",)*6, ("G",)*6)
    requirement = Requirement("RATING_FIXTURE", 500., 500., 550., 550.,
        joint_speed_rad_s=.2, joint_accel_rad_s2=6., allowed_layouts=(8,),
        reducer_load_policy="hold_and_repeated_peak")
    return candidate, inputs, requirement


def evaluate_prescribed(candidate, inputs, requirement, *, gravity=1., inertia=.5):
    """J2 has G=gravity+p and dynamic acceleration torque ±6*inertia."""
    calls = 0

    def basis(model, data, q):
        nonlocal calls
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        payload = calls % 2
        calls += 1
        g, matrix, quadratic = np.zeros(6), np.zeros((6, 6)), np.zeros((21, 6))
        g[1] = gravity+payload
        matrix[1, 1] = inertia
        return g, matrix, quadratic

    with patch("optimization.physics.load_basis", side_effect=basis):
        return evaluate_candidate(candidate, inputs, generate_poses(requirement, 1), requirement)


class ReducerPeakPolicyTests(unittest.TestCase):
    def test_acceleration_above_rated_below_repeated_peak_passes(self):
        candidate, inputs, requirement = fixture()
        result = evaluate_prescribed(candidate, inputs, requirement)
        self.assertTrue(result.feasible)
        self.assertAlmostEqual(result.pmax_kg, 1., places=10)
        self.assertAlmostEqual(result.summary["pmax_known_constraints_kg"], 1., places=10)
        self.assertAlmostEqual(result.per_pose_rows[0]["peak_joint_load_j2_Nm_at_required_payload"], 4.5)
        self.assertAlmostEqual(result.per_pose_rows[0]["peak_reducer_utilization_j2_ratio"], .9)
        self.assertEqual(result.summary["reducer_holding_output_limits_Nm"], [3.]*6)
        self.assertEqual(result.summary["reducer_motion_output_limits_Nm"], [5.]*6)
        continuous = evaluate_prescribed(candidate, inputs,
                                         replace(requirement, reducer_load_policy="continuous"))
        self.assertEqual(continuous.pmax_kg, 0.)
        self.assertFalse(continuous.feasible)

    def test_motion_above_repeated_peak_fails(self):
        candidate, inputs, requirement = fixture(repeated_peak=3.5)
        result = evaluate_prescribed(candidate, inputs, requirement)
        self.assertEqual(result.pmax_kg, 0.)
        self.assertEqual(result.summary["pmax_known_constraints_kg"], 0.)
        self.assertFalse(result.feasible)
        self.assertEqual(result.per_pose_rows[0]["limitation"], "reducer_repeated_peak_torque")

    def test_large_peak_does_not_allow_sustained_gravity_above_rated(self):
        candidate, inputs, requirement = fixture(repeated_peak=10.)
        result = evaluate_prescribed(candidate, inputs, requirement, gravity=4., inertia=0.)
        self.assertEqual(result.pmax_kg, 0.)
        self.assertEqual(result.summary["pmax_known_constraints_kg"], 0.)
        self.assertFalse(result.feasible)
        self.assertEqual(result.per_pose_rows[0]["limitation"], "static_holding_torque")
        self.assertEqual(result.per_pose_rows[0]["limiting_accel_corner"], -1)
        self.assertTrue(all(result.per_pose_rows[0][f"limiting_acceleration_j{j}_rad_s2"] == 0
                            for j in range(1, 7)))

    def test_unknown_motor_transient_is_not_fabricated_by_auxiliary_screen(self):
        candidate, inputs, requirement = fixture(motor_torque=.1)
        result = evaluate_prescribed(candidate, inputs, requirement)
        self.assertEqual(result.pmax_kg, 0.)
        self.assertFalse(result.feasible)
        self.assertAlmostEqual(result.summary["pmax_known_constraints_kg"], 1., places=10)
        self.assertTrue(result.summary["known_constraints_passes_required_payload"])
        self.assertFalse(result.summary["motor_transient_rating_verified"])
        self.assertFalse(result.summary["duty_cycle_verified"])
        self.assertIn("not a complete arm payload rating", result.summary["known_constraints_definition"])
        self.assertAlmostEqual(result.per_pose_rows[0]["peak_motor_utilization_j2_ratio"], 1.500018, places=8)

    def test_known_input_limit_caps_repeated_output_torque(self):
        candidate, inputs, requirement = fixture(input_limit=.1)
        result = evaluate_prescribed(candidate, inputs, requirement)
        self.assertEqual(result.summary["reducer_repeated_peak_output_limits_Nm"], [3.]*6)
        self.assertEqual(result.pmax_kg, 0.)
        self.assertEqual(result.summary["pmax_known_constraints_kg"], 0.)

    def test_common_gravity_score_is_distinct_from_motor_capacity_margin(self):
        candidate, inputs, requirement = fixture(motor_torque=.05)
        result = evaluate_prescribed(candidate, inputs, requirement)
        self.assertAlmostEqual(result.jg, .25)
        self.assertAlmostEqual(result.summary["gravity_capacity_score_JG"], 1.)
        np.testing.assert_allclose(result.summary["gravity_mean_square_Nm2_by_joint"],
                                   [0., 2.25, 0., 0., 0., 0.])

    def test_policy_parser_and_validation_keep_legacy_default(self):
        self.assertEqual(Requirement("LEGACY", 500, 500, 550, 550).reducer_load_policy, "continuous")
        with self.assertRaisesRegex(ValueError, "reducer_load_policy"):
            Requirement("INVALID", 500, 500, 550, 550, reducer_load_policy="momentary")
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)
            for name in ("motor.csv", "reducer.csv", "requirement.csv", "link_bounds.csv"):
                shutil.copyfile(DATA_DIR/name, destination/name)
            (destination/"study.json").write_text(json.dumps({
                "reducer_load_policy": "hold_and_repeated_peak"}), encoding="utf-8")
            self.assertEqual(load_inputs(destination, require_catalogs=False).requirements[0].reducer_load_policy,
                             "hold_and_repeated_peak")
            (destination/"study.json").write_text(json.dumps({"reducer_load_policy": "momentary"}),
                                                  encoding="utf-8")
            with self.assertRaisesRegex(InputError, "reducer_load_policy"):
                load_inputs(destination, require_catalogs=False)


if __name__ == "__main__":
    unittest.main()

"""Independent numerical checks of the headless mechanics, not GUI models."""

from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from optimization.design import Candidate
from optimization.inputs import Inputs, LinkBounds, Motor, Reducer, STRUCTURAL_BODIES
from optimization.physics import (build_model, evaluate_candidate, load_basis,
                                  motor_torque_limits, solve_pose, _velocity_features)
from optimization.poses import generate_poses
from optimization.reach import extended_reach_m


def inputs():
    motor = Motor("test_motor", "Synthetic test fixture", 5., 2000., 10., 6000.,
                  6000., .2, 20., 18., 1e-5)
    reducer = Reducer("test_reducer", "Synthetic test fixture", 50., .1, 25.,
                      20., 200., 300., 8., 6000.)
    links = tuple(LinkBounds(name, .05, 1., .2, .8, .02, .1, 10.)
                  for name in STRUCTURAL_BODIES)
    return Inputs((motor,), (reducer,), (), links)


def candidate(layout=1):
    return Candidate(layout, (.4, .2, .2), (.1,)*6, (.5,)*6, (1/12,)*6,
                     (0., 0., 0., 0.), ("test_motor",)*6, ("test_reducer",)*6)


def pose_from_q(model, q, pose_id="fixture"):
    data = mujoco.MjData(model)
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    site = model.site("tool_frame").id
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, data.site_xmat[site])
    return SimpleNamespace(pose_id=pose_id, position_m=data.site_xpos[site].copy(),
                           quaternion_wxyz=quat, hold_weight=1.)


class ConceptualPhysicsTests(unittest.TestCase):
    def setUp(self):
        self.inputs = inputs()

    def test_models_have_six_unlimited_contact_free_joints(self):
        for number in range(1, 10):
            model, _ = build_model(candidate(number), self.inputs)
            self.assertEqual(model.nv, 6)
            self.assertFalse(model.jnt_limited.any())
            self.assertFalse(model.geom_contype.any())
            self.assertFalse(model.geom_conaffinity.any())
            self.assertTrue(np.all(model.dof_armature == 0))

    def test_analytic_ik_matches_forward_kinematics_for_all_families(self):
        rng = np.random.default_rng(14)
        for number in range(1, 10):
            model, _ = build_model(candidate(number), self.inputs)
            data = mujoco.MjData(model)
            for _ in range(15):
                q = rng.uniform(-np.pi, np.pi, 6)
                pose = pose_from_q(model, q)
                found = solve_pose(model, data, pose.position_m, pose.quaternion_wxyz)
                self.assertIsNotNone(found, number)
                np.testing.assert_allclose(data.site("tool_frame").xpos,
                                           pose.position_m, atol=2e-8)

    def test_infeasible_pose_is_not_silently_skipped(self):
        model, _ = build_model(candidate(), self.inputs)
        result = solve_pose(model, mujoco.MjData(model), [10., 0., 0.], [1., 0., 0., 0.])
        self.assertIsNone(result)

    def test_quadratic_basis_reconstructs_arbitrary_mixed_motion(self):
        model, _ = build_model(candidate(8), self.inputs, .4)
        data = mujoco.MjData(model)
        q = np.array([.4, .7, -1.1, .5, .8, -.3])
        gravity, matrix, quadratic = load_basis(model, data, q)
        rng = np.random.default_rng(7)
        for _ in range(10):
            velocity = rng.uniform(-1.5, 1.5, 6)
            acceleration = rng.uniform(-2., 2., 6)
            expected = gravity+matrix@acceleration+_velocity_features(velocity[None, :])[0]@quadratic
            data.qpos[:] = q
            data.qvel[:] = velocity
            data.qacc[:] = acceleration
            mujoco.mj_inverse(model, data)
            np.testing.assert_allclose(data.qfrc_inverse, expected, atol=2e-10)

    def test_payload_load_is_affine_in_mass(self):
        torques = []
        for mass in (0., 1., .37):
            model, _ = build_model(candidate(7), self.inputs, mass)
            data = mujoco.MjData(model)
            data.qpos[:] = [.2, .6, -.8, .4, .7, -.2]
            data.qvel[:] = [.1, .5, -.3, .4, .2, -.1]
            data.qacc[:] = [.4, -.2, .6, .3, -.5, .2]
            mujoco.mj_inverse(model, data)
            torques.append(data.qfrc_inverse.copy())
        np.testing.assert_allclose(torques[2], torques[0]+.37*(torques[1]-torques[0]), atol=1e-10)

    def test_j2_motor_planar_placement_changes_yaw_inertia_not_own_gravity(self):
        reference = candidate(2)
        matrices, gravities = [], []
        for offsets in ((0., 0., 0., 0.), (.005, 0., 0., 0.), (0., .001, 0., 0.)):
            model, _ = build_model(replace(reference, motor_offsets_m=offsets), self.inputs)
            gravity, matrix, _ = load_basis(model, mujoco.MjData(model), [.2, .7, -.9, .3, .6, .4])
            matrices.append(matrix)
            gravities.append(gravity)
        np.testing.assert_allclose(gravities[0], gravities[1], atol=1e-11)
        np.testing.assert_allclose(gravities[0], gravities[2], atol=1e-11)
        self.assertGreater(matrices[1][0, 0], matrices[0][0, 0])
        # Changing mount height changes the lengths/mass of its real support
        # edges, so even a vertical shift may slightly change yaw inertia.
        # Parent-mounted hardware still cannot change the downstream block.
        for matrix in matrices[1:]:
            difference = matrix-matrices[0]
            difference[0, 0] = 0
            np.testing.assert_allclose(difference, 0, atol=1e-11)

    def test_coaxial_layout_ignores_inactive_offset_coordinates(self):
        c = candidate(1)
        _, a = build_model(c, self.inputs)
        _, b = build_model(replace(c, motor_offsets_m=(.2, .2, .2, .2)), self.inputs)
        self.assertEqual(a, b)

    def test_motor_envelope_is_conservative_and_speed_limited(self):
        motor = self.inputs.motors[0]
        values = motor_torque_limits([motor], np.array([[0.], [3000.], [6000.], [7000.]]))
        np.testing.assert_allclose(values[:, 0], [5., 5., 0., 0.])

    def test_rated_continuous_envelope_needs_no_invented_stall_or_speed(self):
        motor = replace(self.inputs.motors[0], capacity_model="rated_continuous",
                        rated_speed_rpm=2000., stall_torque_Nm=None,
                        no_load_speed_rpm=None, max_speed_rpm=None,
                        peak_torque_Nm=500., peak_duration_s=None)
        speeds = np.array([[0.], [1000.], [-2000.], [2000.1], [-4000.]])
        limits = motor_torque_limits([motor], speeds)
        np.testing.assert_allclose(limits[:, 0], [5., 5., 5., 0., 0.])
        # Even a known higher mechanical limit cannot extend the published
        # continuous operating point; a lower one still caps the evaluation.
        self.assertEqual(replace(motor, max_speed_rpm=6000).evaluation_speed_limit_rpm, 2000)
        low = replace(motor, max_speed_rpm=1500)
        np.testing.assert_allclose(motor_torque_limits([low], [[1499.], [1501.]])[:, 0], [5., 0.])

    def test_nullable_reducer_input_rating_and_rated_speed_are_enforced(self):
        motor = replace(self.inputs.motors[0], capacity_model="rated_continuous",
                        stall_torque_Nm=None, no_load_speed_rpm=None, max_speed_rpm=None)
        reducer = replace(self.inputs.reducers[0], max_input_torque_Nm=None,
                          rated_input_speed_rpm=2000.)
        inputs_with_ratings = replace(self.inputs, motors=(motor,), reducers=(reducer,))
        c = candidate(8)
        model, _ = build_model(c, inputs_with_ratings)
        pose = pose_from_q(model, [.2, .7, -.9, .3, .8, -.2])
        req = SimpleNamespace(joint_speed_rad_s=.3, joint_accel_rad_s2=.4, payload_upper_kg=1.)
        result = evaluate_candidate(c, inputs_with_ratings, [pose], req)
        self.assertTrue(result.feasible)
        self.assertGreater(result.pmax_kg, 1.)
        self.assertEqual(result.summary["unpublished_reducer_input_torque_limits"], [1, 2, 3, 4, 5, 6])
        self.assertEqual(result.summary["motor_capacity_models"], ["rated_continuous"]*6)
        self.assertEqual(result.summary["reducer_evaluation_input_speed_limits_rpm"], [2000.]*6)
        self.assertEqual(reducer.output_torque_limit_Nm, 200.)
        self.assertEqual(replace(reducer, max_input_torque_Nm=.1).output_torque_limit_Nm, 5.)
        # The mechanical maximum is6000rpm, but the published torque reference
        # is2000rpm. The basis must not silently use that torque above2000.
        fast = replace(reducer, rated_input_speed_rpm=100.)
        speed_limited = replace(inputs_with_ratings, reducers=(fast,))
        result = evaluate_candidate(c, speed_limited, [pose], req)
        self.assertFalse(result.feasible)
        self.assertEqual(result.pmax_kg, 0.)
        self.assertEqual(result.per_pose_rows[0]["limitation"], "reducer_input_speed")

    def test_real_policy_export_keeps_unknown_ratings_null_and_servo_limits_finite(self):
        from optimization.model_factory import build_template_model
        import json
        import xml.etree.ElementTree as ET
        motor = replace(self.inputs.motors[0], capacity_model="rated_continuous",
                        stall_torque_Nm=None, no_load_speed_rpm=None, max_speed_rpm=None)
        reducer = replace(self.inputs.reducers[0], max_input_torque_Nm=None,
                          rated_input_speed_rpm=1500.)
        real = replace(self.inputs, motors=(motor,), reducers=(reducer,))
        model, xml = build_template_model(candidate(8), real, actuated=True,
                                        requirement=SimpleNamespace(project_id="REAL_FIXTURE"))
        metadata = json.loads(ET.fromstring(xml).find("./custom/text[@name='armsim_metadata']").get("data"))
        self.assertEqual(model.nu, 6)
        self.assertTrue(np.isfinite(model.actuator_forcerange).all())
        np.testing.assert_allclose(model.actuator_forcerange[:, 1], 4.)
        for drive in metadata["drives"]:
            self.assertIsNone(drive["max_input_torque_Nm"])
            self.assertIsNone(drive["max_motor_speed_rpm"])
            self.assertIsNone(drive["stall_torque_Nm"])
            self.assertEqual(drive["motor_capacity_model"], "rated_continuous")
            self.assertEqual(drive["motor_evaluation_speed_limit_rpm"], 2000.)
            self.assertEqual(drive["reducer_evaluation_input_speed_limit_rpm"], 1500.)

    def test_evaluation_preserves_basis_and_reports_unreachable_cases(self):
        c = candidate()
        model, _ = build_model(c, self.inputs)
        poses = [pose_from_q(model, [0., .7, -1., .3, .8, -.2], "reachable")]
        req = SimpleNamespace(joint_speed_rad_s=.2, joint_accel_rad_s2=.3, payload_upper_kg=1.)
        result = evaluate_candidate(c, self.inputs, poses, req, collect_basis=True)
        self.assertTrue(result.feasible)
        self.assertGreater(result.pmax_kg, 1.)
        self.assertGreater(result.jg, 0)
        self.assertEqual(len(result.basis_rows), 1)
        self.assertEqual(result.summary["velocity_cases_per_pose"], 37)
        poses.append(SimpleNamespace(pose_id="unreachable", position_m=np.array([10., 0., 0.]),
                                     quaternion_wxyz=np.array([1., 0., 0., 0.]), hold_weight=1.))
        result = evaluate_candidate(c, self.inputs, poses, req)
        self.assertFalse(result.feasible)
        self.assertEqual(result.pmax_kg, 0.)
        self.assertEqual(result.reachable_count, 1)
        self.assertEqual(len(result.per_pose_rows), 2)
        self.assertIsNone(result.jg)

    def test_reported_payload_boundary_matches_direct_inverse_dynamics(self):
        c = candidate(8)
        model, _ = build_model(c, self.inputs)
        pose = pose_from_q(model, [.2, .7, -.9, .3, .8, -.2])
        req = SimpleNamespace(joint_speed_rad_s=.3, joint_accel_rad_s2=.4, payload_upper_kg=1.)
        result = evaluate_candidate(c, self.inputs, [pose], req)
        self.assertGreater(result.pmax_kg, 1.)
        row = result.per_pose_rows[0]
        q = np.array([row[f"q{j}_rad"] for j in range(1, 7)])
        velocity = np.array([row[f"limiting_velocity_j{j}_rad_s"] for j in range(1, 7)])
        acceleration = np.array([row[f"limiting_acceleration_j{j}_rad_s2"] for j in range(1, 7)])
        motor = self.inputs.motors[0]
        reducer = self.inputs.reducers[0]
        ratios = np.full(6, reducer.reduction_ratio)
        caps = motor_torque_limits([motor]*6, np.abs(velocity)*ratios*60/(2*np.pi))
        gear_cap = min(reducer.continuous_output_torque_Nm,
                       reducer.max_input_torque_Nm*reducer.reduction_ratio)
        utilizations = []
        for payload in (result.pmax_kg, result.pmax_kg*1.01):
            model, _ = build_model(c, self.inputs, payload)
            data = mujoco.MjData(model)
            data.qpos[:] = q
            data.qvel[:] = velocity
            data.qacc[:] = acceleration
            mujoco.mj_inverse(model, data)
            tau_motor = data.qfrc_inverse/ratios+motor.rotor_inertia_kg_m2*ratios*acceleration
            utilizations.append(max(np.max(np.abs(tau_motor)/caps),
                                    np.max(np.abs(data.qfrc_inverse)/gear_cap)))
        self.assertAlmostEqual(utilizations[0], 1., places=8)
        self.assertGreater(utilizations[1], 1.)

    def test_joint_basis_bypasses_ik_and_records_actual_tcp_positions(self):
        c = candidate(7)
        radius_mm = extended_reach_m(c)*1000
        req = SimpleNamespace(joint_speed_rad_s=.2, joint_accel_rad_s2=.3,
                              payload_upper_kg=1., reach_lower_mm=radius_mm-.1,
                              reach_upper_mm=radius_mm+.1)
        poses = [p for p in generate_poses(req) if p.extension_direction]
        with patch("optimization.physics.solve_pose", side_effect=AssertionError("joint basis invoked IK")):
            result = evaluate_candidate(c, self.inputs, poses, req, collect_basis=True)
        self.assertTrue(result.feasible)
        self.assertTrue(result.summary["reach_bounds_pass"])
        self.assertEqual(result.reachable_count, 6)
        self.assertEqual(len(result.basis_rows), 6)
        for row in result.per_pose_rows:
            self.assertAlmostEqual(row["shoulder_to_tcp_mm"], radius_mm, places=8)
            self.assertIn("tool_px_mm", row)
            self.assertIn("tool_qw", row)
            self.assertIn("q3_rad", row)

    def test_out_of_bounds_extension_is_rejected_before_load_scoring(self):
        c = candidate(7)
        req = SimpleNamespace(joint_speed_rad_s=.2, joint_accel_rad_s2=.3,
                              payload_upper_kg=1., reach_lower_mm=450., reach_upper_mm=600.)
        with patch("optimization.physics.load_basis", side_effect=AssertionError("wrong reach was scored")):
            result = evaluate_candidate(c, self.inputs, generate_poses(req, limit=1), req)
        self.assertTrue(result.summary["geometry_valid"])
        self.assertFalse(result.summary["reach_bounds_pass"])
        self.assertEqual(result.summary["failure_reason"], "extended_reach_out_of_bounds")
        self.assertEqual(result.pmax_kg, 0.)
        self.assertFalse(result.feasible)


if __name__ == "__main__":
    unittest.main()

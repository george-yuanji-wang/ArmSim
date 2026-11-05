"""Independent checks for the fast-joint study and unpublished rotor inertia."""

from dataclasses import replace
import csv
from itertools import combinations, product
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from armsim.simulation import Simulation
from optimization.design import Candidate
from optimization.inputs import (Inputs, LinkBounds, Motor, Reducer, Requirement,
                                 STRUCTURAL_BODIES, load_inputs, load_motors, load_reducers)
from optimization.model_factory import build_template_model
from armsim.mechanism_parameters import GeometryError
from optimization.physics import (_velocity_cases, _velocity_features, build_model,
                                  evaluate_candidate, load_basis, solve_pose)
from optimization.poses import generate_poses
from optimization.search import _rank
from optimization.real_catalog import compile_inputs


ROOT = Path(__file__).resolve().parents[1]
JOINT_SPEEDS = np.array([np.pi]*3+[5*np.pi/3]*3)
ACCELERATION = 6.


def fixture():
    """Compact numerical fixture; these dimensions are not supplier products."""
    mass, radius = .2, .020
    motor = Motor(
        "bounded_rotor", "Compact independent physics fixture", 1., 6000.,
        None, None, None, mass, radius*1000, 18., None,
        capacity_model="rated_continuous",
        rotor_inertia_upper_bound_kg_m2=mass*radius**2,
        rotor_inertia_basis="mass_radius_upper_bound")
    reducer = Reducer("fixture_gear", "Independent test reducer", 30., .1, 25.,
                      20., 100., 150., None, 6000., allowed_joints=(1, 2, 3, 4, 5, 6))
    links = tuple(LinkBounds(name, .01, 1., .1, .9, .01, .2, 10.)
                  for name in STRUCTURAL_BODIES)
    inputs = Inputs((motor,), (reducer,), (), links)
    candidate = Candidate(
        8, (.55-.15-np.hypot(.09, .16), .09, .16),
        (.1,)*6, (.5,)*6, (.1,)*6, (0.,)*4,
        (motor.motor_id,)*6, (reducer.reducer_id,)*6)
    requirement = Requirement(
        "FAST_550_FIXTURE", 500., 500., 550., 550.,
        joint_accel_rad_s2=ACCELERATION,
        joint_speed_limits_rad_s=tuple(JOINT_SPEEDS), allowed_layouts=(8,))
    return candidate, inputs, requirement


class StepperOnlinePhysicsTests(unittest.TestCase):
    def test_manual_eight_motor_selection_loads_without_erasing_full_source_survey(self):
        bundle = compile_inputs()
        expected = {"SO_42BSB41-24-01", "SO_42BSA62-36-01", "SO_57BYA54-24-01",
                    "SO_57BLY54-24-01", "SO_57BLR50-24-01", "SO_57BLR70-24-02",
                    "SO_57BYA94-36-01", "SO_57BSD96-36-01"}
        self.assertEqual({row["motor_id"] for row in bundle["tables"]["motor.csv"]["rows"]}, expected)
        source = json.loads((ROOT/"optimization/catalogs/stepperonline/motors.json").read_text(
            encoding="utf-8"))
        sources = {row["motor_id"]: row for row in source["motors"]}
        self.assertEqual(len(sources), 28)
        self.assertEqual(sum(row["active_study_source_fields_complete_except_inertia"]
                             for row in sources.values()), 21)
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)
            for name, table in bundle["tables"].items():
                with (destination/name).open("w", encoding="utf-8", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=table["columns"])
                    writer.writeheader()
                    writer.writerows(table["rows"])
            (destination/"link_bounds.csv").write_bytes((ROOT/"optimization/data/link_bounds.csv").read_bytes())
            (destination/"study.json").write_text(json.dumps(bundle["study"]), encoding="utf-8")
            loaded = load_inputs(destination, require_catalogs=True)
        self.assertEqual({motor.motor_id for motor in loaded.motors}, expected)
        self.assertEqual(len(loaded.reducers), 3)
        for motor in loaded.motors:
            published = sources[motor.motor_id]
            self.assertEqual(motor.mass_kg, published["mass_kg"])
            self.assertEqual(motor.length_mm, published["body_length_mm"])
            self.assertEqual(motor.radius_mm, published["derived_circumscribed_radius_mm"])
            self.assertEqual(motor.rated_torque_Nm, published["rated_torque_Nm"])
            self.assertEqual(motor.rated_speed_rpm, published["rated_speed_rpm"])
            self.assertEqual(motor.rated_voltage_V, published["rated_voltage_V"])
            self.assertEqual(motor.price_usd, published["price_USD"])
            self.assertEqual(motor.source_url, published["source_url"])
            self.assertIn(motor.rated_voltage_V, (24., 36.))
            self.assertLessEqual(motor.price_usd, 50.)
            self.assertLess(motor.length_mm, 100.)
            self.assertIsNone(motor.rotor_inertia_kg_m2)
            self.assertIsNone(motor.stall_torque_Nm)
            self.assertIsNone(motor.max_speed_rpm)
            self.assertEqual(motor.capacity_model, "rated_continuous")
            self.assertEqual(motor.rotor_inertia_basis, "mass_radius_upper_bound")
            self.assertAlmostEqual(motor.evaluation_rotor_inertia_kg_m2,
                                   motor.mass_kg*motor.radius_m**2, places=15)
        self.assertEqual({r.reducer_id: r.allowed_joints for r in loaded.reducers}, {
            "SITO_HD-14_30": (1,), "SITO_CD-20_30": (2, 3),
            "SITO_CS-11-2XH-A_30": (4, 5, 6)})
        self.assertTrue(all(reducer.reduction_ratio == 30 for reducer in loaded.reducers))
        requirement = loaded.requirements[0]
        np.testing.assert_allclose(requirement.joint_speeds_rad_s, JOINT_SPEEDS)
        self.assertEqual(requirement.joint_accel_rad_s2, ACCELERATION)
        self.assertEqual(requirement.payload_bounds_kg, (.5, .5))
        self.assertEqual(requirement.reach_bounds_m, (.55, .55))
        self.assertEqual(bundle["study"]["wrist2_length_mm"], 110)
        self.assertEqual(requirement.wrist2_length_m, .110)
        self.assertEqual(requirement.shared_wrist_motor_id, "SO_42BSB41-24-01")
        self.assertEqual([len(options) for options in requirement.motor_candidates_by_joint], [2, 3, 5])

    def test_missing_or_understated_rotor_bounds_are_rejected(self):
        table = compile_inputs()["tables"]["motor.csv"]
        source = table["rows"][0]
        physical_bound = source["mass_kg"]*(source["radius_mm"]/1000)**2
        for bound, reason in ((None, "published value or explicit mass_radius_upper_bound"),
                              (.99*physical_bound, "below mass_kg")):
            with self.subTest(bound=bound), tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/"motor.csv"
                row = {**source, "rotor_inertia_kg_m2": None,
                       "rotor_inertia_upper_bound_kg_m2": bound}
                with path.open("w", encoding="utf-8", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=table["columns"])
                    writer.writeheader()
                    writer.writerow(row)
                with self.assertRaisesRegex(ValueError, reason):
                    load_motors(path)

    def test_invalid_or_inapplicable_reducer_joint_eligibility_is_rejected(self):
        table = compile_inputs()["tables"]["reducer.csv"]
        for eligibility in ("1;1", "0", "7", "2.5"):
            with self.subTest(eligibility=eligibility), tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/"reducer.csv"
                with path.open("w", encoding="utf-8", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=table["columns"])
                    writer.writeheader()
                    writer.writerow({**table["rows"][0], "allowed_joints": eligibility})
                with self.assertRaisesRegex(ValueError, "invalid allowed_joints"):
                    load_reducers(path)
        candidate, inputs, _ = fixture()
        wrong_joint = replace(inputs.reducers[0], allowed_joints=(2, 3))
        with self.assertRaisesRegex(GeometryError, "not allowed at J1"):
            build_model(candidate, replace(inputs, reducers=(wrong_joint,)))

    def test_37_velocity_states_reach_the_individual_speed_limits(self):
        velocities = _velocity_cases(JOINT_SPEEDS)
        self.assertEqual(velocities.shape, (37, 6))
        np.testing.assert_array_equal(velocities[0], np.zeros(6))
        np.testing.assert_allclose(np.max(np.abs(velocities), axis=0), JOINT_SPEEDS)
        self.assertEqual(np.count_nonzero(np.count_nonzero(velocities, axis=1) == 1), 6)
        self.assertEqual(np.count_nonzero(np.count_nonzero(velocities, axis=1) == 2), 30)
        for first, second in combinations(range(6), 2):
            for sign in (-1, 1):
                expected = np.zeros(6)
                expected[first], expected[second] = JOINT_SPEEDS[first], sign*JOINT_SPEEDS[second]
                self.assertTrue(np.any(np.all(velocities == expected, axis=1)))
        # Simultaneous sign reversal adds no new quadratic-velocity load.
        np.testing.assert_array_equal(_velocity_features(-velocities),
                                      _velocity_features(velocities))

    def test_ik_reads_the_actual_default_and_110_mm_distal_offsets(self):
        candidate, inputs, _ = fixture()
        rng = np.random.default_rng(311)
        for wrist_length in (.070, .110):
            model, _ = build_model(candidate, inputs)
            # Isolate the FK/IK contract from catalog fitting: the scalar
            # joint-6 origin is the authoritative wrist-to-tool offset.
            model.body_pos[model.jnt_bodyid[5], 2] = wrist_length
            data = mujoco.MjData(model)
            mujoco.mj_setConst(model, data)
            for _ in range(12):
                data.qpos[:] = rng.uniform(-np.pi, np.pi, 6)
                mujoco.mj_forward(model, data)
                position = data.site("tool_frame").xpos.copy()
                rotation = data.site("tool_frame").xmat.copy()
                quaternion = np.empty(4)
                mujoco.mju_mat2Quat(quaternion, rotation)
                found = solve_pose(model, data, position, quaternion)
                self.assertIsNotNone(found)
                np.testing.assert_allclose(data.site("tool_frame").xpos, position, atol=2e-8)
                np.testing.assert_allclose(data.site("tool_frame").xmat, rotation, atol=2e-8)

    def test_quadratic_basis_matches_inverse_dynamics_at_fast_mixed_speeds(self):
        candidate, inputs, _ = fixture()
        model, _ = build_model(candidate, inputs, payload_kg=.5)
        data = mujoco.MjData(model)
        q = np.array([.3, .7, -.9, .4, .8, -.5])
        gravity, mass_matrix, quadratic = load_basis(model, data, q)
        rng = np.random.default_rng(164)
        for _ in range(8):
            velocity = rng.uniform(-1., 1., 6)*JOINT_SPEEDS
            acceleration = rng.uniform(-ACCELERATION, ACCELERATION, 6)
            expected = (gravity+mass_matrix@acceleration+
                        _velocity_features(velocity[None])[0]@quadratic)
            data.qpos[:], data.qvel[:], data.qacc[:] = q, velocity, acceleration
            mujoco.mj_inverse(model, data)
            np.testing.assert_allclose(data.qfrc_inverse, expected, atol=2e-11)

    def test_upper_bound_cannot_improve_acceleration_box_or_payload_capacity(self):
        candidate, inputs, requirement = fixture()
        motor = inputs.motors[0]
        bound = motor.mass_kg*motor.radius_m**2
        self.assertIsNone(motor.rotor_inertia_kg_m2)
        self.assertEqual(motor.evaluation_rotor_inertia_kg_m2, bound)
        model, _ = build_model(candidate, inputs, payload_kg=.5)
        gravity, matrix, quadratic = load_basis(
            model, mujoco.MjData(model), [.2, .7, -.9, .4, .8, -.3])
        velocities = _velocity_cases(JOINT_SPEEDS)
        constant = (gravity+_velocity_features(velocities)@quadratic)/30.
        corners = np.array(list(product((-ACCELERATION, ACCELERATION), repeat=6)))
        previous = None
        capacities = []
        poses = [pose for pose in generate_poses(requirement) if pose.extension_direction]
        for inertia in (0., .2*bound, bound):
            # Independent exact formula for max |c+B*a| over a symmetric box.
            # The positive diagonal Mjj/N + N*J increases monotonically with J.
            coefficients = matrix/30.+np.eye(6)*30.*inertia
            expected_worst = np.abs(constant)+ACCELERATION*np.abs(coefficients).sum(axis=1)
            brute_worst = np.max(np.abs(constant[:, None, :]+
                                       (corners@coefficients.T)[None]), axis=1)
            np.testing.assert_allclose(brute_worst, expected_worst, atol=2e-12)
            if previous is not None:
                self.assertTrue(np.all(expected_worst >= previous-1e-12))
            previous = expected_worst
            actual_motor = replace(motor, rotor_inertia_kg_m2=inertia,
                                   rotor_inertia_upper_bound_kg_m2=None,
                                   rotor_inertia_basis="published")
            result = evaluate_candidate(candidate, replace(inputs, motors=(actual_motor,)),
                                        poses, requirement)
            self.assertTrue(result.summary["geometry_valid"])
            self.assertGreater(result.pmax_kg, 0.)
            capacities.append(result.pmax_kg)
            np.testing.assert_allclose(result.summary["joint_speed_limits_rad_s"], JOINT_SPEEDS)
        self.assertGreater(capacities[0], capacities[-1])
        self.assertTrue(all(a >= b for a, b in zip(capacities, capacities[1:])))
        bounded = evaluate_candidate(candidate, inputs, poses, requirement)
        self.assertAlmostEqual(bounded.pmax_kg, capacities[-1], places=12)

    def test_empty_arm_diagnostics_guide_an_all_zero_payload_search(self):
        candidate, inputs, requirement = fixture()
        poses = [pose for pose in generate_poses(requirement) if pose.extension_direction]
        evaluations = []
        for capacity, mass in ((.01, .01), (.1, .2)):
            reducer = replace(inputs.reducers[0], continuous_output_torque_Nm=capacity,
                              peak_output_torque_Nm=2*capacity, mass_kg=mass)
            result = evaluate_candidate(candidate, replace(inputs, reducers=(reducer,)),
                                        poses, requirement)
            self.assertEqual(result.pmax_kg, 0.)
            self.assertFalse(result.summary["empty_arm_passes"])
            self.assertTrue(all(row["pmax_kg"] == 0. for row in result.per_pose_rows))
            empty = np.array([[row[f"peak_empty_arm_utilization_j{j}_ratio"]
                               for j in range(1, 7)] for row in result.per_pose_rows])
            required = np.array([[max(row[f"peak_motor_utilization_j{j}_ratio"],
                                      row[f"peak_reducer_utilization_j{j}_ratio"])
                                  for j in range(1, 7)] for row in result.per_pose_rows])
            np.testing.assert_allclose(result.summary["peak_empty_arm_utilization_by_joint"],
                                       empty.max(axis=0), atol=1e-12)
            np.testing.assert_allclose(result.summary["peak_required_payload_utilization_by_joint"],
                                       required.max(axis=0), atol=1e-12)
            self.assertEqual(result.summary["peak_empty_arm_utilization"], float(empty.max()))
            self.assertEqual(result.summary["peak_required_payload_utilization"], float(required.max()))
            for row in result.per_pose_rows:
                for joint in range(1, 7):
                    self.assertAlmostEqual(
                        row[f"peak_reducer_utilization_j{joint}_ratio"],
                        row[f"peak_joint_load_j{joint}_Nm_at_required_payload"]/capacity,
                        places=10)
            evaluations.append(result)
        weak_light, closer_heavy = evaluations
        self.assertGreater(closer_heavy.total_mass_kg, weak_light.total_mass_kg)
        self.assertLess(closer_heavy.summary["peak_empty_arm_utilization"],
                        weak_light.summary["peak_empty_arm_utilization"])
        self.assertGreater(_rank(closer_heavy), _rank(weak_light))

    def test_positive_payload_ranking_preserves_payload_then_mass_priority(self):
        candidate, inputs, requirement = fixture()
        poses = [pose for pose in generate_poses(requirement) if pose.extension_direction]
        positive = evaluate_candidate(candidate, inputs, poses, requirement)
        self.assertGreater(positive.pmax_kg, 0.)
        zero = replace(positive, pmax_kg=0., total_mass_kg=.001,
                       summary={**positive.summary, "peak_empty_arm_utilization": 1.01})
        self.assertGreater(_rank(positive), _rank(zero))
        # Once capacity is positive, empty-arm utilization is diagnostic only;
        # mass remains the tie-breaker for equal payload capacities.
        heavy = replace(positive, total_mass_kg=10.,
                        summary={**positive.summary, "peak_empty_arm_utilization": .2})
        light = replace(positive, total_mass_kg=1.,
                        summary={**positive.summary, "peak_empty_arm_utilization": .9})
        self.assertGreater(_rank(light), _rank(heavy))
        higher_payload = replace(heavy, pmax_kg=positive.pmax_kg+.01)
        self.assertGreater(_rank(higher_payload), _rank(light))

    def test_export_counts_bound_once_retains_unknown_source_and_runs_finitely(self):
        candidate, inputs, requirement = fixture()
        scoring, _ = build_template_model(candidate, inputs, payload_kg=.5)
        exported, xml = build_template_model(candidate, inputs, payload_kg=.5,
                                             actuated=True, requirement=requirement)
        rotor = inputs.motors[0].evaluation_rotor_inertia_kg_m2
        expected_armature = np.full(6, 30.**2*rotor)
        np.testing.assert_array_equal(scoring.dof_armature, np.zeros(6))
        np.testing.assert_allclose(exported.dof_armature, expected_armature, atol=1e-14)
        metadata = json.loads(ET.fromstring(xml).find(
            "./custom/text[@name='armsim_metadata']").get("data"))
        for drive in metadata["drives"]:
            self.assertIsNone(drive["published_rotor_inertia_kg_m2"])
            self.assertEqual(drive["rotor_inertia_basis"], "mass_radius_upper_bound")
            self.assertAlmostEqual(drive["reflected_armature_kg_m2"], expected_armature[0])
        q = np.array([.2, .7, -.9, .4, .8, -.3])
        velocity = np.array([.8, -.4, .5, -.7, .6, -.9])*JOINT_SPEEDS
        acceleration = np.array([.9, -.7, .5, -.3, .1, -.8])*ACCELERATION
        torques = []
        for model in (scoring, exported):
            data = mujoco.MjData(model)
            data.qpos[:], data.qvel[:], data.qacc[:] = q, velocity, acceleration
            mujoco.mj_inverse(model, data)
            torques.append(data.qfrc_inverse.copy())
        np.testing.assert_allclose(torques[1]-torques[0],
                                   expected_armature*acceleration, atol=2e-11)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"bounded_rotor.xml"
            path.write_text(xml, encoding="utf-8")
            sim = Simulation.from_path(path)
            self.assertTrue(sim.supports_dynamics)
            sim.dynamic = True
            for joint in range(6):
                sim.set_target(joint, sim.targets[joint]+.1*(-1)**joint)
            for _ in range(25):
                sim.advance(.02)
            self.assertTrue(np.isfinite(sim.data.qpos).all())
            self.assertTrue(np.isfinite(sim.data.qvel).all())
            self.assertFalse(any(warning.number for warning in sim.data.warning))


if __name__ == "__main__":
    unittest.main()

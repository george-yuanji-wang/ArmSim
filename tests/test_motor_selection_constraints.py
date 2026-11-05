"""Motor shortlists and a fixed wrist apply to every hardware proposal path."""

import csv
from dataclasses import replace
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from optimization.design import (hardware_pairs, initial_candidate, propose_candidate,
                                 _fit_initial_hardware, _uniform_hardware_seed)
from optimization.inputs import (InputError, MOTOR_COLUMNS, REDUCER_COLUMNS, Requirement,
                                 load_inputs, to_jsonable)


DEMO = Path(__file__).resolve().parents[1]/"optimization/examples/synthetic"


def fixture():
    """All fixture packages fit; only IDs and drive capacities distinguish them."""
    original = load_inputs(DEMO)
    motors = tuple(replace(original.motor_by_id[
                               "SYNTHETIC_M_WRIST" if identity == "WRIST" else "SYNTHETIC_M_SMALL"],
                           motor_id=identity, name=identity,
                           rated_torque_Nm=torque, stall_torque_Nm=3.,
                           peak_torque_Nm=1.8, peak_duration_s=1.)
                   for identity, torque in (("ARM_A", .2), ("ARM_B", .35),
                                            ("ARM_C", .4), ("WRIST", .22),
                                            ("OUTSIDE", .8)))
    compact_gear = original.reducer_by_id["SYNTHETIC_R_WRIST"]
    reducers = tuple(replace(compact_gear, reducer_id=identity, name=identity,
                             reduction_ratio=30., allowed_joints=joints)
                     for identity, joints in (("G1", (1,)), ("G23", (2, 3)),
                                               ("G456", (4, 5, 6))))
    data = replace(original, motors=motors, reducers=reducers)
    requirement = replace(original.requirements[0],
        motor_candidates_by_joint=(("ARM_A", "ARM_B"), ("ARM_C",), ("ARM_A", "ARM_B")),
        shared_wrist_motor_id="WRIST")
    return data, requirement


def write_input_fixture(destination, study):
    data, _ = fixture()
    for name in ("link_bounds.csv", "requirement.csv"):
        shutil.copyfile(DEMO/name, destination/name)
    for name, columns, rows in (
            ("motor.csv", MOTOR_COLUMNS, data.motors),
            ("reducer.csv", (*REDUCER_COLUMNS, "allowed_joints"), data.reducers)):
        with (destination/name).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            for row in rows:
                values = {key: getattr(row, key) for key in columns}
                if "allowed_joints" in values:
                    values["allowed_joints"] = ";".join(map(str, values["allowed_joints"]))
                writer.writerow(values)
    (destination/"study.json").write_text(json.dumps(study), encoding="utf-8")


def scoped_study():
    return {
        "motor_candidates_by_joint": {"J1": ["ARM_A", "ARM_B"], "J2": ["ARM_C"],
                                      "J3": ["ARM_A", "ARM_B"]},
        "shared_wrist_motor_id": "WRIST",
        "joint_speed_limits_deg_s": [180, 180, 180, 300, 300, 300],
        "joint_accel_rad_s2": 6,
        "wrist2_length_mm": 110,
    }


class MotorSelectionConstraintTests(unittest.TestCase):
    def assert_scope(self, candidate, requirement):
        for joint, mid in enumerate(candidate.motor_ids, 1):
            self.assertTrue(requirement.allows_motor(joint, mid), (joint, mid))
        self.assertEqual(candidate.motor_ids[3:], ("WRIST",)*3)

    def test_saved_requirements_keep_an_immutable_scope_and_legacy_defaults(self):
        _, requirement = fixture()
        saved = json.loads(json.dumps(to_jsonable(requirement)))
        saved["allowed_layouts"] = tuple(saved["allowed_layouts"])
        restored = Requirement(**saved)
        self.assertEqual(restored, requirement)
        self.assertIsInstance(restored.motor_candidates_by_joint, tuple)
        self.assertIsInstance(restored.motor_candidates_by_joint[0], tuple)
        self.assertEqual(hash(restored), hash(requirement))
        self.assertTrue(restored.allows_motor(1, "ARM_B"))
        self.assertFalse(restored.allows_motor(2, "ARM_B"))
        self.assertFalse(restored.allows_motor(4, "ARM_B"))
        self.assertTrue(restored.allows_motor(6, "WRIST"))
        unrestricted = replace(restored, motor_candidates_by_joint=None, shared_wrist_motor_id=None)
        self.assertTrue(all(unrestricted.allows_motor(joint, "ANYTHING") for joint in range(1, 7)))
        for joint in (0, 7, True, "J1"):
            with self.subTest(joint=joint), self.assertRaises(ValueError):
                restored.allows_motor(joint, "ARM_A")

    def test_malformed_or_empty_scopes_are_rejected(self):
        _, requirement = fixture()
        for groups in ((["A"], ["B"]), (["A"], ["B"], []),
                       (["A", " A "], ["B"], ["C"]),
                       (["A"], "B", ["C"]), (["A"], [42], ["C"])):
            with self.subTest(groups=groups), self.assertRaisesRegex(ValueError, "motor_candidates_by_joint"):
                replace(requirement, motor_candidates_by_joint=groups)
        for mid in ("", "  ", 23):
            with self.subTest(mid=mid), self.assertRaisesRegex(ValueError, "shared_wrist_motor_id"):
                replace(requirement, shared_wrist_motor_id=mid)

    def test_study_file_loads_exact_joint_map_and_existing_motion_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)
            write_input_fixture(destination, scoped_study())
            data = load_inputs(destination)
        requirement, = data.requirements
        self.assertEqual(requirement.motor_candidates_by_joint,
                         (("ARM_A", "ARM_B"), ("ARM_C",), ("ARM_A", "ARM_B")))
        self.assertEqual(requirement.shared_wrist_motor_id, "WRIST")
        np.testing.assert_allclose(requirement.joint_speeds_rad_s, [np.pi]*3+[5*np.pi/3]*3)
        self.assertEqual(requirement.joint_accel_rad_s2, 6)
        self.assertEqual(requirement.wrist2_length_m, .11)

    def test_study_rejects_unknown_ids_missing_joint_maps_and_extra_wrist_lists(self):
        invalid = []
        study = scoped_study()
        study["motor_candidates_by_joint"]["J2"] = ["UNLISTED"]
        invalid.append((study, "unknown motor IDs"))
        study = scoped_study()
        study["shared_wrist_motor_id"] = "UNLISTED"
        invalid.append((study, "unknown motor IDs"))
        for groups in ({"J1": ["ARM_A"], "J2": ["ARM_C"]},
                       {**scoped_study()["motor_candidates_by_joint"], "J4": ["WRIST"]},
                       {"J1": [], "J2": ["ARM_C"], "J3": ["ARM_A"]}):
            study = scoped_study()
            study["motor_candidates_by_joint"] = groups
            invalid.append((study, "motor_candidates_by_joint"))
        for value in (None, "", 3):
            study = scoped_study()
            study["shared_wrist_motor_id"] = value
            invalid.append((study, "shared_wrist_motor_id"))
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)
            for study, reason in invalid:
                with self.subTest(study=study):
                    write_input_fixture(destination, study)
                    with self.assertRaisesRegex(InputError, reason):
                        load_inputs(destination)

    def test_hardware_pools_never_borrow_a_motor_from_another_joint(self):
        inputs, requirement = fixture()
        for joint in range(6):
            pool = hardware_pairs(inputs, requirement, joint)
            self.assertTrue(pool)
            self.assertTrue(all(requirement.allows_motor(joint+1, motor.motor_id)
                                and joint+1 in reducer.allowed_joints for motor, reducer in pool))
        missing = replace(requirement, motor_candidates_by_joint=(("MISSING",), ("ARM_C",), ("ARM_A",)))
        with self.assertRaisesRegex(ValueError, "J1.*motor selection"):
            hardware_pairs(inputs, missing, 0)

    def test_every_geometry_proposal_preserves_the_fixed_wrist_and_joint_shortlists(self):
        inputs, requirement = fixture()
        from optimization.design import _compile_candidate
        from optimization.model_factory import build_template_model
        attempted = []

        def checked(candidate, data):
            self.assert_scope(candidate, requirement)
            attempted.append(candidate)
            return _compile_candidate(candidate, data)

        with patch("optimization.design._compile_candidate", side_effect=checked):
            for number in (7, 8, 9):
                requirement = replace(requirement, allowed_layouts=(number,))
                candidate = initial_candidate(inputs, requirement)
                self.assertEqual(build_template_model(candidate, inputs)[0].nv, 6)
                rng = np.random.default_rng(7)
                for cycle in range(1, 18):
                    candidate = propose_candidate(candidate, inputs, requirement, rng, cycle)
                    self.assert_scope(candidate, requirement)
        self.assertGreater(len(attempted), 50)

    def test_repair_and_uniform_fallback_apply_scope_before_geometry(self):
        inputs, requirement = fixture()
        requirement = replace(requirement, allowed_layouts=(8,))
        seed = initial_candidate(inputs, requirement)
        wrong = replace(seed, motor_ids=("OUTSIDE",)*6)
        pairs = hardware_pairs(inputs, requirement)
        from optimization.model_factory import build_template_model
        probes = []

        def checked(candidate, data):
            self.assert_scope(candidate, requirement)
            probes.append(candidate)
            return build_template_model(candidate, data)

        with patch("optimization.model_factory.build_template_model", side_effect=checked):
            repaired = _fit_initial_hardware(wrong, inputs, pairs, requirement=requirement)
        self.assert_scope(repaired, requirement)
        self.assertTrue(probes)
        uniform = _uniform_hardware_seed(wrong, inputs, pairs, requirement)
        self.assertIsNotNone(uniform)
        self.assert_scope(uniform, requirement)
        with self.assertRaisesRegex(ValueError, "outside this study's selection"):
            propose_candidate(wrong, inputs, requirement, np.random.default_rng(0), 1)

    def test_seed_repairs_permitted_placement_before_downgrading_hardware(self):
        inputs, requirement = fixture()
        requirement = replace(requirement, allowed_layouts=(8,), motor_offset_max_mm=2.)
        expected = (0., 0., -.002, -.002)
        attempts = []

        def geometry(candidate, data):
            self.assert_scope(candidate, requirement)
            self.assertEqual(candidate.motor_ids[:3], ("ARM_B", "ARM_C", "ARM_B"))
            self.assertTrue(all(abs(value) <= .002 for value in candidate.motor_offsets_m))
            attempts.append(candidate.motor_offsets_m)
            return candidate if candidate.motor_offsets_m == expected else None

        with patch("optimization.design._compile_candidate", side_effect=geometry), patch(
                "optimization.design._fit_initial_hardware",
                side_effect=AssertionError("Selected hardware must not be downgraded before trying placement")):
            seed = initial_candidate(inputs, requirement)
        self.assertEqual(attempts[0], (0.,)*4)
        self.assertEqual(attempts[1], expected)
        self.assertEqual(seed.motor_offsets_m, expected)
        self.assertEqual(seed.motor_ids[3:], ("WRIST",)*3)


if __name__ == "__main__":
    unittest.main()

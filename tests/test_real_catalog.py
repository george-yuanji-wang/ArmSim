"""Real hardware follows the existing supplier -> data/real -> run workflow."""

import csv
import hashlib
import inspect
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from optimization.inputs import DATA_DIR, load_inputs
from optimization.real_catalog import CATALOG_DIR, REAL_DATA_DIR, build_real_catalog
from optimization.storage import create_run


REPO = Path(__file__).resolve().parents[1]
SELECTED_MOTORS = {
    "SO_42BSB41-24-01", "SO_42BSA62-36-01", "SO_57BYA54-24-01",
    "SO_57BLY54-24-01", "SO_57BLR50-24-01", "SO_57BLR70-24-02",
    "SO_57BYA94-36-01", "SO_57BSD96-36-01",
}


def read_csv(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class RealCatalogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temporary.name)
        cls.data = cls.root/"real_inputs"
        cls.generic_before = {path.name: path.read_bytes() for path in DATA_DIR.glob("*.csv")}
        motor_path = CATALOG_DIR/"stepperonline/motors.json"
        selection_path = CATALOG_DIR/"stepperonline/motor_selection.json"
        reducer_path = CATALOG_DIR/"sitodrive/reducer_selection.json"
        cls.original = {path: digest(path) for path in (motor_path, selection_path, reducer_path)}
        cls.motor_sources = {row["motor_id"]: row for row in json.loads(
            motor_path.read_text(encoding="utf-8"))["motors"]}
        cls.reducer_sources = {row["reducer_id"]: row for row in json.loads(
            reducer_path.read_text(encoding="utf-8"))["reducers"]}
        cls.audit = build_real_catalog(output_dir=cls.data)
        cls.inputs = load_inputs(cls.data)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_real_builder_defaults_to_existing_real_directory_and_preserves_generic_inputs(self):
        self.assertEqual(REAL_DATA_DIR, DATA_DIR/"real")
        self.assertEqual(inspect.signature(build_real_catalog).parameters["output_dir"].default,
                         DATA_DIR/"real")
        self.assertEqual({path.name: path.read_bytes() for path in DATA_DIR.glob("*.csv")},
                         self.generic_before)
        self.assertEqual({path: digest(path) for path in self.original}, self.original)

    def test_only_selected_complete_motors_load_with_source_ratings_and_explicit_inertia_bounds(self):
        self.assertEqual(set(self.inputs.motor_by_id), SELECTED_MOTORS)
        self.assertEqual(self.audit["motor_count"], 8)
        self.assertEqual(sum(m.rated_voltage_V == 36 for m in self.inputs.motors), 3)
        # Manual selection does not erase supplier records or the prior
        # 26 earlier records plus two fresh 36 V alternatives remain available.
        self.assertEqual(len(self.motor_sources), 28)
        self.assertEqual(sum(row["active_study_source_fields_complete_except_inertia"]
                             for row in self.motor_sources.values()), 21)
        for motor in self.inputs.motors:
            source = self.motor_sources[motor.motor_id]
            self.assertEqual(source["manufacturer"], "STEPPERONLINE")
            self.assertTrue(source["hall_sensors_included"])
            self.assertFalse(source["driver_included"])
            self.assertEqual(motor.capacity_model, "rated_continuous")
            self.assertEqual(motor.rated_torque_Nm, source["rated_torque_Nm"])
            self.assertEqual(motor.rated_speed_rpm, source["rated_speed_rpm"])
            self.assertEqual(motor.mass_kg, source["mass_kg"])
            self.assertEqual(motor.radius_mm, source["derived_circumscribed_radius_mm"])
            self.assertEqual(motor.length_mm, source["body_length_mm"])
            self.assertEqual(motor.rated_voltage_V, source["rated_voltage_V"])
            self.assertIn(motor.rated_voltage_V, (24, 36))
            self.assertLessEqual(motor.price_usd, 50)
            self.assertLess(motor.length_mm, 100)
            self.assertEqual(motor.source_url, source["source_url"])
            self.assertIsNone(motor.rotor_inertia_kg_m2)
            self.assertEqual(motor.rotor_inertia_basis, "mass_radius_upper_bound")
            self.assertAlmostEqual(motor.evaluation_rotor_inertia_kg_m2,
                                   motor.mass_kg*motor.radius_m**2, places=15)
            for key in ("stall_torque_Nm", "no_load_speed_rpm", "max_speed_rpm"):
                self.assertEqual(getattr(motor, key), source.get(key))
            self.assertLessEqual(motor.evaluation_speed_limit_rpm, motor.rated_speed_rpm)

    def test_reducers_retain_exact_user_models_ratios_joint_assignments_and_supplier_paths(self):
        expected = {
            "SITO_HD-14_30": ((1,), 3., 35., 17.5),
            "SITO_CD-20_30": ((2, 3), 10.2, 35., 29.7),
            "SITO_CS-11-2XH-A_30": ((4, 5, 6), 2.2, 29., 32.),
        }
        self.assertEqual(set(self.inputs.reducer_by_id), set(expected))
        self.assertEqual(self.audit["reducer_count"], 3)
        for reducer in self.inputs.reducers:
            joints, torque, radius, length = expected[reducer.reducer_id]
            self.assertEqual(reducer.allowed_joints, joints)
            self.assertEqual(reducer.reduction_ratio, 30)
            self.assertEqual(reducer.continuous_output_torque_Nm, torque)
            self.assertEqual(reducer.radius_mm, radius)
            self.assertEqual(reducer.length_mm, length)
            self.assertIsNone(reducer.max_input_torque_Nm)
            self.assertEqual(reducer.evaluation_input_speed_limit_rpm, 2000.)
            source = self.reducer_sources[reducer.reducer_id]
            self.assertEqual(reducer.mass_kg, source["mass_kg"])
            drawing = (REPO/source["drawing_local_path"]).resolve()
            self.assertTrue(drawing.is_relative_to((CATALOG_DIR/"sitodrive").resolve()))
            self.assertTrue(drawing.is_file())
            self.assertEqual(digest(drawing), source["drawing_sha256"])

    def test_prepared_requirement_and_motion_settings_are_the_selected_study(self):
        requirement, = self.inputs.requirements
        self.assertEqual(requirement.project_id, "SO_30_550")
        self.assertEqual(requirement.reach_bounds_m, (.55, .55))
        self.assertEqual(requirement.payload_bounds_kg, (.5, .5))
        for measured, expected in zip(requirement.joint_speeds_rad_s,
                                      (math.pi,)*3+(5*math.pi/3,)*3):
            self.assertAlmostEqual(measured, expected)
        self.assertEqual(requirement.joint_accel_rad_s2, 6)
        self.assertEqual(requirement.wrist2_length_m, .110)
        self.assertEqual(requirement.allowed_layouts, (7, 8, 9))
        self.assertEqual(requirement.shared_wrist_motor_id, "SO_42BSB41-24-01")
        self.assertEqual(requirement.motor_candidates_by_joint, (
            ("SO_57BLR50-24-01", "SO_57BLR70-24-02"),
            ("SO_57BLR70-24-02", "SO_57BYA94-36-01", "SO_57BSD96-36-01"),
            ("SO_42BSA62-36-01", "SO_57BYA54-24-01", "SO_57BLY54-24-01",
             "SO_57BLR50-24-01", "SO_57BLR70-24-02"),
        ))
        self.assertEqual(math.prod(map(len, requirement.motor_candidates_by_joint)), 30)
        for joint in (4, 5, 6):
            self.assertTrue(requirement.allows_motor(joint, "SO_42BSB41-24-01"))
            self.assertFalse(requirement.allows_motor(joint, "SO_57BLR70-24-02"))

    def test_manual_screen_records_the_selection_and_source_hash(self):
        selection_path = CATALOG_DIR/"stepperonline/motor_selection.json"
        selection = json.loads(selection_path.read_text(encoding="utf-8"))
        self.assertEqual(set(selection["selectable_motor_ids"]), SELECTED_MOTORS)
        self.assertEqual(set(selection["motor_decisions"]), set(self.motor_sources))
        self.assertTrue(all(row["reason"].strip() for row in selection["motor_decisions"].values()))
        self.assertEqual(self.audit["catalog_source_sha256"]["stepperonline/motor_selection.json"],
                         digest(selection_path))
        self.assertEqual(self.audit["manual_motor_selection"], selection)

    def test_source_and_assumption_tables_are_kept_in_existing_snapshot_layout(self):
        exclusions = read_csv(self.data/"excluded_hardware.csv")
        self.assertEqual(len(exclusions), 20)
        self.assertTrue(all(row["kind"] == "motor" for row in exclusions))
        self.assertTrue(all(row["reasons"] and row["source_url"] for row in exclusions))
        assumptions = json.loads((self.data/"input_assumptions.json").read_text(encoding="utf-8"))
        self.assertEqual(assumptions["motor_count"], 8)
        self.assertEqual(assumptions["reducer_count"], 3)
        provenance = read_csv(self.data/"hardware_provenance.csv")
        self.assertEqual(len(provenance), 11)
        self.assertEqual({row["id"] for row in provenance},
                         set(self.inputs.motor_by_id) | set(self.inputs.reducer_by_id))
        self.assertTrue(all(row["source_url"].startswith("https://") for row in provenance))
        store = create_run(self.root/"runs", "provenance_check", self.data, {"mode": "test"})
        for name in ("hardware_provenance.csv", "study_assumptions.csv", "excluded_hardware.csv",
                     "study.json", "input_assumptions.json", "motor.csv", "reducer.csv",
                     "requirement.csv", "link_bounds.csv"):
            self.assertEqual((store.path/"inputs"/name).read_bytes(), (self.data/name).read_bytes())
        self.assertEqual({path: digest(path) for path in self.original}, self.original)

    def test_regeneration_archives_prior_inputs_before_replacing_them(self):
        destination = self.root/"repeatable_real_inputs"
        build_real_catalog(output_dir=destination)
        old_motor = b"previous manually edited motor inputs\r\n"
        (destination/"motor.csv").write_bytes(old_motor)
        prior = {name: (destination/name).read_bytes() for name in
                 ("motor.csv", "reducer.csv", "requirement.csv", "study.json")}
        build_real_catalog(output_dir=destination)
        archives = list((destination/"archive").iterdir())
        matching = [folder for folder in archives if (folder/"motor.csv").is_file()
                    and (folder/"motor.csv").read_bytes() == old_motor]
        self.assertEqual(len(matching), 1)
        for name, content in prior.items():
            self.assertEqual((matching[0]/name).read_bytes(), content)
        rebuilt = load_inputs(destination)
        self.assertEqual(set(rebuilt.motor_by_id), SELECTED_MOTORS)
        self.assertEqual(len(rebuilt.reducers), 3)
        self.assertEqual({path.name: path.read_bytes() for path in DATA_DIR.glob("*.csv")},
                         self.generic_before)

    def test_real_launcher_uses_real_inputs_without_stale_project_filter(self):
        from run_optimization import main
        with patch("optimization.__main__.main", return_value=0) as launch:
            self.assertEqual(main(["--real", "--cycles", "1"]), 0)
        arguments, = launch.call_args.args
        self.assertEqual(arguments[arguments.index("--data-dir")+1], str(REAL_DATA_DIR))
        self.assertNotIn("--project", arguments)
        self.assertEqual(arguments[-2:], ["--cycles", "1"])

    def test_36v_j2_alternatives_preserve_net_mass_and_unknown_current_convention(self):
        expected = {
            "SO_57BYA94-36-01": ("round", 93.5, 1.03, 36.74, .58, 1.72, 7.15),
            "SO_57BSD96-36-01": ("square", 96., .81, 24.83, .91, .95, 5.7),
        }
        for identity, (shape, length, mass, price, resistance, inductance, emf) in expected.items():
            source = self.motor_sources[identity]
            motor = self.inputs.motor_by_id[identity]
            self.assertEqual((motor.rated_voltage_V, motor.rated_torque_Nm,
                              motor.rated_speed_rpm, motor.rated_current_A), (36., .45, 3000., 5.4))
            self.assertEqual((motor.length_mm, motor.mass_kg, motor.price_usd), (length, mass, price))
            self.assertEqual(source["body_shape"], shape)
            self.assertEqual(source["phase_resistance_ohm"], resistance)
            self.assertEqual(source["phase_inductance_mH"], inductance)
            self.assertEqual(source["back_emf_published_V_per_krpm"], emf)
            self.assertIn("does not distinguish", source["rated_current_convention"])
            snapshot = CATALOG_DIR/"stepperonline"/source["source_snapshot"]
            self.assertEqual(digest(snapshot), source["source_sha256"])
            self.assertTrue(self.inputs.requirements[0].allows_motor(2, identity))


if __name__ == "__main__":
    unittest.main()

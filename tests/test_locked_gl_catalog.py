"""The fixed study must not reintroduce old hardware or turn estimates into ratings."""

import csv
from dataclasses import asdict
import json
from pathlib import Path
import tempfile
import unittest

from optimization.inputs import InputError, Motor, load_inputs
from optimization.locked_catalog import compile_locked_inputs
from optimization.real_catalog import CATALOG_DIR, build_real_catalog
from optimization.storage import create_run


class LockedGLCatalogTests(unittest.TestCase):
    def test_exact_hardware_scope_and_original_motor_ratings(self):
        bundle = compile_locked_inputs(CATALOG_DIR)
        motors = {row["motor_id"]: row for row in bundle["tables"]["motor.csv"]["rows"]}
        self.assertEqual(set(motors), {"CUBEMARS_GL40_KV210", "CUBEMARS_GL30_KV290"})
        wrist = motors["CUBEMARS_GL30_KV290"]
        self.assertEqual((wrist["rated_voltage_V"], wrist["operating_voltage_V"]), (12, 16))
        self.assertEqual(wrist["rated_torque_Nm"], .08)
        self.assertIsNone(wrist["peak_duration_s"])
        self.assertEqual(wrist["rotor_inertia_kg_m2"], 2.42e-6)
        gears = {row["reducer_id"]: row for row in bundle["tables"]["reducer.csv"]["rows"]}
        self.assertEqual(set(gears), {"SITO_HD-14_50", "SITO_CD-20_50", "SITO_CD-20_30", "SITO_CS-11_30_USER_ENVELOPE"})
        self.assertEqual(gears["SITO_CD-20_50"]["allowed_joints"], "2")
        self.assertEqual(gears["SITO_CD-20_30"]["allowed_joints"], "3")
        self.assertEqual(gears["SITO_CD-20_50"]["mass_kg"], .591)
        cs = gears["SITO_CS-11_30_USER_ENVELOPE"]
        self.assertEqual((cs["radius_mm"], cs["length_mm"], cs["mass_kg"]), (21.5, 32, .2))
        self.assertEqual(bundle["tables"]["requirement.csv"]["rows"][0]["project_id"], "GL_CD20_550")
        self.assertEqual(len(bundle["tables"]["requirement.csv"]["columns"]), 5)
        with self.assertRaisesRegex(ValueError, "30:1 on J3"):
            compile_locked_inputs(CATALOG_DIR, j3_ratio=50)

    def test_csv_roundtrip_snapshots_curves_and_saved_motor_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            data = Path(directory) / "inputs"
            build_real_catalog(output_dir=data, study="locked-gl")
            inputs = load_inputs(data)
            self.assertEqual(inputs.requirements[0].motor_load_policy, "hold_and_peak_envelope")
            self.assertEqual(inputs.requirements[0].reducer_load_policy, "hold_and_repeated_peak")
            for motor in inputs.motors:
                self.assertGreater(len(motor.peak_torque_speed_curve), 5)
                self.assertEqual(Motor(**json.loads(json.dumps(asdict(motor)))), motor)
            wrist = inputs.motor_by_id["CUBEMARS_GL30_KV290"]
            self.assertIn("voltage_shift_estimate", wrist.peak_curve_basis)
            store = create_run(Path(directory)/"runs", "locked_test", data)
            self.assertEqual((store.path/"inputs/motor_curve.csv").read_bytes(),
                             (data/"motor_curve.csv").read_bytes())
            original = (data/"motor.csv").read_bytes()
            build_real_catalog(output_dir=data, study="locked-gl")
            archives = list((data/"archive").iterdir())
            self.assertEqual(len(archives), 1)
            self.assertEqual((archives[0]/"motor.csv").read_bytes(), original)
            self.assertTrue((archives[0]/"motor_curve.csv").is_file())

    def test_unknown_curve_motor_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            data = Path(directory)
            build_real_catalog(output_dir=data, study="locked-gl")
            with (data/"motor_curve.csv").open("a", encoding="utf-8", newline="") as stream:
                csv.writer(stream).writerow(["UNSELECTED", 0, .2, "test", 16, 16, ""])
            with self.assertRaisesRegex(InputError, "unknown motor_id"):
                load_inputs(data)

    def test_curves_cannot_exceed_peak_or_hide_nonmonotonic_speed(self):
        base = dict(motor_id="test", name="test", rated_torque_Nm=.1, rated_speed_rpm=1000,
            stall_torque_Nm=None, no_load_speed_rpm=2000, max_speed_rpm=None,
            mass_kg=.1, radius_mm=20, length_mm=20, rotor_inertia_kg_m2=1e-6,
            peak_torque_Nm=.3, capacity_model="rated_continuous")
        with self.assertRaisesRegex(ValueError, "exceeds"):
            Motor(**base, peak_torque_speed_curve=((0, .4), (1000, .1)))
        with self.assertRaisesRegex(ValueError, "increasing speed"):
            Motor(**base, peak_torque_speed_curve=((0, .3), (1000, .1), (500, .05)))


if __name__ == "__main__":
    unittest.main()

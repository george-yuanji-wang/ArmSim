"""CSV validation checks for user-editable optimization projects."""

import csv
from dataclasses import asdict
from pathlib import Path
import shutil
import tempfile
import unittest

from optimization.inputs import (
    DATA_DIR, MOTOR_COLUMNS, REDUCER_COLUMNS, REQUIREMENT_COLUMNS, InputError,
    load_inputs, load_motors, load_reducers, load_requirements,
)


def write_rows(path, columns, rows):
    with Path(path).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows({key: row[key] for key in columns} for row in rows)


def motor_row(**updates):
    result = {
        "motor_id": "TEST_ONLY", "name": "Synthetic validation fixture",
        "rated_torque_Nm": 0.2, "rated_speed_rpm": 3000,
        "rated_power_W": "", "stall_torque_Nm": 1,
        "no_load_speed_rpm": 4000, "max_speed_rpm": 5000,
        "mass_kg": 0.3, "radius_mm": 20, "length_mm": 40,
        "rotor_inertia_kg_m2": 0.00001,
        "peak_torque_Nm": "", "peak_duration_s": "",
    }
    result.update(updates)
    return result


class OptimizationInputTests(unittest.TestCase):
    def test_templates_are_readable_but_never_invent_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("requirement.csv", "link_bounds.csv"):
                shutil.copyfile(DATA_DIR / name, root / name)
            write_rows(root / "motor.csv", MOTOR_COLUMNS, [])
            write_rows(root / "reducer.csv", REDUCER_COLUMNS, [])
            inputs = load_inputs(root, require_catalogs=False)
            with self.assertRaisesRegex(InputError, "Fill at least one real catalog row"):
                load_inputs(root)
        self.assertEqual(inputs.motors, ())
        self.assertEqual(inputs.reducers, ())
        self.assertEqual(len(inputs.requirements), 1)
        self.assertEqual(len(REQUIREMENT_COLUMNS), 5)
        self.assertEqual(len(inputs.links), 6)

    def test_equal_requirement_bounds_and_si_conversion(self):
        example = asdict(load_requirements()[0])
        example.update(project_id="fixed_target", payload_lower_g=500,
                       payload_upper_g=500, reach_lower_mm=750, reach_upper_mm=750)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "requirement.csv"
            write_rows(path, REQUIREMENT_COLUMNS, [example])
            requirement = load_requirements(path)[0]
        self.assertEqual(requirement.payload_bounds_kg, (.5, .5))
        self.assertEqual(requirement.reach_bounds_m, (.75, .75))
        self.assertEqual(requirement.allowed_layouts, (7, 8, 9))
        self.assertEqual(requirement.cycles, 200)
        self.assertEqual(requirement.checkpoint_every, 10)
        self.assertEqual(requirement.l1_upper_mm, 675)

    def test_multiple_projects_remain_independent_and_duplicates_rejected(self):
        example = dict(asdict(load_requirements()[0]), payload_upper_g=1000)
        other = dict(example, project_id="another", payload_upper_g=2000)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "requirement.csv"
            write_rows(path, REQUIREMENT_COLUMNS, [example, other])
            self.assertEqual([row.payload_upper_g for row in load_requirements(path)], [1000, 2000])
            write_rows(path, REQUIREMENT_COLUMNS, [example, example])
            with self.assertRaisesRegex(InputError, "duplicate project_id"):
                load_requirements(path)

    def test_reversed_and_nonfinite_bounds_name_the_offending_column(self):
        example = asdict(load_requirements()[0])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "requirement.csv"
            write_rows(path, REQUIREMENT_COLUMNS, [dict(example, reach_lower_mm=1001)])
            with self.assertRaisesRegex(InputError, "reach_lower_mm exceeds reach_upper_mm"):
                load_requirements(path)
            write_rows(path, REQUIREMENT_COLUMNS, [dict(example, payload_lower_g="NaN")])
            with self.assertRaisesRegex(InputError, "payload_lower_g must be finite"):
                load_requirements(path)

    def test_motor_units_and_conservative_optional_peak(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motor.csv"
            write_rows(path, MOTOR_COLUMNS, [motor_row()])
            motor = load_motors(path)[0]
        self.assertEqual(motor.radius_m, .02)
        self.assertEqual(motor.length_m, .04)
        self.assertEqual(motor.peak_torque_nm, motor.rated_torque_nm)
        self.assertIsNone(motor.rated_power_W)

    def test_motor_peak_duration_duplicate_and_missing_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motor.csv"
            write_rows(path, MOTOR_COLUMNS, [motor_row(peak_torque_Nm=.5)])
            with self.assertRaisesRegex(InputError, "peak_duration_s is required"):
                load_motors(path)
            write_rows(path, MOTOR_COLUMNS, [motor_row(), motor_row()])
            with self.assertRaisesRegex(InputError, "duplicate motor_id"):
                load_motors(path)
            write_rows(path, MOTOR_COLUMNS, [motor_row(mass_kg="")])
            with self.assertRaisesRegex(InputError, "mass_kg must contain a number"):
                load_motors(path)

    def test_rated_continuous_motor_preserves_unpublished_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"motor.csv"
            columns = (*MOTOR_COLUMNS, "capacity_model")
            row = motor_row(capacity_model="rated_continuous", stall_torque_Nm="",
                            no_load_speed_rpm="", max_speed_rpm="",
                            peak_torque_Nm=.6, peak_duration_s="")
            write_rows(path, columns, [row])
            motor = load_motors(path)[0]
            self.assertEqual(motor.capacity_model, "rated_continuous")
            self.assertIsNone(motor.stall_torque_Nm)
            self.assertIsNone(motor.no_load_speed_rpm)
            self.assertIsNone(motor.max_speed_rpm)
            self.assertEqual(motor.evaluation_speed_limit_rpm, 3000)
            self.assertEqual(motor.peak_torque_Nm, .6)
            self.assertIsNone(motor.peak_duration_s)
            row["capacity_model"] = "legacy_linear"
            write_rows(path, columns, [row])
            with self.assertRaisesRegex(InputError, "must contain a number"):
                load_motors(path)
            row["capacity_model"] = "unknown_curve"
            write_rows(path, columns, [row])
            with self.assertRaisesRegex(InputError, "capacity_model"):
                load_motors(path)

    def test_reducer_uses_published_output_and_reference_speed_without_input_torque(self):
        row = dict(zip(REDUCER_COLUMNS, (
            "REAL_TEST", "Published output fixture", 40, .08, 24, 20, 8, 12, "", 8500)))
        row["rated_input_speed_rpm"] = 2000
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"reducer.csv"
            write_rows(path, (*REDUCER_COLUMNS, "rated_input_speed_rpm"), [row])
            reducer = load_reducers(path)[0]
        self.assertIsNone(reducer.max_input_torque_Nm)
        self.assertEqual(reducer.output_torque_limit_Nm, 8)
        self.assertEqual(reducer.max_input_speed_rpm, 8500)
        self.assertEqual(reducer.evaluation_input_speed_limit_rpm, 2000)

    def test_infeasible_link_spread_bounds_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for path in DATA_DIR.glob("*.csv"):
                shutil.copyfile(path, root / path.name)
            path = root / "link_bounds.csv"
            with path.open(newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                columns, rows = reader.fieldnames, list(reader)
            rows[0].update(com_fraction_lower=.01, com_fraction_upper=.02,
                           spread_fraction_lower=.1, spread_fraction_upper=.2)
            write_rows(path, columns, rows)
            with self.assertRaisesRegex(InputError, "cannot satisfy kappa"):
                load_inputs(root, require_catalogs=False)


if __name__ == "__main__":
    unittest.main()

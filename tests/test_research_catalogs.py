"""Combined supplier research keeps source facts separate from study inputs."""

from contextlib import redirect_stdout
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

from optimization.catalogs.build_catalogs import (
    INPUTS, NUMERIC_FIELDS, ROOT, build_catalogs,
)
from optimization.inputs import load_motors


def rows(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ResearchCatalogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.output = Path(cls.temporary.name)
        cls.unchanged = {
            path: sha(path) for path in (*INPUTS, ROOT/"motors.csv", ROOT/"motors.json",
                                        ROOT/"catalog_audit.json", ROOT/"motors_optimizer_ready.csv")
        }
        with redirect_stdout(io.StringIO()):
            cls.audit = build_catalogs(output_dir=cls.output)
        cls.records = json.loads((cls.output/"motors.json").read_text(encoding="utf-8"))
        cls.by_id = {row["motor_id"]: row for row in cls.records}

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_all_three_source_catalogs_are_combined_with_existing_legacy_subset(self):
        self.assertEqual(self.audit["motor_records"], 65)
        self.assertEqual(self.audit["manufacturer_record_counts"], {
            "CubeMars": 11, "Nanotec": 11, "maxon": 15, "STEPPERONLINE": 28,
        })
        self.assertEqual(len(self.by_id), 65)
        ready = load_motors(self.output/"motors_optimizer_ready.csv")
        self.assertEqual(len(ready), 15)
        self.assertTrue(all(motor.motor_id.startswith("MAXON_") for motor in ready))
        manifest = {row["path"]: row["sha256"] for row in self.audit["input_catalogs"]}
        self.assertEqual(manifest, {path.relative_to(ROOT).as_posix(): sha(path) for path in INPUTS})
        self.assertEqual(self.audit["retrieval_dates"], ["2026-11-01", "2026-11-04"])

    def test_all_37_existing_supplier_records_retain_their_original_values(self):
        count = 0
        for source in INPUTS[:2]:
            for original in rows(source):
                record = self.by_id[original["motor_id"]]
                for old_key, value in original.items():
                    key = "overall_axial_length_mm" if old_key == "total_axial_envelope_mm" else old_key
                    expected = (float(value) if value not in ("", None) else None) if key in NUMERIC_FIELDS else value
                    self.assertEqual(record[key], expected, (original["motor_id"], key))
                self.assertEqual(record["research_source_catalog"], source.relative_to(ROOT).as_posix())
                self.assertTrue(record["size_filter_pass"])
                count += 1
        self.assertEqual(count, 37)

    def test_all_28_stepperonline_source_facts_are_retained_without_inertia_fabrication(self):
        original = json.loads(INPUTS[2].read_text(encoding="utf-8"))["motors"]
        for source in original:
            record = self.by_id[source["motor_id"]]
            for key, value in source.items():
                self.assertEqual(record[key], value, (source["motor_id"], key))
            self.assertEqual(record["voltage_V"], source["rated_voltage_V"])
            self.assertEqual(record["construction"], source["assembly_type"])
            self.assertEqual(record["research_source_catalog"], "stepperonline/motors.json")
            self.assertIsNone(record["rotor_inertia_kg_m2"])
            self.assertNotIn("rotor_inertia_upper_bound_kg_m2", record)
            self.assertFalse(record["optimizer_ready"])
            self.assertIsNone(record["overall_axial_length_mm"])
            self.assertIn("full envelope not established", record["complete_axial_envelope_status"])

    def test_square_57mm_frame_retains_true_envelope_and_is_flagged_outside_legacy_filter(self):
        square = self.by_id["SO_57BSA79-36-01"]
        self.assertEqual((square["frame_width_mm"], square["frame_height_mm"]), (57., 57.))
        self.assertAlmostEqual(square["cylindrical_envelope_diameter_mm"], math.hypot(57, 57))
        self.assertFalse(square["size_filter_pass"])
        self.assertGreater(self.audit["body_size_filter_counts"]["outside"], 0)
        self.assertIn("lower bound >=100 mm", square["complete_axial_envelope_status"])
        round_motor = self.by_id["SO_57BLR70-36-01"]
        self.assertEqual(round_motor["cylindrical_envelope_diameter_mm"], 57.)
        self.assertIsNone(round_motor["frame_width_mm"])
        self.assertTrue(round_motor["size_filter_pass"])

    def test_nested_source_notes_round_trip_in_csv_and_inputs_are_untouched(self):
        combined = {row["motor_id"]: row for row in rows(self.output/"motors.csv")}
        for identity, source in self.by_id.items():
            if source["manufacturer"] == "STEPPERONLINE":
                self.assertEqual(json.loads(combined[identity]["notes"]), source["notes"])
                self.assertEqual(json.loads(combined[identity]["active_study_exclusion_reasons"]),
                                 source["active_study_exclusion_reasons"])
        self.assertEqual({path: sha(path) for path in self.unchanged}, self.unchanged)


if __name__ == "__main__":
    unittest.main()

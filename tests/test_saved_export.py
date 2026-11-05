"""Re-export archived winners without losing the previous artifact."""

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import mujoco

from optimization.inputs import load_inputs
from optimization.model_factory import GEOMETRY_CONTRACT
from optimization.saved_export import export_saved_run, load_saved_design
from optimization.search import optimize_project


class SavedExportTests(unittest.TestCase):
    def test_legacy_or_invalid_geometry_scores_cannot_be_reexported(self):
        results = (
            [],
            {"status": "completed", "summary": {}},
            {"status": "completed", "summary": {
                "geometry_contract": "old-proxy", "geometry_valid": True}},
            {"status": "completed", "summary": {
                "geometry_contract": "approved-base-template-v2", "geometry_valid": True}},
            {"status": "completed", "summary": {
                "geometry_contract": GEOMETRY_CONTRACT, "geometry_valid": False}},
        )
        for result in results:
            with self.subTest(result=result), tempfile.TemporaryDirectory() as folder:
                run = Path(folder) / "legacy"
                run.mkdir()
                (run / "result.json").write_text(json.dumps(result), encoding="utf-8")
                output = Path(folder) / "model" / "best_model.xml"
                output.parent.mkdir()
                output.write_text("unchanged previous model", encoding="utf-8")
                with patch("optimization.export.export_model", side_effect=AssertionError("legacy winner reached generator")):
                    with self.assertRaisesRegex(ValueError, "Rerun optimization"):
                        export_saved_run(run, output, replace=True)
                self.assertEqual(output.read_text(), "unchanged previous model")
                self.assertFalse((output.parent / "archive").exists())
                self.assertFalse((run / "exports").exists())

    def test_missing_scored_result_does_not_fall_back_to_run_settings(self):
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder) / "unscored"
            run.mkdir()
            (run / "run.json").write_text(json.dumps({"metadata": {
                "geometry_contract": GEOMETRY_CONTRACT, "geometry_valid": True}}))
            output = Path(folder) / "new_model" / "best_model.xml"
            with self.assertRaisesRegex(ValueError, "result.json.*Rerun optimization"):
                export_saved_run(run, output)
            self.assertFalse(output.parent.exists())

    def test_saved_inputs_and_explicit_replacement_preserve_original(self):
        source = Path(__file__).resolve().parents[1] / "optimization/examples/synthetic"
        inputs = load_inputs(source)
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            run = optimize_project(inputs, inputs.requirements[0], source, folder,
                                   cycles=1, pose_limit=1)
            candidate, saved, requirement = load_saved_design(run)
            self.assertEqual(requirement.cycles, 1)
            self.assertIn(candidate.motor_ids[0], inputs.motor_by_id)
            self.assertEqual(saved.motors, inputs.motors)
            original = (run / "best_model.xml").read_bytes()
            appended = export_saved_run(run)
            self.assertEqual((run / "best_model.xml").read_bytes(), original)
            self.assertEqual(mujoco.MjModel.from_xml_path(str(appended)).nu, 6)
            output = Path(folder) / "model" / "best_model.xml"
            output.parent.mkdir()
            output.write_text("original preview", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "already exists"):
                export_saved_run(run, output)
            export_saved_run(run, output, replace=True)
            backups = list((output.parent / "archive").glob("*.xml"))
            self.assertEqual(len(backups), 1)
            self.assertEqual(backups[0].read_text(), "original preview")
            self.assertEqual(mujoco.MjModel.from_xml_path(str(output)).nu, 6)


if __name__ == "__main__":
    unittest.main()

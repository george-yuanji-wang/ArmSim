"""Startup visibility and working-directory-independent launcher checks."""

from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from optimization.__main__ import main


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "run_optimization.py"
DEMO_INPUTS = ROOT / "optimization" / "examples" / "synthetic"


class OptimizationCliTests(unittest.TestCase):
    def test_demo_launcher_runs_from_an_unrelated_working_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            temporary = Path(directory)
            outputs = temporary / "saved_results"
            process = subprocess.run(
                [sys.executable, str(LAUNCHER), "--demo", "--cycles", "1",
                 "--pose-limit", "1", "--runs-dir", str(outputs)],
                cwd=temporary,
                capture_output=True,
                text=True,
                timeout=60,
                check=False,
            )
            self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
            self.assertIn(str(DEMO_INPUTS.resolve()), process.stdout)
            self.assertIn(str(outputs.resolve()), process.stdout)
            for number in (7, 8, 9):
                runs = list((outputs / f"SYNTHETIC_DEMO_{number:02d}").iterdir())
                self.assertEqual(len(runs), 1)
                result = json.loads((runs[0] / "result.json").read_text(encoding="utf-8"))
                self.assertEqual(result["evaluations"], 1)
                self.assertEqual(result["pose_count"], 1)
                self.assertEqual(result["best_model_file"], "best_model.xml")
                design = json.loads((runs[0] / "best_design.json").read_text())
                self.assertEqual(design["layout_number"], number)
                self.assertTrue((runs[0] / "best_model.xml").is_file())
                self.assertTrue((runs[0] / "hallmarks.csv").is_file())

    def test_invalid_inputs_still_print_resolved_paths_and_an_error(self):
        with tempfile.TemporaryDirectory() as directory:
            data = Path(directory) / "missing_inputs"
            outputs = Path(directory) / "outputs"
            stdout, stderr = io.StringIO(), io.StringIO()
            with redirect_stdout(stdout), redirect_stderr(stderr):
                code = main(["optimize", "--data-dir", str(data),
                             "--runs-dir", str(outputs)])
            self.assertEqual(code, 2)
            self.assertIn(str(data.resolve()), stdout.getvalue())
            self.assertIn(str(outputs.resolve()), stdout.getvalue())
            self.assertIn("Input/run error:", stderr.getvalue())
            self.assertIn("Missing input file:", stderr.getvalue())
            self.assertFalse(outputs.exists())


if __name__ == "__main__":
    unittest.main()

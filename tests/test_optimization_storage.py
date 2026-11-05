"""Run persistence must preserve records across reruns and interruptions."""

import csv
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from optimization.storage import create_run, load_checkpoint


class OptimizationStorageTests(unittest.TestCase):
    def test_supplier_assumptions_are_snapshotted_with_their_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            inputs = root/"data"
            inputs.mkdir()
            (inputs/"motor.csv").write_text("motor_id\n", encoding="utf-8")
            assumptions = b'{"unpublished_input_torque":null}\n'
            (inputs/"input_assumptions.json").write_bytes(assumptions)
            store = create_run(root/"runs", "real", inputs)
            self.assertEqual((store.path/"inputs/input_assumptions.json").read_bytes(), assumptions)
            meta = json.loads((store.path/"run.json").read_text())
            self.assertEqual(meta["input_sha256"]["input_assumptions.json"],
                             hashlib.sha256(assumptions).hexdigest())

    def test_reruns_get_unique_folders_and_exact_input_snapshots(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            inputs = root / "data"
            inputs.mkdir()
            source = inputs / "requirement.csv"
            original = b"project_id,enabled\r\ncase,true\r\n"
            source.write_bytes(original)
            first = create_run(root / "runs", "case", inputs)
            second = create_run(root / "runs", "case", inputs)
            self.assertNotEqual(first.path, second.path)
            source.write_text("edited", encoding="utf-8")
            self.assertEqual((first.path / "inputs" / source.name).read_bytes(), original)
            manifest = json.loads((first.path / "inputs" / "manifest.json").read_text())
            self.assertEqual(manifest[source.name]["sha256"], hashlib.sha256(original).hexdigest())
            with self.assertRaises(FileExistsError):
                first.snapshot_inputs(inputs)

    def test_checkpoints_are_exclusive_and_hallmarks_append_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            inputs = root / "data"
            inputs.mkdir()
            (inputs / "motor.csv").write_text("motor_id\n", encoding="utf-8")
            store = create_run(root / "runs", "case", inputs)
            checkpoint = store.checkpoint(10, {"cycle": 10, "best_payload_kg": 1.2})
            self.assertEqual(load_checkpoint(checkpoint)["cycle"], 10)
            with self.assertRaises(FileExistsError):
                store.checkpoint(10, {"cycle": 10})
            store.append_hallmark({"cycle": 0, "payload_kg": .5})
            store.append_hallmark({"payload_kg": 1.2, "cycle": 10})
            with self.assertRaisesRegex(ValueError, "cycle must increase"):
                store.append_hallmark({"cycle": 10, "payload_kg": 9})
            with (store.path / "hallmarks.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([row["cycle"] for row in rows], ["0", "10"])
            self.assertEqual([row["payload_kg"] for row in rows], ["0.5", "1.2"])

    def test_output_paths_cannot_escape_run_and_snapshots_cannot_be_replaced(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            inputs = root / "data"
            inputs.mkdir()
            (inputs / "input.csv").write_text("value\n", encoding="utf-8")
            store = create_run(root / "runs", "safe", inputs)
            with self.assertRaises(ValueError):
                store.write_json("../escape.json", {})
            with self.assertRaises(ValueError):
                store.write_json(root / "absolute.json", {})
            store.write_csv("results/data.csv", ["value"], [{"value": 1}])
            with self.assertRaises(FileExistsError):
                store.write_csv("results/data.csv", ["value"], [{"value": 2}])


if __name__ == "__main__":
    unittest.main()

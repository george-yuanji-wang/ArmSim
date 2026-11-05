"""Independent configuration comparisons must preserve fair study conditions."""

import csv
import json
import math
from pathlib import Path
import tempfile
import unittest

from optimization.comparison import COMPARISON_COLUMNS, write_comparison


class ConfigurationComparisonTests(unittest.TestCase):
    @staticmethod
    def add_enumeration(path, counts=(100, 100)):
        result = json.loads((path/"result.json").read_text())
        tuples = [{"tuple_index": index+1, "layout_number": result["layout_number"],
                   "motor_ids": [f"M{index}"]*6, "allocated_evaluations": 100,
                   "evaluations": count, "attempted": True,
                   "status": ("inner_budget_complete" if count == 100 else
                              "no_supported_seed_in_probe_grid" if count == 0 else "inner_proposal_limit")}
                  for index, count in enumerate(counts)]
        result.update(evaluations=sum(counts), hardware_enumeration={
            "categorical_coverage_complete": True, "hardware_tuple_count": 2, "tuples": tuples})
        (path/"result.json").write_text(json.dumps(result), encoding="utf-8")

    def make_run(self, root, number):
        path = root/f"demo_{number:02d}"/"finished"
        path.mkdir(parents=True)
        lengths = (.20, .09, math.sqrt(.20**2-.09**2))
        requirement = {
            "project_id": f"DEMO_{number:02d}", "payload_lower_g": 500., "payload_upper_g": 1000.,
            "reach_lower_mm": 550., "reach_upper_mm": 550., "seed": 42,
            "joint_speed_rad_s": 1., "joint_accel_rad_s2": 2., "cycles": 200,
            "allowed_layouts": [number],
        }
        meta = {
            "requirement": requirement, "pose_count": 1008, "pose_limit": None,
            "sampling": "same recorded joint-state basis", "reach_definition": "maximum extension",
            "structural_mass_model": "actual aluminium stock", "geometry_contract": "test-v4",
            "python": "3.14", "mujoco": "3.17", "numpy": "2.5",
        }
        result = {
            "status": "completed", "evaluations": 200, "pose_count": 1008, "layout_number": number,
            "extended_reach_mm": 550., "arm_mass_kg": 3.+number/10, "pmax_kg": 1.+number/10,
            "gravity_score_JG": .1+number/100, "requirements_met": True,
            "best_model_file": "best_model.xml",
            "summary": {"geometry_valid": True, "geometry_contract": "test-v4"},
        }
        for name, content in (
                ("run.json", {"metadata": meta, "input_sha256": {"motor.csv": "abc", "reducer.csv": "def"}}),
                ("result.json", result),
                ("best_design.json", {"layout_number": number, "lengths_m": lengths})):
            (path/name).write_text(json.dumps(content), encoding="utf-8")
        with (path/"best_links.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=("body_name", "length_mm", "structural_mass_kg"))
            writer.writeheader()
            writer.writerows({"body_name": name, "length_mm": length*1000, "structural_mass_kg": mass}
                             for name, length, mass in zip(
                                 ("upper_arm_link", "forearm_link", "wrist_1_link"), lengths, (.12, .08, .10)))
        (path/"reference_poses.csv").write_text("pose_id,q1_rad\nstate_1,0\n", encoding="utf-8")
        (path/"best_model.xml").write_text("<mujoco/>", encoding="utf-8")
        (path/"best_joints.csv").write_text("joint,motor_id\nJ1,fixture\n", encoding="utf-8")
        (path/"RESULT.md").write_text("Completed fixture", encoding="utf-8")
        return path

    @staticmethod
    def snapshot(root):
        return {str(path.relative_to(root)): path.read_bytes() for path in root.rglob("*") if path.is_file()}

    def make_detailed_run(self, root, number):
        path = self.make_run(root, number)
        design = json.loads((path/"best_design.json").read_text())
        design.update(motor_ids=["motor_fixture"]*6, reducer_ids=["reducer_fixture"]*6)
        (path/"best_design.json").write_text(json.dumps(design), encoding="utf-8")
        inputs = {
            "motors": [{"motor_id": "motor_fixture", "name": "Complete motor fixture",
                        "rated_voltage_V": 36., "rated_torque_Nm": .2, "rated_speed_rpm": 3000.,
                        "mass_kg": .3, "radius_mm": 20., "length_mm": 40.,
                        "rotor_inertia_kg_m2": None, "rotor_inertia_upper_bound_kg_m2": .00012,
                        "rotor_inertia_basis": "mass_radius_upper_bound"}],
            "reducers": [{"reducer_id": "reducer_fixture", "name": "Reducer fixture",
                          "reduction_ratio": 30., "mass_kg": .2, "continuous_output_torque_Nm": 6.,
                          "max_input_torque_Nm": .1, "rated_input_speed_rpm": 2000.}],
        }
        (path/"evaluated_inputs.json").write_text(json.dumps(inputs), encoding="utf-8")
        poses = []
        for index in range(3):
            pose = {"pose_id": str(index), "reachable": index < 2}
            for joint in range(1, 7):
                pose.update({
                    f"gravity_j{joint}_Nm_at_required_payload": -joint if index == 0 else joint*.5,
                    f"peak_joint_load_j{joint}_Nm_at_required_payload": joint*(index+2),
                    f"peak_motor_torque_j{joint}_Nm_at_required_payload": joint*(index+1)*.1,
                    f"peak_motor_utilization_j{joint}_ratio": joint*(index+1)*.25,
                })
            poses.append(pose)
        with (path/"pose_results.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(poses[0]))
            writer.writeheader()
            writer.writerows(poses)
        return path

    def test_sorted_comparison_has_named_dimensions_and_retains_original_runs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root/"runs"
            runs = [self.make_run(source, number) for number in (9, 7, 8)]
            original = self.snapshot(source)
            output = root/"comparisons"
            first = write_comparison(runs, output)
            second = write_comparison(runs, output)
            self.assertNotEqual(first, second)
            self.assertEqual(self.snapshot(source), original)
            with (first/"comparison.csv").open(encoding="utf-8", newline="") as stream:
                reader = csv.DictReader(stream)
                self.assertEqual(tuple(reader.fieldnames), COMPARISON_COLUMNS)
                rows = list(reader)
            self.assertEqual([int(row["layout_number"]) for row in rows], [7, 8, 9])
            self.assertEqual(float(rows[0]["L2_mm"]), 90.)
            self.assertEqual(float(rows[0]["L2_mass_g"]), 80.)
            self.assertAlmostEqual(float(rows[0]["extended_reach_mm"]), 550.)
            self.assertEqual(Path(rows[0]["model_file"]), runs[1]/"best_model.xml")
            report = (first/"comparison.md").read_text(encoding="utf-8")
            self.assertIn("searched independently", report)
            self.assertIn("full 3D CoM", report)
            self.assertIn("best_links.csv", report)
            self.assertNotIn("joint_loads.csv", report)
            manifest = json.loads((first/"comparison.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["study_conditions"]["seed"], 42)

    def test_complete_saved_details_generate_generic_payload_hardware_and_joint_tables(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runs = [self.make_detailed_run(root/"runs", number) for number in (9, 7, 8)]
            original = self.snapshot(root/"runs")
            output = write_comparison(runs, root/"comparisons")
            with (output/"selected_hardware.csv").open(encoding="utf-8", newline="") as stream:
                hardware = list(csv.DictReader(stream))
            self.assertEqual(len(hardware), 18)
            self.assertEqual((int(hardware[0]["layout_number"]), int(hardware[0]["joint"])), (7, 1))
            self.assertEqual(float(hardware[0]["required_payload_g"]), 1000.)
            self.assertEqual(float(hardware[0]["rated_voltage_V"]), 36.)
            self.assertEqual(float(hardware[0]["rated_torque_Nm"]), .2)
            self.assertEqual(float(hardware[0]["motor_mass_kg"]), .3)
            self.assertEqual(hardware[0]["published_rotor_inertia_kg_m2"], "")
            self.assertEqual(float(hardware[0]["rotor_inertia_upper_bound_kg_m2"]), .00012)
            with (output/"joint_loads.csv").open(encoding="utf-8", newline="") as stream:
                loads = list(csv.DictReader(stream))
            self.assertEqual(len(loads), 18)
            first = loads[0]
            self.assertEqual(int(first["reachable_pose_count"]), 2)
            self.assertEqual(float(first["required_payload_g"]), 1000.)
            self.assertEqual(float(first["max_gravity_torque_Nm"]), 1.)
            self.assertEqual(float(first["max_dynamic_joint_torque_Nm"]), 3.)
            self.assertEqual(float(first["reducer_output_limit_Nm"]), 3.)
            self.assertEqual(float(first["max_reducer_utilization_ratio"]), 1.)
            self.assertAlmostEqual(float(first["max_motor_torque_Nm"]), .2)
            self.assertEqual(float(first["max_motor_utilization_ratio"]), .5)
            report = (output/"comparison.md").read_text(encoding="utf-8")
            self.assertIn("selected_hardware.csv", report)
            self.assertIn("joint_loads.csv", report)
            self.assertIn("1000 g", report)
            self.assertFalse((output/"joint_loads_at_500g.csv").exists())
            self.assertEqual(self.snapshot(root/"runs"), original)

    def test_malformed_detail_files_reject_before_comparison_output_is_created(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runs = [self.make_detailed_run(root/"runs", number) for number in (7, 8, 9)]
            design = json.loads((runs[-1]/"best_design.json").read_text())
            design["motor_ids"] = ["missing_motor"]*6
            (runs[-1]/"best_design.json").write_text(json.dumps(design), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Cannot aggregate saved hardware"):
                write_comparison(runs, root/"new_output")
            self.assertFalse((root/"new_output").exists())

    def test_mismatched_conditions_and_pose_basis_reject_before_output_creation(self):
        for changed in ("joint_speed_rad_s", "seed", "reach_upper_mm", "reference_poses.csv", "algorithm", "evaluation_budget"):
            with self.subTest(changed=changed), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                runs = [self.make_run(root/"runs", number) for number in (7, 8, 9)]
                target = runs[-1]
                if changed == "reference_poses.csv":
                    (target/changed).write_text("different pose basis", encoding="utf-8")
                else:
                    recorded = json.loads((target/"run.json").read_text())
                    if changed in ("algorithm", "evaluation_budget"):
                        recorded["metadata"][changed] = "different search protocol"
                    else:
                        recorded["metadata"]["requirement"][changed] += 1
                    (target/"run.json").write_text(json.dumps(recorded), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, "mismatched study conditions"):
                    write_comparison(runs, root/"new_output")
                self.assertFalse((root/"new_output").exists())

    def test_runs_must_be_completed_distinct_single_configuration_searches(self):
        for changed in ("duplicate", "mixed_search", "incomplete", "short_budget"):
            with self.subTest(changed=changed), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                runs = [self.make_run(root/"runs", number) for number in (7, 8, 9)]
                if changed == "duplicate":
                    runs[-1] = runs[0]
                elif changed == "mixed_search":
                    recorded = json.loads((runs[-1]/"run.json").read_text())
                    recorded["metadata"]["requirement"]["allowed_layouts"] = [7, 8, 9]
                    (runs[-1]/"run.json").write_text(json.dumps(recorded), encoding="utf-8")
                else:
                    result = json.loads((runs[-1]/"result.json").read_text())
                    result["status" if changed == "incomplete" else "evaluations"] = (
                        "interrupted" if changed == "incomplete" else 199)
                    (runs[-1]/"result.json").write_text(json.dumps(result), encoding="utf-8")
                with self.assertRaises(ValueError):
                    write_comparison(runs, root/"new_output")
                self.assertFalse((root/"new_output").exists())

    def test_complete_categorical_coverage_compares_equal_allocations_with_honest_actual_counts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runs = [self.make_run(root/"runs", number) for number in (7, 8, 9)]
            for run, counts in zip(runs, ((0, 100), (50, 100), (100, 100))):
                self.add_enumeration(run, counts)
            output = write_comparison(runs, root/"comparisons")
            with (output/"comparison.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([float(row["evaluations"]) for row in rows], [100, 150, 200])
            self.assertEqual([float(row["allocated_evaluations"]) for row in rows], [200]*3)
            report = (output/"comparison.md").read_text()
            self.assertIn("100 / 200", report)
            self.assertIn("150 / 200", report)

    def test_enumeration_rejects_partial_unfinished_or_unequal_allocations(self):
        for fault in ("partial", "unfinished", "unallocated", "different_plan"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                runs = [self.make_run(root/"runs", number) for number in (7, 8, 9)]
                for run in runs:
                    self.add_enumeration(run)
                result = json.loads((runs[-1]/"result.json").read_text())
                coverage = result["hardware_enumeration"]
                if fault == "partial":
                    coverage["categorical_coverage_complete"] = False
                elif fault == "unfinished":
                    result["evaluations"] = 150
                    coverage["tuples"][0].update(evaluations=50, status="in_progress")
                elif fault == "unallocated":
                    coverage["tuples"][0].update(allocated_evaluations=0, evaluations=0)
                    coverage["tuples"][1]["allocated_evaluations"] = 200
                    result["evaluations"] = 100
                else:
                    coverage["tuples"][0]["allocated_evaluations"] = 99
                    coverage["tuples"][0]["evaluations"] = 99
                    coverage["tuples"][1]["allocated_evaluations"] = 101
                    coverage["tuples"][1]["evaluations"] = 101
                (runs[-1]/"result.json").write_text(json.dumps(result))
                with self.assertRaises(ValueError):
                    write_comparison(runs, root/"new_output")
                self.assertFalse((root/"new_output").exists())

    def test_new_rating_policy_reports_conditional_capacity_and_peak_reducer_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runs = [self.make_detailed_run(root/"runs", number) for number in (7, 8, 9)]
            for run in runs:
                saved = json.loads((run/"run.json").read_text())
                saved["metadata"]["requirement"]["reducer_load_policy"] = "hold_and_repeated_peak"
                (run/"run.json").write_text(json.dumps(saved))
                result = json.loads((run/"result.json").read_text())
                result.update(pmax_known_constraints_kg=4., gravity_capacity_score_JG=.7,
                              hardware_validation_status="conservative_basis_screen_only")
                (run/"result.json").write_text(json.dumps(result))
                inputs = json.loads((run/"evaluated_inputs.json").read_text())
                inputs["reducers"][0].update(max_input_torque_Nm=None, peak_output_torque_Nm=12.)
                (run/"evaluated_inputs.json").write_text(json.dumps(inputs))
            output = write_comparison(runs, root/"comparisons")
            with (output/"comparison.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertTrue(all(float(row["pmax_known_constraints_kg"]) == 4. for row in rows))
            with (output/"joint_loads.csv").open(newline="") as stream:
                loads = list(csv.DictReader(stream))
            self.assertEqual(float(loads[0]["reducer_output_limit_Nm"]), 12.)
            self.assertEqual(float(loads[0]["reducer_holding_limit_Nm"]), 6.)
            self.assertAlmostEqual(float(loads[0]["max_reducer_utilization_ratio"]), .25)
            self.assertIn("omits unverified motor", (output/"comparison.md").read_text())

    def test_zero_payload_ranking_targets_required_task_before_empty_load_and_auxiliary_capacity(self):
        from types import SimpleNamespace
        from optimization.search import _rank
        def score(required, empty, known=1., pmax=0.):
            return SimpleNamespace(pmax_kg=pmax, total_mass_kg=1., reachable_count=1, pose_count=1,
                summary={"peak_required_payload_utilization": required,
                         "peak_empty_arm_utilization": empty, "pmax_known_constraints_kg": known})
        self.assertGreater(_rank(score(2., 1.5)), _rank(score(3., .5)))
        self.assertGreater(_rank(score(2., 1.5, 2.)), _rank(score(2., .5, 1.)))
        self.assertGreater(_rank(score(20., 20., pmax=.1)), _rank(score(1., .1, 100.)))

    def test_score_ranking_ignores_float_noise_but_keeps_meaningful_differences(self):
        from types import SimpleNamespace
        from optimization.search import _rank
        from optimization.comparison import _j2_voltage_rows

        def score(payload=0., required=2., known=1., mass=2.):
            return SimpleNamespace(pmax_kg=payload, total_mass_kg=mass,
                reachable_count=1, pose_count=1,
                summary={"peak_required_payload_utilization": required,
                         "pmax_known_constraints_kg": known})

        cases = (
            # Payload/utilization roundoff must yield to conditional capacity.
            (score(payload=1e-15, required=2.-1e-15, known=.5), score(), 2),
            # Conditional-capacity roundoff must yield to lower mass.
            (score(known=1.+1e-15, mass=3.), score(), 2),
            # Mass roundoff must remain a tie.
            (score(mass=2.+1e-15), score(), 1),
            # Actual score differences must retain their original priority.
            (score(payload=1e-6, required=20., known=.1), score(), 1),
            (score(required=2.-1e-6, known=.1, mass=3.), score(), 1),
            (score(known=1.+1e-6, mass=3.), score(), 1),
            (score(mass=2.-1e-6), score(), 1),
        )
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run/"evaluated_inputs.json").write_text(json.dumps({"motors": [{"motor_id": "M"}]}))
            for first, second, expected in cases:
                with self.subTest(first=first, second=second):
                    ranked = max(enumerate((first, second), 1), key=lambda item: _rank(item[1]))[0]
                    self.assertEqual(ranked, expected)
                    tuples = [{"tuple_index": index, "motor_ids": ["M"]*6,
                        "evaluations": 1, "allocated_evaluations": 1,
                        "best_design": {"lengths_m": [.2, .09, .1]},
                        "best_pmax_kg": value.pmax_kg,
                        "best_peak_required_payload_utilization": value.summary["peak_required_payload_utilization"],
                        "best_pmax_known_constraints_kg": value.summary["pmax_known_constraints_kg"],
                        "best_arm_mass_kg": value.total_mass_kg}
                        for index, value in enumerate((first, second), 1)]
                    (run/"result.json").write_text(json.dumps({"hardware_enumeration": {"tuples": tuples}}))
                    rows = _j2_voltage_rows([{"run_dir": str(run), "layout_number": 7}])
                    self.assertEqual(rows[0]["best_tuple_index"], expected)

    def test_voltage_tradeoff_keeps_best_geometry_for_each_J2_model_without_a_heat_claim(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runs = [self.make_detailed_run(root/"runs", number) for number in (7, 8, 9)]
            for run in runs:
                inputs = json.loads((run/"evaluated_inputs.json").read_text())
                original = inputs["motors"][0]
                original.update(rated_voltage_V=24., rated_current_A=10.4, rated_torque_Nm=.47)
                inputs["motors"] += [dict(original, motor_id="J1_other"), dict(original,
                    motor_id="J2_36", rated_voltage_V=36., rated_current_A=5.4,
                    rated_torque_Nm=.45, mass_kg=.81, length_mm=96.)]
                (run/"evaluated_inputs.json").write_text(json.dumps(inputs))
                design = json.loads((run/"best_design.json").read_text())
                assignments = [["motor_fixture"]*6, ["J1_other"]+["motor_fixture"]*5,
                               ["motor_fixture", "J2_36"]+["motor_fixture"]*4]
                tuples = []
                for index, (assignment, allocated, utilization) in enumerate(zip(
                        assignments, (67, 67, 66), (2., 1.5, 1.8))):
                    tuples.append({"tuple_index": index+1, "layout_number": design["layout_number"],
                        "motor_ids": assignment, "allocated_evaluations": allocated,
                        "evaluations": allocated, "attempted": True, "status": "inner_budget_complete",
                        "best_design": dict(design, motor_ids=assignment), "best_pmax_kg": 0.,
                        "best_pmax_known_constraints_kg": 1.+index/10,
                        "best_peak_required_payload_utilization": utilization,
                        "best_gravity_score_JG": .2, "best_arm_mass_kg": 3.+index/10})
                design["motor_ids"] = assignments[1]
                (run/"best_design.json").write_text(json.dumps(design))
                result = json.loads((run/"result.json").read_text())
                result.update(pmax_kg=0., requirements_met=False, hardware_enumeration={
                    "categorical_coverage_complete": True, "hardware_tuple_count": 3, "tuples": tuples})
                (run/"result.json").write_text(json.dumps(result))
            output = write_comparison(runs, root/"comparison")
            with (output/"J2_voltage_tradeoff.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 6)
            by_model = {row["J2_motor_id"]: row for row in rows if int(row["layout_number"]) == 7}
            self.assertEqual(int(by_model["motor_fixture"]["best_tuple_index"]), 2)
            self.assertEqual(float(by_model["J2_36"]["listed_rated_current_A"]), 5.4)
            self.assertEqual(float(by_model["J2_36"]["motor_body_length_mm"]), 96.)
            self.assertEqual(float(by_model["J2_36"]["best_pmax_known_constraints_kg"]), 1.2)
            self.assertIn("Unpublished", by_model["J2_36"]["current_convention"])
            self.assertIn("J2_voltage_tradeoff.csv", (output/"comparison.md").read_text())


if __name__ == "__main__":
    unittest.main()

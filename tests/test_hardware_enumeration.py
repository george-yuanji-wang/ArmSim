"""Every shortlisted motor tuple gets an independent, balanced geometry search."""

from contextlib import redirect_stdout
from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from optimization.design import initial_candidate
from optimization.inputs import load_inputs
from optimization.physics import Evaluation
from optimization.reach import extended_reach_m, length_coordinates, lengths_from_coordinates
from optimization.search import _HardwareEnumeration, optimize_project


DEMO = Path(__file__).resolve().parents[1]/"optimization/examples/synthetic"


def fixture():
    source = load_inputs(DEMO)
    arm = source.motor_by_id["SYNTHETIC_M_SMALL"]
    wrist = source.motor_by_id["SYNTHETIC_M_WRIST"]
    motors = tuple(replace(arm, motor_id=mid, name=mid, rated_torque_Nm=torque)
                   for mid, torque in (("A", .3), ("B", .4), ("C", .5)))
    motors += (replace(wrist, motor_id="WRIST", name="Fixed shared wrist"),)
    gear = source.reducer_by_id["SYNTHETIC_R_WRIST"]
    reducers = tuple(replace(gear, reducer_id=identity, name=identity,
                             reduction_ratio=30., allowed_joints=joints)
                     for identity, joints in (("G1", (1,)), ("G23", (2, 3)),
                                               ("G456", (4, 5, 6))))
    inputs = replace(source, motors=motors, reducers=reducers)
    requirement = replace(source.requirements[0], allowed_layouts=(7,), cycles=8,
        motor_candidates_by_joint=(("A", "B"), ("C",), ("A", "B")),
        shared_wrist_motor_id="WRIST")
    return inputs, requirement


def fake_evaluate(candidate, inputs, poses, requirement, *, collect_basis=False):
    """A deterministic smooth score isolates the scheduler from physical sizing."""
    score = (10 if candidate.motor_ids[0] == "B" else 0) + (
        2 if candidate.motor_ids[2] == "B" else 0) + candidate.lengths_m[0]
    return Evaluation(score, .2, 2., len(poses), len(poses), True,
        [{"pose_id": pose.pose_id, "reachable": True} for pose in poses], "", {
            "geometry_valid": True, "reach_bounds_pass": True,
            "extended_reach_mm": extended_reach_m(candidate)*1000,
            "peak_empty_arm_utilization": .5,
        })


def continuous_proposal(best, inputs, requirement, rng, proposal_number, *, diagnostics):
    fixed = tuple(group[0] for group in requirement.motor_candidates_by_joint)
    assert best.motor_ids == (*fixed, *(requirement.shared_wrist_motor_id,)*3)
    reach, allocation = length_coordinates(best, requirement)
    allocation += float(rng.uniform(.001, .005))
    diagnostics["selected_block"] = "lengths"
    return replace(best, lengths_m=lengths_from_coordinates(requirement, best.layout_number,
                                                           reach, allocation))


class HardwareEnumerationTests(unittest.TestCase):
    def run_fixture(self, requirement, folder, proposal=continuous_proposal):
        inputs, _ = fixture()
        with redirect_stdout(io.StringIO()), patch("optimization.search.evaluate_candidate", side_effect=fake_evaluate), patch(
                "optimization.search.propose_candidate", side_effect=proposal):
            path = optimize_project(inputs, requirement, DEMO, folder, pose_limit=1)
        return path, json.loads((path/"result.json").read_text(encoding="utf-8")), json.loads(
            (path/"hardware_coverage.json").read_text(encoding="utf-8"))

    def test_two_hundred_evaluations_allocate_twenty_to_each_of_ten_motor_tuples(self):
        _, requirement = fixture()
        requirement = replace(requirement, cycles=200, motor_candidates_by_joint=(
            ("J1A", "J1B"), ("J2",), ("J3A", "J3B", "J3C", "J3D", "J3E")))
        scheduler = _HardwareEnumeration(requirement)
        self.assertEqual(len(scheduler.states), 10)
        self.assertEqual([state["allocated_evaluations"] for state in scheduler.states], [20]*10)
        self.assertTrue(all(state["requirement"].shared_wrist_motor_id == "WRIST"
                            and all(len(group) == 1 for group in state["requirement"].motor_candidates_by_joint)
                            for state in scheduler.states))

    def test_every_tuple_is_evaluated_before_any_tuple_gets_its_second_evaluation(self):
        inputs, requirement = fixture()
        visited = []

        def score(candidate, data, poses, local, *, collect_basis=False):
            if not collect_basis:
                visited.append(candidate.motor_ids[:3])
            return fake_evaluate(candidate, data, poses, local, collect_basis=collect_basis)

        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()), patch(
                "optimization.search.evaluate_candidate", side_effect=score), patch(
                "optimization.search.propose_candidate", side_effect=continuous_proposal):
            path = optimize_project(inputs, requirement, DEMO, folder, pose_limit=1)
            coverage = json.loads((path/"hardware_coverage.json").read_text())
            saved = json.loads((path/"result.json").read_text())
            self.assertTrue((path/"hardware_coverage.csv").is_file())
            self.assertIn("Motor tuple coverage", (path/"RESULT.md").read_text())
        self.assertEqual(len(set(visited[:4])), 4)
        self.assertEqual(visited[:4], visited[4:8])
        self.assertEqual([row["evaluations"] for row in coverage["tuples"]], [2]*4)
        self.assertTrue(coverage["categorical_coverage_complete"])
        self.assertTrue(coverage["load_evaluation_coverage_complete"])
        self.assertTrue(coverage["all_inner_budgets_met"])
        self.assertFalse(coverage["continuous_global_optimality_proven"])
        self.assertEqual(saved["evaluations"], 8)
        self.assertEqual(saved["hardware_enumeration"], coverage)
        # Each proposal's singleton requirement ensures it starts from that
        # tuple's incumbent, even after another tuple has a better global score.
        self.assertEqual(saved["layout_number"], 7)

    def test_short_smoke_budget_explicitly_leaves_unvisited_tuples(self):
        _, requirement = fixture()
        with tempfile.TemporaryDirectory() as folder:
            _, saved, coverage = self.run_fixture(replace(requirement, cycles=2), folder)
        self.assertEqual(saved["evaluations"], 2)
        self.assertEqual(coverage["hardware_tuples_attempted"], 2)
        self.assertEqual(coverage["hardware_tuple_count"], 4)
        self.assertFalse(coverage["categorical_coverage_complete"])
        self.assertFalse(coverage["load_evaluation_coverage_complete"])
        self.assertEqual([row["status"] for row in coverage["tuples"]],
                         ["inner_budget_complete"]*2+["not_allocated"]*2)

    def test_each_tuple_has_its_own_seed_and_reproducible_random_stream(self):
        _, requirement = fixture()
        with tempfile.TemporaryDirectory() as folder:
            _, _, first = self.run_fixture(requirement, folder)
            _, _, second = self.run_fixture(requirement, folder)
        self.assertEqual(first, second)
        self.assertEqual(len({json.dumps(row["rng_state"], sort_keys=True) for row in first["tuples"]}), 4)
        self.assertTrue(all(row["best_design"]["motor_ids"] == row["motor_ids"] for row in first["tuples"]))

    def test_unsupported_seed_is_reported_without_calling_the_tuple_globally_impossible(self):
        _, requirement = fixture()
        original = _HardwareEnumeration._seed

        def seed(scheduler, state, data):
            if state["tuple_index"] == 1:
                raise ValueError("Fixture has no supported seed in the probe grid")
            return original(scheduler, state, data)

        with tempfile.TemporaryDirectory() as folder, patch.object(_HardwareEnumeration, "_seed", seed):
            _, saved, coverage = self.run_fixture(replace(requirement, cycles=4), folder)
        self.assertEqual(saved["evaluations"], 3)
        self.assertFalse(saved["budget_reached"])
        self.assertEqual(saved["termination_reason"], "hardware_inner_budgets_exhausted")
        self.assertTrue(coverage["categorical_coverage_complete"])
        self.assertFalse(coverage["load_evaluation_coverage_complete"])
        self.assertEqual(coverage["tuples"][0]["status"], "no_supported_seed_in_probe_grid")
        self.assertIn("Fixture", coverage["tuples"][0]["seed_error"])
        self.assertIn("not a proof", coverage["seed_failure_scope"])

    def test_duplicate_geometry_does_not_spend_a_tuples_distinct_evaluation_budget(self):
        _, requirement = fixture()

        def duplicate(best, inputs, local, rng, proposal_number, *, diagnostics):
            return best

        with tempfile.TemporaryDirectory() as folder:
            _, saved, coverage = self.run_fixture(requirement, folder, duplicate)
        self.assertEqual(saved["evaluations"], 4)
        self.assertEqual(saved["proposal_attempts"], 80)
        self.assertEqual(saved["duplicate_proposals_skipped"], 76)
        self.assertTrue(coverage["categorical_coverage_complete"])
        self.assertFalse(coverage["all_inner_budgets_met"])
        self.assertEqual([row["evaluations"] for row in coverage["tuples"]], [1]*4)
        self.assertTrue(all(row["status"] == "inner_proposal_limit" for row in coverage["tuples"]))


if __name__ == "__main__":
    unittest.main()

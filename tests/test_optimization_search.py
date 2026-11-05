"""Verify bounded design proposals and sparse durable run integration."""

from pathlib import Path
from dataclasses import replace
import csv
from contextlib import redirect_stdout
import io
import json
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import mujoco

from optimization.design import (Candidate, initial_candidate, propose_candidate,
                                 placement_delta_bounds, template_length_bounds, _capacity)
from optimization.inputs import load_inputs
from optimization.reach import (extended_reach_m, maximum_reach_pose,
                                 length_coordinates, lengths_from_coordinates)


DEMO = Path(__file__).resolve().parents[1] / "optimization" / "examples" / "synthetic"


class DesignSearchTests(unittest.TestCase):
    def setUp(self):
        self.inputs = load_inputs(DEMO)
        self.requirement = replace(self.inputs.requirements[0], allowed_layouts=(7,))

    def restricted_inputs(self):
        """Keep known fitting envelopes but assign catalog rows to joint groups."""
        reducers = tuple(replace(reducer, reducer_id=f"{reducer.reducer_id}_J{group[0]}",
                                 allowed_joints=group)
                         for group in ((1,), (2, 3), (4, 5, 6))
                         for reducer in self.inputs.reducers)
        return replace(self.inputs, reducers=reducers)

    def assert_joint_restrictions(self, candidate, inputs):
        for joint, rid in enumerate(candidate.reducer_ids, 1):
            self.assertIn(joint, inputs.reducer_by_id[rid].allowed_joints)

    def test_restricted_catalog_seeds_and_all_proposal_blocks_obey_joint_assignment(self):
        from optimization.model_factory import build_template_model
        inputs = self.restricted_inputs()
        # Inspect every attempted geometry, including rejected mutations and
        # coordinated upgrades. A valid final candidate alone could hide
        # borrowing another joint's reducer during the search.
        from optimization.design import _compile_candidate
        attempted = []

        def checked(candidate, data):
            self.assert_joint_restrictions(candidate, data)
            attempted.append(candidate)
            return _compile_candidate(candidate, data)

        with patch("optimization.design._compile_candidate", side_effect=checked):
            for number in (7, 8, 9):
                requirement = replace(self.requirement, allowed_layouts=(number,),
                                      reach_lower_mm=550, reach_upper_mm=550)
                candidate = initial_candidate(inputs, requirement)
                self.assert_joint_restrictions(candidate, inputs)
                self.assertEqual(build_template_model(candidate, inputs)[0].nv, 6)
                rng = np.random.default_rng(42)
                for cycle in range(1, 18):
                    candidate = propose_candidate(candidate, inputs, requirement, rng, cycle)
                    self.assert_joint_restrictions(candidate, inputs)
                    self.assertAlmostEqual(extended_reach_m(candidate), .55, places=12)
        self.assertGreater(len(attempted), 60)

    def test_restricted_uniform_seed_and_repair_never_borrow_a_reducer(self):
        from optimization.design import (_fit_initial_hardware, _uniform_hardware_seed,
                                         hardware_pairs)
        inputs = self.restricted_inputs()
        seed = initial_candidate(inputs, self.requirement)
        pairs = hardware_pairs(inputs, self.requirement)

        def checked(candidate, data):
            self.assert_joint_restrictions(candidate, data)
            return candidate

        with patch("optimization.design._compile_candidate", side_effect=checked) as probe:
            uniform = _uniform_hardware_seed(seed, inputs, pairs, self.requirement)
        self.assertIsNotNone(uniform)
        self.assertGreater(probe.call_count, 0)
        # A saved candidate can contain an obsolete assignment. Repair must
        # replace it with an eligible row before calling the model factory.
        wrong = replace(seed, reducer_ids=(seed.reducer_ids[0],)*6)
        with patch("optimization.model_factory.build_template_model", side_effect=checked) as probe:
            repaired = _fit_initial_hardware(wrong, inputs, pairs,
                                             requirement=self.requirement)
        self.assert_joint_restrictions(repaired, inputs)
        self.assertEqual(probe.call_count, 1)

    def test_restricted_pool_uses_each_joints_speed_and_retains_its_own_fallback(self):
        from optimization.design import hardware_pairs, _compile_candidate, _with_hardware
        inputs = self.restricted_inputs()
        motor = replace(inputs.motors[0], max_speed_rpm=2200.)
        # Joint1 has only a ratio100 part and cannot reach its requested speed;
        # J2-6 have a ratio40 part and can. Global preference must retain J1's
        # own diagnostic fallback rather than borrowing those rows.
        reducers = (replace(inputs.reducers[1], reduction_ratio=100.),
                    replace(inputs.reducers[-1], allowed_joints=(2, 3, 4, 5, 6)))
        data = replace(inputs, motors=(motor,), reducers=reducers)
        requirement = replace(self.requirement,
            joint_speed_limits_rad_s=(np.pi,)*3+(5*np.pi/3,)*3)
        all_pairs = hardware_pairs(data, requirement)
        self.assertEqual({r.reducer_id for _, r in all_pairs},
                         {r.reducer_id for r in reducers})
        for joint in range(6):
            self.assertEqual(len(hardware_pairs(data, requirement, joint)), 1)
        # The same motor with ratio50 fits180deg/s but not300deg/s, so a
        # per-joint speed preference must select a different wrist ratio.
        slow = replace(reducers[1], reducer_id="ratio50", reduction_ratio=50.,
                       allowed_joints=())
        fast = replace(reducers[1], reducer_id="ratio40", reduction_ratio=40.,
                       allowed_joints=())
        data = replace(data, motors=(replace(motor, max_speed_rpm=2100.),),
                       reducers=(slow, fast))
        self.assertEqual(len(hardware_pairs(data, requirement, 0)), 2)
        self.assertEqual([r.reducer_id for _, r in hardware_pairs(data, requirement, 5)],
                         ["ratio40"])
        seed = initial_candidate(inputs, self.requirement)
        invalid = replace(seed, reducer_ids=(seed.reducer_ids[0],)*6)
        self.assertIsNone(_compile_candidate(invalid, inputs))
        with self.assertRaisesRegex(ValueError, "not eligible for J2"):
            _with_hardware(seed, 1, (inputs.motors[0], inputs.reducers[0]))
        missing = replace(inputs, reducers=tuple(r for r in inputs.reducers
                                                 if 3 not in r.allowed_joints))
        with self.assertRaisesRegex(ValueError, "J2.*allowed_joints"):
            initial_candidate(missing, self.requirement)

    def test_wrist_span_is_persisted_in_chain_reach_cache_and_design_tables(self):
        from optimization.design import _compile_candidate, _physical_key
        from optimization.model_factory import build_template_model
        from optimization.search import _candidate_key, optimize_project
        requirement = replace(self.requirement, wrist2_length_m=.11,
                              reach_lower_mm=550, reach_upper_mm=550)
        for number in (7, 8, 9):
            with self.subTest(layout=number):
                req = replace(requirement, allowed_layouts=(number,))
                seed = initial_candidate(self.inputs, req)
                self.assertEqual(seed.wrist2_length_m, .11)
                self.assertEqual(Candidate.from_dict(seed.to_dict()), seed)
                model, _ = build_template_model(seed, self.inputs)
                self.assertAlmostEqual(model.body("tool_flange_link").pos[2], .11)
                state = mujoco.MjData(model)
                state.qpos[:] = maximum_reach_pose(seed)
                mujoco.mj_forward(model, state)
                measured = np.linalg.norm(state.site("tool_frame").xpos-[0., 0., .22])
                self.assertAlmostEqual(measured, .55, places=12)
                longer = replace(seed, wrist2_length_m=.12)
                self.assertNotEqual(_physical_key(longer), _physical_key(seed))
                self.assertNotEqual(_candidate_key(longer), _candidate_key(seed))
                fitted = _compile_candidate(longer, self.inputs)
                self.assertIsNotNone(fitted)
                self.assertEqual(fitted.wrist2_length_m, .12)
                self.assertAlmostEqual(extended_reach_m(fitted), .56, places=12)
        # Older saved candidates remain readable with the original70mm span.
        old_record = seed.to_dict()
        old_record.pop("wrist2_length_m")
        self.assertEqual(Candidate.from_dict(old_record).wrist2_length_m, .07)
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            run = optimize_project(self.inputs, requirement, DEMO, folder,
                                   cycles=1, pose_limit=1)
            saved = json.loads((run/"best_design.json").read_text())
            self.assertEqual(saved["wrist2_length_m"], .11)
            with (run/"best_links.csv").open(newline="") as stream:
                rows = {row["body_name"]: row for row in csv.DictReader(stream)}
            self.assertEqual(float(rows["wrist_2_link"]["length_mm"]), 110.)
            self.assertEqual(json.loads((run/"result.json").read_text())["wrist2_length_mm"], 110.)

    def test_active_framed_hardware_has_complete_supported_seeds_and_preserves_study_span(self):
        from optimization.model_factory import build_template_model, GeometryError
        active = load_inputs(Path(__file__).resolve().parents[1]/"optimization/data/real")
        if not active.motors or not all(m.motor_id.startswith("SO_") for m in active.motors):
            self.skipTest("The framed-motor study has not been prepared")
        expected = ("SITO_HD-14_30", "SITO_CD-20_30", "SITO_CD-20_30",
                    "SITO_CS-11-2XH-A_30", "SITO_CS-11-2XH-A_30", "SITO_CS-11-2XH-A_30")
        for number in (7, 8, 9):
            with self.subTest(layout=number):
                requirement = replace(active.requirements[0], allowed_layouts=(number,))
                seed = initial_candidate(active, requirement)
                self.assertEqual(seed.reducer_ids, expected)
                self.assertEqual(seed.motor_ids[3:], ("SO_42BSB41-24-01",)*3)
                self.assertEqual(seed.wrist2_length_m, .11)
                self.assertEqual(build_template_model(seed, active)[0].nv, 6)
                # The original span cannot surround these complete cases. It
                # must remain rejected rather than acquiring weakened checks.
                with self.assertRaises(GeometryError):
                    build_template_model(replace(seed, wrist2_length_m=.07), active)
                rng = np.random.default_rng(11)
                for cycle in (1, 2, 16, 17):
                    seed = propose_candidate(seed, active, requirement, rng, cycle)
                    self.assertEqual(seed.wrist2_length_m, .11)
                    self.assertEqual(seed.reducer_ids, expected)
                    self.assertEqual(seed.motor_ids[3:], ("SO_42BSB41-24-01",)*3)
                    self.assertAlmostEqual(extended_reach_m(seed), .55, places=12)
                    self.assertEqual(build_template_model(seed, active)[0].nv, 6)

    def test_catalog_seeds_fit_each_approved_base_family(self):
        from optimization.model_factory import build_template_model
        for number in (7, 8, 9):
            with self.subTest(layout=number):
                seed = initial_candidate(self.inputs, replace(self.requirement, allowed_layouts=(number,)))
                model, _ = build_template_model(seed, self.inputs)
                self.assertEqual(model.nv, 6)
                data = mujoco.MjData(model)
                data.qpos[:] = maximum_reach_pose(seed)
                mujoco.mj_forward(model, data)
                observed = np.linalg.norm(data.site("tool_frame").xpos-np.array([0., 0., .22]))
                self.assertAlmostEqual(observed, extended_reach_m(seed), places=12)
                self.assertAlmostEqual(observed, seed.lengths_m[0]+np.hypot(.09, seed.lengths_m[2])+.15)

    def test_real_catalog_seeds_fit_complete_fixed_reach_assemblies(self):
        from optimization.model_factory import build_template_model
        from optimization.real_catalog import build_real_catalog
        # This regression exercises the retained multi-motor SO study,
        # independently of whichever hardware the user currently selects.
        with tempfile.TemporaryDirectory() as directory:
            build_real_catalog(output_dir=directory, study="stepperonline")
            inputs = load_inputs(directory)
        for number in (7, 8, 9):
            with self.subTest(layout=number):
                requirement = replace(inputs.requirements[0], allowed_layouts=(number,))
                seed = initial_candidate(inputs, requirement)
                model, _ = build_template_model(seed, inputs)
                self.assertEqual(model.nv, 6)
                self.assertEqual(seed.layout_number, number)
                self.assertAlmostEqual(extended_reach_m(seed), .55, places=12)
                self.assertEqual(seed.lengths_m[1], .09)
                self.assertTrue(all(name.startswith("SO_") for name in seed.motor_ids))
                self.assertEqual(seed.reducer_ids,
                                 ("SITO_HD-14_30", "SITO_CD-20_30", "SITO_CD-20_30",
                                  "SITO_CS-11-2XH-A_30", "SITO_CS-11-2XH-A_30",
                                  "SITO_CS-11-2XH-A_30"))
                self.assertEqual(seed.wrist2_length_m, .11)
                self.assertEqual(seed.motor_ids[3:], ("SO_42BSB41-24-01",)*3)
                for joint, rid in enumerate(seed.reducer_ids, 1):
                    self.assertIn(joint, inputs.reducer_by_id[rid].allowed_joints)

    def test_real_capacity_challenge_upgrades_a_motor_while_preserving_fixed_reducers(self):
        from optimization.model_factory import build_template_model
        from optimization.real_catalog import build_real_catalog
        with tempfile.TemporaryDirectory() as directory:
            build_real_catalog(output_dir=directory, study="stepperonline")
            inputs = load_inputs(directory)
        requirement = replace(inputs.requirements[0], allowed_layouts=(9,))
        # This screened lower-torque candidate remains eligible on J3. A
        # fitting stronger package must be discovered without changing the
        # prescribed reducer or the shared wrist motor model. Construct its
        # own supported placement: transplanting it into the stronger motor's
        # placement can correctly collide with a cage edge.
        weak_scope = replace(requirement, motor_candidates_by_joint=(
            *requirement.motor_candidates_by_joint[:2], ("SO_42BSA62-36-01",)))
        weaker = initial_candidate(inputs, weak_scope)
        self.assertEqual(build_template_model(weaker, inputs)[0].nv, 6)
        before = _capacity((inputs.motor_by_id[weaker.motor_ids[2]],
                            inputs.reducer_by_id[weaker.reducer_ids[2]]))
        diagnostics = {}
        upgrade = propose_candidate(weaker, inputs, requirement,
                                    np.random.default_rng(42), 16, diagnostics=diagnostics)
        after = _capacity((inputs.motor_by_id[upgrade.motor_ids[2]],
                           inputs.reducer_by_id[upgrade.reducer_ids[2]]))
        self.assertGreater(after, before)
        self.assertEqual(upgrade.reducer_ids, weaker.reducer_ids)
        self.assertEqual(upgrade.motor_ids[3:], ("SO_42BSB41-24-01",)*3)
        self.assertEqual(upgrade.lengths_m, weaker.lengths_m)
        self.assertEqual(upgrade.motor_offsets_m, weaker.motor_offsets_m)
        self.assertEqual(diagnostics["selected_block"], "coordinated_capacity_challenge")
        self.assertEqual(build_template_model(upgrade, inputs)[0].nv, 6)

    def test_proposals_obey_bounds_and_placement_freedoms(self):
        for number in (7, 8, 9):
            rng = np.random.default_rng(17)
            requirement = replace(self.requirement, allowed_layouts=(number,))
            candidate = initial_candidate(self.inputs, requirement)
            for cycle in range(1, 51):
                candidate = propose_candidate(candidate, self.inputs, requirement, rng, cycle)
                self.assertEqual(candidate.layout_number, number)
                self.assertEqual(candidate.lengths_m[1], .09)
                self.assertEqual(Candidate.from_dict(candidate.to_dict()), candidate)
                self.assertGreaterEqual(extended_reach_m(candidate), requirement.reach_lower_m-1e-12)
                self.assertLessEqual(extended_reach_m(candidate), requirement.reach_upper_m+1e-12)
                for value, (low, high) in zip(candidate.lengths_m, template_length_bounds(
                        requirement, candidate.layout_number)):
                    self.assertLessEqual(low-1e-12, value)
                    self.assertLessEqual(value, high+1e-12)
                self.assertTrue(np.all(np.abs(candidate.motor_offsets_m) <=
                                       placement_delta_bounds(candidate.lengths_m, requirement)))
                if number == 7:
                    self.assertEqual(candidate.motor_offsets_m, (0., 0., 0., 0.))

    def test_fixed_550_proposals_spend_load_budget_on_supported_geometry(self):
        from optimization.model_factory import build_template_model
        rejected = 0
        for number in (7, 8, 9):
            requirement = replace(self.requirement, allowed_layouts=(number,),
                                  reach_lower_mm=550, reach_upper_mm=550)
            current = initial_candidate(self.inputs, requirement)
            rng = np.random.default_rng(19)
            for cycle in range(1, 13):
                diagnostics = {}
                proposal = propose_candidate(current, self.inputs, requirement, rng, cycle,
                                             diagnostics=diagnostics)
                self.assertTrue(diagnostics["load_evaluation_ready"])
                self.assertFalse(diagnostics["fallback_to_incumbent"])
                self.assertAlmostEqual(extended_reach_m(proposal), .55, places=12)
                self.assertEqual(proposal.lengths_m[1], .09)
                self.assertEqual(proposal.layout_number, number)
                self.assertEqual(build_template_model(proposal, self.inputs)[0].nv, 6)
                rejected += diagnostics["geometry_rejected_trials"]
                current = proposal
        self.assertGreater(rejected, 0)

    def test_capacity_challenge_restores_a_previously_downsized_drive(self):
        from optimization.structural_mass import canonicalize_candidate
        requirement = replace(self.requirement, allowed_layouts=(9,),
                              reach_lower_mm=550, reach_upper_mm=550)
        strong = initial_candidate(self.inputs, requirement)
        motors = list(strong.motor_ids)
        motors[1] = "SYNTHETIC_M_WRIST"
        small = canonicalize_candidate(replace(strong, motor_ids=tuple(motors)), self.inputs)
        before = _capacity((self.inputs.motor_by_id[small.motor_ids[1]],
                            self.inputs.reducer_by_id[small.reducer_ids[1]]))
        diagnostics = {}
        challenge = propose_candidate(small, self.inputs, requirement,
                                      np.random.default_rng(42), 16, diagnostics=diagnostics)
        after = _capacity((self.inputs.motor_by_id[challenge.motor_ids[1]],
                           self.inputs.reducer_by_id[challenge.reducer_ids[1]]))
        self.assertEqual(before, 22.)
        self.assertEqual(after, 120.)
        self.assertEqual(challenge.lengths_m, small.lengths_m)
        self.assertEqual(challenge.motor_offsets_m, small.motor_offsets_m)
        self.assertEqual(diagnostics["selected_block"], "coordinated_capacity_challenge")

    def test_large_catalog_repair_covers_compact_options_beyond_strong_prefix(self):
        from optimization.design import _fit_initial_hardware, _strongest_fitting_hardware
        from armsim.mechanism_parameters import GeometryError
        seed = initial_candidate(self.inputs, self.requirement)
        oversized = tuple(replace(self.inputs.reducers[0],
                                  reducer_id=f"oversized_{index:03d}",
                                  reduction_ratio=1000.+index, radius_mm=150.+index,
                                  length_mm=150.+index, continuous_output_torque_Nm=1e6,
                                  peak_output_torque_Nm=1e6, max_input_torque_Nm=1e6,
                                  max_input_speed_rpm=1e6)
                          for index in range(160))
        expanded = replace(self.inputs, reducers=(*oversized, *self.inputs.reducers))
        # Keep all pairs so the fixture isolates repair coverage rather than
        # shaft-speed screening. The first96 capacity-ranked entries are bad.
        pairs = [(motor, reducer) for motor in expanded.motors for reducer in expanded.reducers]
        self.assertTrue(all(pair[1].reducer_id.startswith("oversized_")
                            for pair in sorted(pairs, key=_capacity, reverse=True)[:96]))
        broken = replace(seed, reducer_ids=(oversized[0].reducer_id,)*6)

        def compile_fixture(candidate, inputs):
            for joint, rid in enumerate(candidate.reducer_ids, 1):
                if rid.startswith("oversized_"):
                    raise GeometryError(f"J{joint} catalog envelope cannot fit")
            return None, "supported fixture"

        with patch("optimization.model_factory.build_template_model", side_effect=compile_fixture) as probe:
            repaired = _fit_initial_hardware(broken, expanded, pairs)
        self.assertTrue(all(not rid.startswith("oversized_") for rid in repaired.reducer_ids))
        self.assertLessEqual(probe.call_count, 96)
        small = replace(seed, motor_ids=("SYNTHETIC_M_WRIST",)*6,
                        reducer_ids=("SYNTHETIC_R_WRIST",)*6)

        def fitted(candidate, inputs):
            return None if any(rid.startswith("oversized_") for rid in candidate.reducer_ids) else candidate

        diagnostics = {}
        with patch("optimization.design._compile_candidate", side_effect=fitted):
            improved = _strongest_fitting_hardware(small, expanded, pairs, diagnostics=diagnostics)
        capacities = [_capacity((expanded.motor_by_id[mid], expanded.reducer_by_id[rid]))
                      for mid, rid in zip(improved.motor_ids, improved.reducer_ids)]
        self.assertEqual(capacities, [120.]*6)
        self.assertLessEqual(diagnostics["capacity_challenge_probes"], 96)

    def test_exhausted_proposal_work_exposes_its_incumbent_fallback(self):
        seed = initial_candidate(self.inputs, self.requirement)
        diagnostics = {}
        with patch("optimization.design._compile_candidate", return_value=None):
            proposal = propose_candidate(seed, self.inputs, self.requirement,
                                         np.random.default_rng(42), 3, diagnostics=diagnostics)
        self.assertEqual(proposal, seed)
        self.assertEqual(diagnostics["block_trials"], 24)
        self.assertTrue(diagnostics["fallback_to_incumbent"])
        self.assertIn("24 bounded geometry trials", diagnostics["fallback_reason"])

    def test_sparse_checkpoints_and_fresh_rerun_folders(self):
        from optimization.search import optimize_project
        # Use the real evaluator to construct a tiny fixture independent of
        # Evaluation's optional detailed-output fields.
        from optimization.physics import evaluate_candidate
        from optimization.poses import generate_poses
        result = evaluate_candidate(initial_candidate(self.inputs, self.requirement), self.inputs,
                                    generate_poses(self.requirement, limit=1), self.requirement)
        with tempfile.TemporaryDirectory() as folder:
            with patch("optimization.search.evaluate_candidate", return_value=result):
                path1 = optimize_project(self.inputs, self.requirement, DEMO, folder,
                                         cycles=5, checkpoint_every=2, pose_limit=1)
                path2 = optimize_project(self.inputs, self.requirement, DEMO, folder,
                                         cycles=1, checkpoint_every=2, pose_limit=1)
            self.assertNotEqual(path1, path2)
            self.assertEqual(len(list((path1 / "checkpoints").glob("*.json"))), 4)
            self.assertTrue((path1 / "inputs" / "motor.csv").exists())
            self.assertTrue((path1 / "result.json").exists())
            self.assertTrue((path1 / "best_model.xml").exists())
            self.assertTrue((path1 / "best_links.csv").exists())
            self.assertTrue((path1 / "best_joints.csv").exists())
            self.assertTrue((path1 / "evaluated_inputs.json").exists())
            import mujoco
            exported = mujoco.MjModel.from_xml_path(str(path1 / "best_model.xml"))
            self.assertEqual(exported.njnt, 6)
            self.assertEqual(exported.nu, 6)
            result_info = json.loads((path1 / "result.json").read_text())
            self.assertEqual(result_info["best_model_file"], "best_model.xml")

    def test_default_budget_saves_21_hallmarks_and_no_per_candidate_history(self):
        from optimization.physics import evaluate_candidate
        from optimization.poses import generate_poses
        from optimization.search import optimize_project
        candidate = initial_candidate(self.inputs, self.requirement)
        result = evaluate_candidate(candidate, self.inputs,
                                    generate_poses(self.requirement, limit=1), self.requirement)
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            with patch("optimization.search.evaluate_candidate", return_value=result):
                path = optimize_project(self.inputs, replace(self.requirement, cycles=200, checkpoint_every=10),
                                        DEMO, folder, pose_limit=1)
            with (path / "hallmarks.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([int(row["cycle"]) for row in rows], [1, *range(10, 201, 10)])
            self.assertEqual(len(list((path / "checkpoints").glob("*.json"))), 21)
            self.assertFalse((path / "all_evaluations.csv").exists())

    def test_duplicate_physical_proposals_do_not_consume_load_budget(self):
        from optimization.physics import evaluate_candidate
        from optimization.poses import generate_poses
        from optimization.search import optimize_project
        seed = initial_candidate(self.inputs, self.requirement)
        reach, allocation = length_coordinates(seed, self.requirement)
        fresh = replace(seed, lengths_m=lengths_from_coordinates(
            self.requirement, 7, reach, allocation+.01))
        same_physics = replace(seed, link_masses_kg=tuple(2*m for m in seed.link_masses_kg))
        result = evaluate_candidate(seed, self.inputs,
                                    generate_poses(self.requirement, limit=1), self.requirement)
        improved = replace(result, pmax_kg=result.pmax_kg+.1)
        trials = iter((seed, same_physics, fresh))

        def propose(best, inputs, requirement, rng, proposal_number, *, diagnostics):
            trial = next(trials)
            diagnostics.update(geometry_rejected_trials=2, block_trials=3,
                               fallback_to_incumbent=trial is not fresh,
                               fallback_reason="fixture fallback" if trial is not fresh else "",
                               capacity_challenge_probes=5 if trial is fresh else 0,
                               selected_block="hardware")
            return trial

        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            with patch("optimization.search.propose_candidate", side_effect=propose), \
                    patch("optimization.search.evaluate_candidate",
                          side_effect=(result, improved, improved)) as evaluate:
                path = optimize_project(self.inputs, self.requirement, DEMO, folder,
                                        cycles=2, checkpoint_every=1, pose_limit=1)
            saved = json.loads((path/"result.json").read_text())
            self.assertEqual(evaluate.call_count, 3)  # two designs plus final detailed output
            self.assertEqual(saved["evaluations"], 2)
            self.assertEqual(saved["requested_evaluations"], 2)
            self.assertTrue(saved["budget_reached"])
            self.assertEqual(saved["proposal_attempts"], 4)
            self.assertEqual(saved["unique_candidates_checked"], 2)
            self.assertEqual(saved["duplicate_proposals_skipped"], 2)
            self.assertEqual(saved["proposal_diagnostics"]["geometry_rejected_trials"], 6)
            self.assertEqual(saved["proposal_diagnostics"]["fallback_to_incumbent"], 2)
            self.assertEqual(saved["proposal_diagnostics"]["capacity_challenge_probes"], 5)
            checkpoint = json.loads((path/"checkpoints/cycle_000002.json").read_text())
            self.assertEqual(checkpoint["proposal_diagnostics"], saved["proposal_diagnostics"])

    def test_proposal_limit_reports_short_budget_and_retains_the_best(self):
        from optimization.physics import evaluate_candidate
        from optimization.poses import generate_poses
        from optimization.search import optimize_project
        seed = initial_candidate(self.inputs, self.requirement)
        result = evaluate_candidate(seed, self.inputs,
                                    generate_poses(self.requirement, limit=1), self.requirement)

        def saturated(best, inputs, requirement, rng, proposal_number, *, diagnostics):
            diagnostics.update(fallback_to_incumbent=True, fallback_reason="No neighbor in fixture")
            return seed

        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            with patch("optimization.search.propose_candidate", side_effect=saturated), \
                    patch("optimization.search.evaluate_candidate", return_value=result) as evaluate:
                path = optimize_project(self.inputs, self.requirement, DEMO, folder,
                                        cycles=3, checkpoint_every=10, pose_limit=1)
            saved = json.loads((path/"result.json").read_text())
            self.assertEqual(evaluate.call_count, 2)  # seed and its final detailed output
            self.assertEqual(saved["status"], "completed")
            self.assertEqual(saved["evaluations"], 1)
            self.assertEqual(saved["requested_evaluations"], 3)
            self.assertFalse(saved["budget_reached"])
            self.assertEqual(saved["proposal_attempts"], 30)
            self.assertEqual(saved["duplicate_proposals_skipped"], 29)
            self.assertEqual(saved["termination_reason"], "proposal_attempt_limit_reached")
            self.assertTrue((path/"best_model.xml").exists())
            self.assertTrue((path/"checkpoints/final_after_proposal_000030.json").exists())
            report = (path/"RESULT.md").read_text()
            self.assertIn("1/3 requested", report)
            self.assertIn("proposal_attempt_limit_reached", report)

    def test_fixed_straightened_reach_is_enforced_and_persisted(self):
        from optimization.search import optimize_project
        requirement = replace(self.requirement, reach_lower_mm=600, reach_upper_mm=600)
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            path = optimize_project(self.inputs, requirement, DEMO, folder, cycles=1, pose_limit=1)
            result = json.loads((path / "result.json").read_text())
            self.assertEqual(result["layout_number"], 7)
            self.assertAlmostEqual(result["extended_reach_mm"], 600, places=8)
            self.assertTrue(result["summary"]["reach_bounds_pass"])
            self.assertEqual(result["reachable_poses"], 1)
            self.assertIsNotNone(result["gravity_score_JG"])
            self.assertIn("extended", (path / "RESULT.md").read_text())
            saved = Candidate.from_dict(json.loads((path/"best_design.json").read_text()))
            self.assertEqual(saved.lengths_m[1], .09)
            self.assertAlmostEqual(extended_reach_m(saved), .6, places=12)

    def test_unsupported_geometry_cannot_win_or_produce_a_best_model(self):
        from optimization.physics import Evaluation
        from optimization.search import _rank, optimize_project
        rejected = Evaluation(0., None, 0., 0, 1, False, [], "", {
            "geometry_valid": False, "geometry_error": "No room for approved wrist mount"})
        supported = replace(rejected, total_mass_kg=10., summary={"geometry_valid": True})
        self.assertGreater(_rank(supported), _rank(rejected))
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            with patch("optimization.search.evaluate_candidate", return_value=rejected):
                with self.assertRaisesRegex(ValueError, "No candidate fits the approved base geometry"):
                    optimize_project(self.inputs, self.requirement, DEMO, folder, cycles=2, pose_limit=1)
            run = next((Path(folder)/self.requirement.project_id).iterdir())
            self.assertTrue((run/"failed.json").exists())
            self.assertFalse((run/"best_model.xml").exists())
            checkpoint = json.loads(next((run/"checkpoints").iterdir()).read_text())
            self.assertGreaterEqual(checkpoint["geometry_rejection_counts"]["No room for approved wrist mount"], 1)


if __name__ == "__main__":
    unittest.main()

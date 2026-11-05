"""Finite seeded optimization with sparse, persistent checkpoint records."""

from collections import Counter
from dataclasses import replace
from itertools import product
import math
import platform
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .design import (initial_candidate, propose_candidate, _compile_candidate,
                     _fit_initial_placement)
from .export import export_model
from .inputs import STRUCTURAL_BODIES, to_jsonable
from .model_factory import GEOMETRY_CONTRACT, build_template_model, GeometryError
from .reach import extended_reach_m, coordinate_bounds, lengths_from_coordinates
from .structural_mass import structural_properties, STRUCTURAL_MASS_MODEL
from armsim.catalog import layout_by_number
from .physics import evaluate_candidate
from .poses import POSE_COLUMNS, SAMPLING_DESCRIPTION, generate_poses, pose_rows
from .storage import create_run
from .settings import RANK_SCORE_DECIMALS


def _finite(value):
    """Keep JSON standard-compliant if a diagnostic is not defined."""
    if isinstance(value, dict):
        return {key: _finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite(item) for item in value]
    if isinstance(value, (float, np.floating)) and not math.isfinite(value):
        return None
    return to_jsonable(value)


def _rank(evaluation):
    """Rank selected-policy payload first; guide zero ties toward the required task."""
    payload = round(evaluation.pmax_kg, RANK_SCORE_DECIMALS)
    required_progress = (-round(evaluation.summary.get("peak_required_payload_utilization",
        evaluation.summary.get("peak_empty_arm_utilization", math.inf)), RANK_SCORE_DECIMALS)
        if payload <= 0 else 0.)
    known_progress = (round(evaluation.summary.get("pmax_known_constraints_kg", 0.), RANK_SCORE_DECIMALS)
                      if payload <= 0 else 0.)
    return (bool(evaluation.summary.get("geometry_valid", True)),
            bool(evaluation.summary.get("reach_bounds_pass", True)),
            evaluation.reachable_count / max(evaluation.pose_count, 1),
            payload, required_progress, known_progress,
            -round(evaluation.total_mass_kg, RANK_SCORE_DECIMALS))


def _number(value, digits=4):
    return "unavailable" if value is None else f"{value:.{digits}g}"


def _candidate_key(candidate):
    """Deduplicate physical choices, excluding derived mass-property fields."""
    return (candidate.layout_number,
            tuple(round(value, 12) for value in candidate.lengths_m),
            tuple(round(value, 12) for value in candidate.motor_offsets_m),
            candidate.motor_ids, candidate.reducer_ids,
            round(candidate.wrist2_length_m, 12))


def _write_rows(store, name, rows):
    if not rows:
        return
    fields = list(dict.fromkeys(key for row in rows for key in row))
    store.write_csv(name, fields, ({key: row.get(key, "") for key in fields} for row in rows))


class _HardwareEnumeration:
    """Balanced discrete motor coverage with a separate geometry search per tuple."""

    def __init__(self, requirement):
        choices = list(product(requirement.allowed_layouts,
                               *requirement.motor_candidates_by_joint))
        quotient, remainder = divmod(requirement.cycles, len(choices))
        self.states = []
        self.cursor = 0
        for index, (layout, *motors) in enumerate(choices):
            budget = quotient + (index < remainder)
            motor_ids = (*motors, *(requirement.shared_wrist_motor_id,)*3)
            local = replace(requirement, allowed_layouts=(layout,), cycles=max(1, budget),
                            motor_candidates_by_joint=tuple((mid,) for mid in motors))
            self.states.append({
                "tuple_index": index+1, "layout_number": layout, "motor_ids": motor_ids,
                "allocated_evaluations": budget, "evaluations": 0, "proposal_attempts": 0,
                "duplicate_proposals": 0, "geometry_rejections": 0, "reach_rejections": 0,
                "seed_probes": 0, "seed_error": None, "attempted": False,
                "requirement": local, "candidate": None, "evaluation": None,
                "rng": np.random.default_rng(np.random.SeedSequence([requirement.seed, index, layout])),
                "proposal_inputs": None,
            })

    @staticmethod
    def _finished(state):
        return (not state["allocated_evaluations"] or state["seed_error"] is not None or
                state["evaluations"] >= state["allocated_evaluations"] or
                state["proposal_attempts"] >= 10*state["allocated_evaluations"])

    @property
    def proposal_attempts(self):
        return sum(state["proposal_attempts"] for state in self.states)

    def _seed(self, state, inputs):
        """Do not equate a bad middle allocation with an impossible motor tuple."""
        local = state["requirement"]
        seed = initial_candidate(inputs, local)
        probes = [seed]
        lower, upper = coordinate_bounds(local, state["layout_number"])
        # The initial candidate already probes the middle allocation and nearby
        # placements. Add a fixed small reach/allocation grid before reporting
        # that this tuple has no supported starting geometry.
        for reach in dict.fromkeys(((lower+upper)/2, lower, upper)):
            for allocation in (0., .25, .75, 1.):
                probes.append(replace(seed, lengths_m=lengths_from_coordinates(
                    local, state["layout_number"], reach, allocation)))
        last_error = "No supported starting geometry in the bounded seed probes"
        seen = set()
        for trial in probes:
            key = _candidate_key(trial)
            if key in seen:
                continue
            seen.add(key)
            state["seed_probes"] += 1
            fitted = _compile_candidate(trial, inputs)
            if fitted is None:
                fitted = _fit_initial_placement(trial, inputs, local)
            if fitted is not None:
                return fitted
            try:
                build_template_model(trial, inputs)
            except GeometryError as exc:
                last_error = str(exc)
        state["seed_error"] = last_error
        return seed  # The regular evaluator records its unsupported geometry.

    def next(self, inputs):
        for _ in self.states:
            state = self.states[self.cursor]
            self.cursor = (self.cursor+1) % len(self.states)
            if self._finished(state):
                continue
            state["attempted"] = True
            state["proposal_attempts"] += 1
            diagnostics = {}
            if state["candidate"] is None:
                try:
                    candidate = self._seed(state, inputs)
                except (ValueError, KeyError) as exc:
                    state["seed_error"] = f"{type(exc).__name__}: {exc}"
                    continue
                state["candidate"] = candidate
                # Freeze reducer assignments too. Singleton motor scopes alone
                # would still allow an unrestricted catalog to change ratios.
                reducers = tuple(replace(inputs.reducer_by_id[rid], allowed_joints=tuple(
                    joint for joint, assigned in enumerate(candidate.reducer_ids, 1) if assigned == rid))
                    for rid in dict.fromkeys(candidate.reducer_ids))
                state["proposal_inputs"] = replace(inputs, reducers=reducers)
                diagnostics["selected_block"] = "hardware_tuple_seed"
            else:
                candidate = propose_candidate(state["candidate"], state["proposal_inputs"],
                    state["requirement"], state["rng"], state["proposal_attempts"], diagnostics=diagnostics)
            if candidate.motor_ids != state["motor_ids"]:
                raise ValueError("A hardware-tuple search attempted to change its fixed motor assignment")
            if candidate.reducer_ids != state["candidate"].reducer_ids:
                raise ValueError("A hardware-tuple search attempted to change its fixed reducer assignment")
            return candidate, diagnostics, state
        return None, {}, None

    def record(self, state, candidate, evaluation, *, counted):
        if counted:
            state["evaluations"] += 1
        elif not evaluation.summary.get("geometry_valid", True):
            state["geometry_rejections"] += 1
        else:
            state["reach_rejections"] += 1
        if state["evaluation"] is None or _rank(evaluation) > _rank(state["evaluation"]):
            state["candidate"], state["evaluation"] = candidate, evaluation

    def coverage(self):
        rows = []
        for state in self.states:
            evaluation = state["evaluation"]
            status = ("not_allocated" if not state["allocated_evaluations"] else
                      "not_attempted" if not state["attempted"] else
                      "no_supported_seed_in_probe_grid" if state["seed_error"] is not None else
                      "inner_budget_complete" if state["evaluations"] >= state["allocated_evaluations"] else
                      "inner_proposal_limit" if self._finished(state) else "in_progress")
            rows.append({**{name: state[name] for name in (
                "tuple_index", "layout_number", "motor_ids", "allocated_evaluations", "evaluations",
                "proposal_attempts", "duplicate_proposals", "geometry_rejections", "reach_rejections",
                "seed_probes", "seed_error", "attempted")}, "status": status,
                "best_pmax_kg": None if evaluation is None else evaluation.pmax_kg,
                "best_pmax_known_constraints_kg": None if evaluation is None else evaluation.summary.get("pmax_known_constraints_kg"),
                "best_peak_required_payload_utilization": None if evaluation is None else evaluation.summary.get("peak_required_payload_utilization"),
                "best_gravity_score_JG": None if evaluation is None else evaluation.jg,
                "best_gravity_capacity_score_JG": None if evaluation is None else evaluation.summary.get("gravity_capacity_score_JG"),
                "best_arm_mass_kg": None if evaluation is None else evaluation.total_mass_kg,
                "best_design": None if state["candidate"] is None else state["candidate"].to_dict(),
                "rng_state": to_jsonable(state["rng"].bit_generator.state)})
        return _finite({"mode": "exhaustive motor tuples with independent bounded geometry searches",
            "hardware_tuple_count": len(rows), "hardware_tuples_attempted": sum(row["attempted"] for row in rows),
            "hardware_tuples_load_evaluated": sum(row["evaluations"] > 0 for row in rows),
            "categorical_coverage_complete": all(row["attempted"] for row in rows),
            "load_evaluation_coverage_complete": all(row["evaluations"] > 0 for row in rows),
            "all_inner_budgets_met": all(row["allocated_evaluations"] > 0 and
                row["evaluations"] == row["allocated_evaluations"] for row in rows),
            "continuous_global_optimality_proven": False,
            "seed_failure_scope": "Only the recorded bounded seed probes; not a proof that no geometry can fit",
            "tuples": rows})


def _write_hardware_coverage(store, enumeration):
    if enumeration is None:
        return None
    coverage = enumeration.coverage()
    if not (store.path/"hardware_coverage.json").exists():
        store.write_json("hardware_coverage.json", coverage)
    rows = [{key: value for key, value in row.items() if key not in ("best_design", "rng_state")}
            for row in coverage["tuples"]]
    for row in rows:
        row["motor_ids"] = ";".join(row["motor_ids"])
    if not (store.path/"hardware_coverage.csv").exists():
        _write_rows(store, "hardware_coverage.csv", rows)
    return coverage


def prepare_project(inputs, requirement, data_dir, runs_dir, *, pose_limit=None):
    poses = generate_poses(requirement, pose_limit)
    store = create_run(runs_dir, requirement.project_id, data_dir, {
        "mode": "prepare", "requirement": to_jsonable(requirement),
        "pose_count": len(poses), "sampling": SAMPLING_DESCRIPTION,
        "pose_limit": pose_limit,
    })
    store.write_json("evaluated_inputs.json", to_jsonable(inputs))
    store.write_csv("reference_poses.csv", POSE_COLUMNS, pose_rows(poses))
    store.write_json("prepared.json", {
        "pose_count": len(poses), "project": to_jsonable(requirement),
        "catalogs_ready": bool(inputs.motors and inputs.reducers),
        "next_step": "Fill catalogs, then run the optimize command.",
    })
    return store.path


def optimize_project(inputs, requirement, data_dir, runs_dir, *, cycles=None,
                     checkpoint_every=None, pose_limit=None, seed=None):
    updates = {key: value for key, value in (
        ("cycles", cycles), ("checkpoint_every", checkpoint_every), ("seed", seed))
        if value is not None}
    requirement = replace(requirement, **updates)
    if requirement.cycles < 1 or requirement.checkpoint_every < 1 or requirement.seed < 0:
        raise ValueError("Cycles/checkpoint interval must be positive and seed nonnegative.")
    poses = generate_poses(requirement, pose_limit)
    enumeration = (_HardwareEnumeration(requirement)
                   if requirement.motor_candidates_by_joint is not None and
                   requirement.shared_wrist_motor_id is not None else None)
    store = create_run(runs_dir, requirement.project_id, data_dir, {
        "mode": "optimize", "requirement": to_jsonable(requirement),
        "pose_count": len(poses), "pose_limit": pose_limit,
        "sampling": SAMPLING_DESCRIPTION,
        "python": platform.python_version(), "mujoco": mujoco.__version__,
        "numpy": np.__version__,
        "algorithm": ("exhaustive motor-tuple enumeration with independent balanced bounded geometry searches"
                      if enumeration is not None else
                      "geometry-filtered block search with near-envelope and global coordinated hardware challenges; one distinct load evaluation per cycle"),
        "hardware_tuple_count": None if enumeration is None else len(enumeration.states),
        "continuous_global_optimality_proven": False,
        "evaluation_budget": "Distinct supported physical candidates only; duplicate proposals are skipped",
        "physical_key_precision_m": 1e-12,
        "proposal_attempt_limit": 10*requirement.cycles,
        "objective": ("maximize conditional transient-envelope sampled-basis payload" if
            getattr(requirement, "motor_load_policy", "continuous") == "hold_and_peak_envelope" else
            "maximize conservative sampled-basis payload") +
            " within straightened reach bounds; zero-payload ties minimize peak required-payload utilization, then maximize conditional known-constraint capacity; otherwise arm mass breaks ties; J_G is recorded",
        "reach_definition": "maximum fully extended shoulder-to-TCP radius",
        "structural_mass_model": STRUCTURAL_MASS_MODEL,
        "geometry_contract": GEOMETRY_CONTRACT,
        "motor_offset_coordinates": "Local deltas about approved base placements; coaxial coordinates are fixed",
        "checks": "approved local mounts, zero-pose own-joint assembly clearance and reach required; global collision and mechanical joint-stop checks disabled",
    })
    store.write_json("evaluated_inputs.json", to_jsonable(inputs))
    store.write_csv("reference_poses.csv", POSE_COLUMNS, pose_rows(poses))
    print(f"Run folder: {store.path}", flush=True)
    print(f"{len(poses)} poses; up to {requirement.cycles} distinct load evaluations; "
          f"checkpoint every {requirement.checkpoint_every}", flush=True)
    if enumeration is not None:
        print(f"Enumerating {len(enumeration.states)} motor tuples with independent geometry budgets; "
              "short smoke budgets may leave tuples untested.", flush=True)
    rng = np.random.default_rng(requirement.seed)
    best = initial_candidate(inputs, requirement) if enumeration is None else None
    best_evaluation = None
    best_cycle = 0
    started = time.perf_counter()
    cycle = 0
    proposal_number = 0
    proposal_limit = 10*requirement.cycles
    checked_keys = set()
    duplicate_proposals = 0
    proposal_totals = Counter()
    proposal_blocks = Counter()
    fallback_reasons = Counter()
    geometry_rejections = Counter()
    reach_rejections = 0
    last_checkpoint_cycle = None

    def proposal_summary():
        return {**dict(proposal_totals), "selected_blocks": dict(proposal_blocks),
                "fallback_reasons": dict(fallback_reasons)}

    def checkpoint(*, exhausted=False):
        nonlocal last_checkpoint_cycle
        coverage = None if enumeration is None else enumeration.coverage()
        if best_evaluation is None:
            return
        record = {
            "cycle": cycle, "best_found_cycle": best_cycle,
            "checkpoint_kind": (("hardware_search_finished" if enumeration is not None else "proposal_limit")
                                if exhausted else "evaluation_milestone"),
            "requested_evaluations": requirement.cycles,
            "proposal_attempts": proposal_number,
            "unique_candidates_checked": len(checked_keys),
            "duplicate_proposals_skipped": duplicate_proposals,
            "elapsed_s": round(time.perf_counter()-started, 3),
            "layout_number": best.layout_number,
            "reachable_poses": best_evaluation.reachable_count,
            "total_poses": len(poses), "pmax_kg": best_evaluation.pmax_kg,
            "required_payload_upper_kg": requirement.payload_upper_kg,
            "requirements_met": best_evaluation.feasible,
            "geometry_valid": bool(best_evaluation.summary.get("geometry_valid", True)),
            "extended_reach_mm": extended_reach_m(best)*1000,
            "wrist2_length_mm": best.wrist2_length_m*1000,
            "geometry_rejections": sum(geometry_rejections.values()),
            "cheap_geometry_rejections": proposal_totals["geometry_rejected_trials"],
            "proposal_fallbacks": proposal_totals["fallback_to_incumbent"],
            "capacity_challenge_probes": proposal_totals["capacity_challenge_probes"],
            "arm_mass_kg": best_evaluation.total_mass_kg,
            "gravity_score_JG": _finite(best_evaluation.jg),
        }
        payload = {"hallmark": record, "best_design": best.to_dict(),
                   "summary": _finite(best_evaluation.summary),
                   "rng_state": to_jsonable(rng.bit_generator.state),
                   "geometry_rejection_counts": dict(geometry_rejections),
                   "proposal_diagnostics": proposal_summary()}
        if coverage is not None:
            payload["hardware_coverage"] = coverage
        same_cycle = last_checkpoint_cycle == cycle
        if exhausted and same_cycle:
            store.write_json(f"checkpoints/final_after_proposal_{proposal_number:06d}.json", payload)
        else:
            store.checkpoint(cycle, payload)
        last_checkpoint_cycle = cycle
        if not same_cycle:
            store.append_hallmark(record)
        print(f"Cycle {cycle}/{requirement.cycles}: "
              f"poses {best_evaluation.reachable_count}/{len(poses)}, "
              f"extension {extended_reach_m(best)*1000:.1f} mm, "
              f"Pmax {best_evaluation.pmax_kg:.4g} kg, "
              f"mass {best_evaluation.total_mass_kg:.3f} kg, "
              f"J_G {_number(best_evaluation.jg)}; "
              f"proposals {proposal_number}, duplicates skipped {duplicate_proposals}, "
              f"cheap geometry rejected {proposal_totals['geometry_rejected_trials']}", flush=True)

    try:
        while cycle < requirement.cycles and proposal_number < proposal_limit:
            hardware_state = None
            if enumeration is not None:
                candidate, diagnostics, hardware_state = enumeration.next(inputs)
                proposal_number = enumeration.proposal_attempts
                if candidate is None:
                    break
            else:
                proposal_number += 1
                diagnostics = {}
                if proposal_number <= len(requirement.allowed_layouts):
                    candidate = (best if proposal_number == 1 else initial_candidate(
                        inputs, replace(requirement, allowed_layouts=(requirement.allowed_layouts[proposal_number-1],))))
                else:
                    candidate = propose_candidate(best, inputs, requirement, rng, proposal_number,
                                                  diagnostics=diagnostics)
            for name in ("block_trials", "geometry_rejected_trials", "duplicate_trials",
                         "capacity_challenge_probes", "capacity_challenge_probe_limit_reached",
                         "fallback_to_incumbent"):
                proposal_totals[name] += int(diagnostics.get(name, 0))
            if diagnostics.get("selected_block"):
                proposal_blocks[diagnostics["selected_block"]] += 1
            if diagnostics.get("fallback_reason"):
                fallback_reasons[diagnostics["fallback_reason"]] += 1
            key = _candidate_key(candidate)
            if key in checked_keys:
                duplicate_proposals += 1
                if hardware_state is not None:
                    hardware_state["duplicate_proposals"] += 1
                if proposal_number % 100 == 0:
                    print(f"Proposal {proposal_number}/{proposal_limit}: {cycle} distinct load "
                          "evaluations; repeated physical candidates skipped.", flush=True)
                continue
            checked_keys.add(key)
            evaluation = evaluate_candidate(candidate, inputs, poses, requirement)
            geometry_valid = evaluation.summary.get("geometry_valid", True)
            reach_valid = evaluation.summary.get("reach_bounds_pass", True)
            if not geometry_valid:
                geometry_rejections[evaluation.summary.get("geometry_error", "Unsupported base geometry")] += 1
            elif not reach_valid:
                reach_rejections += 1
            else:
                cycle += 1
            if hardware_state is not None:
                enumeration.record(hardware_state, candidate, evaluation,
                                   counted=bool(geometry_valid and reach_valid))
            if best_evaluation is None or _rank(evaluation) > _rank(best_evaluation):
                best, best_evaluation, best_cycle = candidate, evaluation, cycle
            if geometry_valid and reach_valid and (cycle == 1 or cycle % requirement.checkpoint_every == 0
                                                     or cycle == requirement.cycles):
                checkpoint()
        budget_reached = cycle == requirement.cycles
        termination_reason = ("evaluation_budget_reached" if budget_reached
                              else "hardware_inner_budgets_exhausted" if enumeration is not None
                              else "proposal_attempt_limit_reached")
        if last_checkpoint_cycle != cycle or not budget_reached:
            checkpoint(exhausted=not budget_reached)
        if not budget_reached:
            reason = "Hardware tuple searches finished" if enumeration is not None else "Proposal limit reached"
            print(f"{reason}: retained best after {cycle}/{requirement.cycles} "
                  f"distinct load evaluations and {proposal_number} proposals.", flush=True)
        # Save detailed coefficients once, instead of one enormous file per evaluation.
        if best_evaluation is None:
            raise ValueError("No hardware tuple produced an initial candidate. See hardware_coverage.json for seed errors.")
        if not best_evaluation.summary.get("geometry_valid", True):
            raise ValueError("No candidate fits the approved base geometry. Review selected hardware envelopes; "
                             "no best_model.xml was exported. See checkpoint geometry errors.")
        if not best_evaluation.summary.get("reach_bounds_pass", True):
            raise ValueError("No candidate meets the fully extended reach bounds; no best_model.xml was exported.")
        final = evaluate_candidate(best, inputs, poses, requirement, collect_basis=True)
        complete_xml = export_model(best, inputs, requirement)
        store.write_json("best_design.json", best.to_dict())
        store.write_text("best_model.xml", complete_xml)
        _write_design_tables(store, best, inputs, complete_xml)
        _write_rows(store, "pose_results.csv", final.per_pose_rows)
        _write_rows(store, "basis_coefficients.csv", getattr(final, "basis_rows", []))
        store.write_json("result.json", _finite({
            "status": "completed", "evaluations": cycle, "best_found_cycle": best_cycle,
            "requested_evaluations": requirement.cycles,
            "budget_reached": budget_reached, "termination_reason": termination_reason,
            "proposal_attempts": proposal_number, "proposal_attempt_limit": proposal_limit,
            "unique_candidates_checked": len(checked_keys),
            "duplicate_proposals_skipped": duplicate_proposals,
            "reach_rejections": reach_rejections,
            "proposal_diagnostics": proposal_summary(),
            "hardware_enumeration": None if enumeration is None else enumeration.coverage(),
            "final_detail_evaluation_included_in_budget": False,
            "best_model_file": "best_model.xml",
            "elapsed_s": time.perf_counter()-started, "pose_count": len(poses),
            "pmax_kg": final.pmax_kg, "gravity_score_JG": final.jg,
            "pmax_continuous_screen_kg": final.summary.get("pmax_continuous_screen_kg"),
            "motor_load_policy": final.summary.get("motor_load_policy", "continuous"),
            "pmax_known_constraints_kg": final.summary.get("pmax_known_constraints_kg"),
            "gravity_capacity_score_JG": final.summary.get("gravity_capacity_score_JG"),
            "reducer_load_policy": final.summary.get("reducer_load_policy", "continuous"),
            "hardware_validation_status": final.summary.get("hardware_validation_status", "legacy_continuous_screen"),
            "motor_transient_rating_verified": final.summary.get("motor_transient_rating_verified", False),
            "duty_cycle_verified": final.summary.get("duty_cycle_verified", False),
            "extended_reach_mm": extended_reach_m(best)*1000,
            "wrist2_length_mm": best.wrist2_length_m*1000,
            "layout_number": best.layout_number,
            "arm_mass_kg": final.total_mass_kg, "requirements_met": final.feasible,
            "reachable_poses": final.reachable_count, "summary": final.summary,
            "geometry_rejection_counts": dict(geometry_rejections),
            "scope": "Finite basis/grid search; this is not a proof of a global design optimum.",
        }))
        store.write_csv("result.csv", (
            "project_id", "layout_number", "extended_reach_mm", "wrist2_length_mm", "candidate_evaluations", "pose_count", "reachable_poses",
            "pmax_kg", "required_payload_upper_kg", "requirements_met", "arm_mass_kg",
            "gravity_score_JG", "pmax_known_constraints_kg", "gravity_capacity_score_JG",
            "reducer_load_policy", "motor_load_policy", "pmax_continuous_screen_kg", "hardware_validation_status", "requested_evaluations", "budget_reached", "proposal_attempts",
            "unique_candidates_checked", "duplicate_proposals_skipped", "cheap_geometry_rejections",
            "proposal_fallbacks", "capacity_challenge_probes", "termination_reason"), [{
                "project_id": requirement.project_id, "candidate_evaluations": cycle,
                "layout_number": best.layout_number,
                "extended_reach_mm": extended_reach_m(best)*1000,
                "wrist2_length_mm": best.wrist2_length_m*1000,
                "pose_count": len(poses), "reachable_poses": final.reachable_count,
                "pmax_kg": final.pmax_kg, "required_payload_upper_kg": requirement.payload_upper_kg,
                "requirements_met": final.feasible, "arm_mass_kg": final.total_mass_kg,
                "gravity_score_JG": final.jg,
                "pmax_known_constraints_kg": final.summary.get("pmax_known_constraints_kg"),
                "gravity_capacity_score_JG": final.summary.get("gravity_capacity_score_JG"),
                "reducer_load_policy": final.summary.get("reducer_load_policy", "continuous"),
                "motor_load_policy": final.summary.get("motor_load_policy", "continuous"),
                "pmax_continuous_screen_kg": final.summary.get("pmax_continuous_screen_kg"),
                "hardware_validation_status": final.summary.get("hardware_validation_status", "legacy_continuous_screen"),
                "requested_evaluations": requirement.cycles,
                "budget_reached": budget_reached, "proposal_attempts": proposal_number,
                "unique_candidates_checked": len(checked_keys),
                "duplicate_proposals_skipped": duplicate_proposals,
                "cheap_geometry_rejections": proposal_totals["geometry_rejected_trials"],
                "proposal_fallbacks": proposal_totals["fallback_to_incumbent"],
                "capacity_challenge_probes": proposal_totals["capacity_challenge_probes"],
                "termination_reason": termination_reason,
            }])
        store.write_text("RESULT.md", _report(requirement, final, cycle, len(poses), pose_limit,
                                              proposal_attempts=proposal_number,
                                              termination_reason=termination_reason))
        if enumeration is not None:
            coverage = _write_hardware_coverage(store, enumeration)
            with (store.path/"RESULT.md").open("a", encoding="utf-8") as stream:
                stream.write("\n## Motor tuple coverage\n\n"
                    f"Attempted {coverage['hardware_tuples_attempted']}/{coverage['hardware_tuple_count']} motor tuples; "
                    f"load-evaluated {coverage['hardware_tuples_load_evaluated']}/{coverage['hardware_tuple_count']}. "
                    f"Categorical coverage complete: {coverage['categorical_coverage_complete']}. "
                    "Each tuple has its own bounded geometry search. Continuous global optimality is not proven.\n\n"
                    "See [hardware coverage](hardware_coverage.csv) and [detailed coverage](hardware_coverage.json).\n")
    except BaseException as exc:
        _write_hardware_coverage(store, enumeration)
        store.write_json("interrupted.json" if isinstance(exc, KeyboardInterrupt) else "failed.json", {
            "cycle": cycle, "exception": type(exc).__name__, "message": str(exc),
            "requested_evaluations": requirement.cycles, "proposal_attempts": proposal_number,
            "unique_candidates_checked": len(checked_keys),
            "duplicate_proposals_skipped": duplicate_proposals,
            "proposal_diagnostics": proposal_summary(),
            "last_checkpoints": "Previously written checkpoints and input snapshots are retained.",
        })
        raise
    return store.path


def _write_design_tables(store, candidate, inputs, xml):
    """Export dimensions and placements in named rows instead of anonymous arrays."""
    lengths = dict(zip(("base_link", *STRUCTURAL_BODIES),
                       (.10, .12, *candidate.lengths_m, candidate.wrist2_length_m, .08)))
    model = mujoco.MjModel.from_xml_string(xml)
    links = []
    for properties in structural_properties(model):
        name = properties["body_name"]
        length = lengths[name]
        center = np.array(properties["com_local_xyz_m"])*1000
        inertia = np.array(properties["inertia_com_local_kg_m2"])
        row = {
            "body_name": name, "length_mm": length*1000,
            "structural_mass_kg": properties["mass_kg"],
            "com_x_mm": center[0], "com_y_mm": center[1], "com_z_mm": center[2],
            "com_along_link_mm": center[2], "com_fraction": center[2]/(length*1000),
            "spread_fraction": properties["longitudinal_variance_m2"]/length**2,
            "mass_source": "approved base stock geometry; catalog cases excluded",
        }
        for a, b, label in ((0,0,"xx"), (1,1,"yy"), (2,2,"zz"), (0,1,"xy"), (0,2,"xz"), (1,2,"yz")):
            row[f"inertia_{label}_kg_m2"] = inertia[a,b]
        links.append(row)
    _write_rows(store, "best_links.csv", links)
    tree = ET.fromstring(xml)
    bodies = {body.get("name"): body for body in tree.iter("body")}
    parents = {child.get("name"): parent.get("name") for parent in tree.iter("body")
               for child in parent.findall("body")}
    layout = layout_by_number(candidate.layout_number)
    joints = []
    for i, (mid, rid) in enumerate(zip(candidate.motor_ids, candidate.reducer_ids), 1):
        motor, reducer = inputs.motor_by_id[mid], inputs.reducer_by_id[rid]
        row = {
            "joint": f"J{i}", "drive": layout.drives[i-1],
            "motor_id": mid, "motor_name": motor.name,
            "reducer_id": rid, "reducer_name": reducer.name,
            "reduction_ratio": reducer.reduction_ratio,
            "motor_mass_kg": motor.mass_kg, "reducer_mass_kg": reducer.mass_kg,
            "rotor_inertia_kg_m2": motor.rotor_inertia_kg_m2,
            "evaluation_rotor_inertia_kg_m2": motor.evaluation_rotor_inertia_kg_m2,
            "rotor_inertia_basis": motor.rotor_inertia_basis,
            "motor_voltage_V": motor.rated_voltage_V,
            "motor_rated_current_A": motor.rated_current_A,
            "motor_price_usd": motor.price_usd,
            "motor_source_url": motor.source_url,
            "reducer_allowed_joints": ";".join(map(str, reducer.allowed_joints)),
            "fixed_parent_body": parents[f"motor_{i}"],
            "position_frame": "fixed_parent_body local XYZ",
        }
        for component in ("motor", "reducer"):
            coordinates = np.fromstring(bodies[f"{component}_{i}"].get("pos"), sep=" ")*1000
            for axis, value in zip("xyz", coordinates):
                row[f"{component}_center_{axis}_mm"] = value
        joints.append(row)
    _write_rows(store, "best_joints.csv", joints)


def _report(requirement, result, cycles, pose_count, pose_limit, *, proposal_attempts=None,
            termination_reason=None):
    status = "MET" if result.feasible else "NOT MET"
    peak_policy = getattr(requirement, "motor_load_policy", "continuous") == "hold_and_peak_envelope"
    capacity_label = "Conditional transient-envelope" if peak_policy else "Conservative continuous-screen"
    capacity_note = (
        "Primary Pmax checks continuous holding and speed-dependent motor transient curves, plus reducer "
        "holding/repeated-peak limits. Motor pulse durations and timed thermal duty remain unverified. "
        "Any voltage-extrapolated motor curve is an explicit assumption, not a manufacturer-qualified operating point. "
        if peak_policy else
        "Primary Pmax retains conservative instantaneous motor continuous limits. ")
    return (
        f"# {requirement.project_id}\n\n"
        f"{capacity_label} sampled requirements: **{status}**.\n\n"
        f"- Reference poses: {pose_count}" + (" (reduced smoke subset)" if pose_limit else "") + "\n"
        f"- Distinct load evaluations: {cycles}/{requirement.cycles} requested\n"
        f"- Proposal attempts: {proposal_attempts if proposal_attempts is not None else cycles}\n"
        f"- Search termination: {termination_reason or 'evaluation_budget_reached'}\n"
        f"- Required payload: {requirement.payload_lower_g:g}–{requirement.payload_upper_g:g} g\n"
        f"- Required fully extended shoulder-to-TCP reach: {requirement.reach_lower_mm:g}–{requirement.reach_upper_mm:g} mm\n"
        f"- Fixed wrist2 span: {getattr(requirement, 'wrist2_length_m', .07)*1000:g} mm\n"
        f"- Fully extended reach: {result.summary.get('extended_reach_mm', 0.):.6g} mm\n"
        f"- Reachable targets: {result.reachable_count}/{pose_count}\n"
        f"- {capacity_label} Pmax: {result.pmax_kg:.6g} kg\n"
        f"- Continuous-only screen: {_number(result.summary.get('pmax_continuous_screen_kg'), 6)} kg\n"
        f"- Motor rating policy: {getattr(requirement, 'motor_load_policy', 'continuous')}\n"
        f"- Conditional known-constraint capacity: {_number(result.summary.get('pmax_known_constraints_kg'), 6)} kg\n"
        f"- Reducer rating policy: {result.summary.get('reducer_load_policy', 'continuous')}\n"
        f"- Hardware validation status: {result.summary.get('hardware_validation_status', 'legacy_continuous_screen')}\n"
        f"- Arm mass: {result.total_mass_kg:.6g} kg\n"
        f"- Gravity loading score J_G: {_number(result.jg, 6)}\n"
        f"- Capacity-normalized gravity score: {_number(result.summary.get('gravity_capacity_score_JG'), 6)}\n\n"
        + capacity_note +
        "The conditional known-constraint capacity checks continuous holding ratings and reducer "
        "repeated peaks but omits unverified motor transient limits. It is not a certified payload: "
        "motor pulse capability and cycle-average thermal/lifetime limits remain unverified.\n\n"
        "Read `pose_results.csv` for per-pose loads, `best_links.csv` for named structural "
        "mass properties, and `best_joints.csv` for selected hardware and local placements. "
        "`basis_coefficients.csv` stores the final "
        "empty/unit-payload load basis. Input CSVs are copied under `inputs/`.\n\n"
        "Scoring and export use the same parameterized approved base geometry. Unsupported "
        "local case mounts, hardware envelopes, link spans and own-joint assembly interference "
        "are rejected before load simulation. "
        "The exported model includes edge frames, mounts, flanges, belts, sites, sensors, "
        "catalog-limited servos and reflected rotor inertia. Structural mass, 3D COM and inertia "
        "are calculated from the approved base stock geometry; they are not free optimization variables. "
        "The study uses unlimited joints and no global self-collision checks. Precision, elastic stiffness, "
        "peak-duration thermal behavior and task-specific forward validation are outside this run. "
        "The finite velocity probes do not certify every possible motion.\n"
    )

"""Readable, append-only comparison of three independent configuration searches."""

import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
from uuid import uuid4

from .inputs import InputError
from .reach import extended_reach_m
from .storage import RunStore
from .settings import RANK_SCORE_DECIMALS


COMPARISON_COLUMNS = (
    "layout_number", "extended_reach_mm", "L1_mm", "L2_mm", "L3_mm",
    "L1_mass_g", "L2_mass_g", "L3_mass_g", "arm_mass_kg", "pmax_kg",
    "pmax_continuous_screen_kg", "motor_load_policy", "pmax_known_constraints_kg", "gravity_score_JG", "gravity_capacity_score_JG",
    "evaluations", "allocated_evaluations", "requirements_met", "hardware_validation_status", "run_dir", "model_file",
)
_LINK_NAMES = ("upper_arm_link", "forearm_link", "wrist_1_link")
_GOAL_FIELDS = ("payload_lower_g", "payload_upper_g", "reach_lower_mm", "reach_upper_mm")
_STUDY_FIELDS = ("seed", "joint_speed_rad_s", "joint_accel_rad_s2", "cycles")


def _json(path):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise InputError(f"Cannot read comparison source {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise InputError(f"Comparison source must contain a JSON object: {path}")
    return value


def _number(value, name):
    if isinstance(value, bool):
        raise InputError(f"Comparison field {name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise InputError(f"Comparison field {name} must be a finite number") from exc
    if not math.isfinite(result):
        raise InputError(f"Comparison field {name} must be a finite number")
    return result


def _enumeration_plan(result, layout, cycles, run):
    """Compare allocated work, allowing explicitly exhausted unsupported tuples."""
    coverage = result.get("hardware_enumeration")
    if coverage is None:
        return None
    tuples = coverage.get("tuples") if isinstance(coverage, dict) else None
    if (not tuples or coverage.get("categorical_coverage_complete") is not True
            or coverage.get("hardware_tuple_count") != len(tuples)):
        raise InputError(f"Run has incomplete categorical hardware coverage: {run}")
    plan, total, evaluated, seen = [], 0, 0, set()
    for item in tuples:
        motors = item.get("motor_ids")
        if (item.get("layout_number") != layout or item.get("attempted") is not True
                or not isinstance(motors, list) or len(motors) != 6 or tuple(motors) in seen):
            raise InputError(f"Invalid or duplicate hardware tuple in comparison: {run}")
        seen.add(tuple(motors))
        allocated = _number(item.get("allocated_evaluations"), "allocated_evaluations")
        count = _number(item.get("evaluations"), "tuple evaluations")
        if allocated <= 0 or not 0 <= count <= allocated:
            raise InputError(f"Run has unallocated or invalid hardware work: {run}")
        if count != allocated and item.get("status") not in (
                "no_supported_seed_in_probe_grid", "inner_proposal_limit"):
            raise InputError(f"Run has unfinished inner geometry search: {run}")
        total += allocated
        evaluated += count
        plan.append({"motor_ids": motors, "allocated_evaluations": allocated})
    if total != cycles or evaluated != result.get("evaluations"):
        raise InputError(f"Hardware coverage disagrees with recorded evaluation totals: {run}")
    return plan


def _read_run(path):
    run = Path(path).resolve()
    result, recorded, design = (_json(run/name) for name in
                                ("result.json", "run.json", "best_design.json"))
    if result.get("status") != "completed":
        raise InputError(f"Only completed independent runs can be compared: {run}")
    meta = recorded.get("metadata", {})
    requirement = meta.get("requirement", {})
    summary = result.get("summary", {})
    if not all(isinstance(value, dict) for value in (meta, requirement, summary)):
        raise InputError(f"Missing study metadata for comparison: {run}")
    layout = design.get("layout_number")
    if layout not in (7, 8, 9) or isinstance(layout, bool):
        raise InputError(f"Comparison requires configurations 07, 08 and 09: {run}")
    if tuple(requirement.get("allowed_layouts", ())) != (layout,):
        raise InputError(f"Run must search only configuration {layout:02d}, independently: {run}")
    if result.get("layout_number", layout) != layout:
        raise InputError(f"Result and winner configuration disagree: {run}")
    if summary.get("geometry_valid") is not True:
        raise InputError(f"Winner has no valid approved base geometry: {run}")
    contract = summary.get("geometry_contract")
    if not contract or meta.get("geometry_contract") != contract:
        raise InputError(f"Scored geometry contract is missing or inconsistent: {run}")
    conditions = {}
    for name in (*_GOAL_FIELDS, *_STUDY_FIELDS):
        conditions[name] = _number(requirement.get(name), name)
    speeds = requirement.get("joint_speed_limits_rad_s")
    if speeds is None:
        speeds = [conditions["joint_speed_rad_s"]] * 6
    if not isinstance(speeds, (tuple, list)) or len(speeds) != 6:
        raise InputError(f"Six joint speed limits are required for comparison: {run}")
    conditions["joint_speed_limits_rad_s"] = [_number(value, "joint_speed_limits_rad_s") for value in speeds]
    conditions["wrist2_length_m"] = _number(requirement.get("wrist2_length_m", .07), "wrist2_length_m")
    conditions["motor_candidates_by_joint"] = requirement.get("motor_candidates_by_joint")
    conditions["shared_wrist_motor_id"] = requirement.get("shared_wrist_motor_id")
    conditions["reducer_load_policy"] = requirement.get("reducer_load_policy", "continuous")
    conditions["motor_load_policy"] = requirement.get("motor_load_policy", "continuous")
    evaluations = _number(result.get("evaluations"), "evaluations")
    plan = _enumeration_plan(result, layout, conditions["cycles"], run)
    if plan is None and evaluations != conditions["cycles"]:
        raise InputError(f"Run did not finish its recorded evaluation budget: {run}")
    conditions["evaluations"] = conditions["cycles"]
    conditions["hardware_enumeration_plan"] = plan
    count = _number(result.get("pose_count"), "pose_count")
    if count < 1 or meta.get("pose_count") != count:
        raise InputError(f"Reference-pose count is missing or inconsistent: {run}")
    conditions["pose_count"] = count
    conditions["pose_limit"] = meta.get("pose_limit")
    for name in ("sampling", "reach_definition", "structural_mass_model"):
        if not isinstance(meta.get(name), str) or not meta[name]:
            raise InputError(f"Comparison field {name} is missing: {run}")
        conditions[name] = meta[name]
    conditions["geometry_contract"] = contract
    for name in ("python", "mujoco", "numpy", "algorithm", "evaluation_budget"):
        conditions[name] = meta.get(name)
    for name in ("score_definition", "capacity_model", "velocity_cases_per_pose",
                 "acceleration_corners_per_velocity", "payload_inertia_model", "jg_definition"):
        conditions[name] = summary.get(name)
    try:
        reference_bytes = (run/"reference_poses.csv").read_bytes()
    except OSError as exc:
        raise InputError(f"Cannot read the actual comparison pose basis in {run}: {exc}") from exc
    conditions["reference_poses_sha256"] = hashlib.sha256(reference_bytes).hexdigest()
    hashes = recorded.get("input_sha256", {})
    conditions["catalog_sha256"] = {name: value for name, value in hashes.items()
                                     if name != "requirement.csv"}

    lengths = design.get("lengths_m", ())
    if len(lengths) != 3 or any(_number(value, "lengths_m") <= 0 for value in lengths):
        raise InputError(f"Winner needs three positive link lengths: {run}")
    wrist2_length = _number(design.get("wrist2_length_m", .07), "wrist2_length_m")
    if not math.isclose(wrist2_length, conditions["wrist2_length_m"], abs_tol=1e-12):
        raise InputError(f"Winner wrist spacing disagrees with study conditions: {run}")
    reach = extended_reach_m(lengths, layout, wrist2_length_m=wrist2_length)*1000
    if not math.isclose(reach, _number(result.get("extended_reach_mm"), "extended_reach_mm"),
                        rel_tol=0., abs_tol=1e-6):
        raise InputError(f"Reported extension disagrees with the winner's geometry: {run}")
    if not conditions["reach_lower_mm"]-1e-6 <= reach <= conditions["reach_upper_mm"]+1e-6:
        raise InputError(f"Winner is outside the fully extended reach bounds: {run}")
    try:
        with (run/"best_links.csv").open(encoding="utf-8", newline="") as stream:
            links = list(csv.DictReader(stream))
        by_name = {row["body_name"]: row for row in links}
    except (OSError, KeyError) as exc:
        raise InputError(f"Cannot read structural comparison properties in {run}: {exc}") from exc
    if len(by_name) != len(links) or not all(name in by_name for name in _LINK_NAMES):
        raise InputError(f"Missing or repeated L1/L2/L3 structural rows in {run}")
    for length, name in zip(lengths, _LINK_NAMES):
        if not math.isclose(_number(by_name[name].get("length_mm"), "length_mm"),
                            length*1000, rel_tol=0., abs_tol=1e-6):
            raise InputError(f"Structural length table disagrees with winner {name}: {run}")
    model_name = Path(result.get("best_model_file", "best_model.xml"))
    model = (run/model_name).resolve()
    if model_name.is_absolute() or not model.is_relative_to(run) or not model.is_file():
        raise InputError(f"Completed winner model is missing or outside its run: {run}")
    met = result.get("requirements_met")
    if not isinstance(met, bool):
        raise InputError(f"Missing requirements_met result: {run}")
    row = {"layout_number": layout, "extended_reach_mm": reach,
           **{f"L{i}_mm": value*1000 for i, value in enumerate(lengths, 1)},
           **{f"L{i}_mass_g": _number(by_name[name].get("structural_mass_kg"), "structural_mass_kg")*1000
              for i, name in enumerate(_LINK_NAMES, 1)},
           "arm_mass_kg": _number(result.get("arm_mass_kg"), "arm_mass_kg"),
           "pmax_kg": _number(result.get("pmax_kg"), "pmax_kg"),
           "pmax_continuous_screen_kg": result.get("pmax_continuous_screen_kg", summary.get("pmax_continuous_screen_kg")),
           "motor_load_policy": conditions["motor_load_policy"],
           "pmax_known_constraints_kg": (None if result.get("pmax_known_constraints_kg",
                summary.get("pmax_known_constraints_kg")) is None else _number(result.get(
                "pmax_known_constraints_kg", summary.get("pmax_known_constraints_kg")), "pmax_known_constraints_kg")),
           "gravity_score_JG": (None if result.get("gravity_score_JG") is None else
                                 _number(result["gravity_score_JG"], "gravity_score_JG")),
           "gravity_capacity_score_JG": result.get("gravity_capacity_score_JG", summary.get("gravity_capacity_score_JG")),
           "evaluations": evaluations, "allocated_evaluations": conditions["cycles"],
           "hardware_validation_status": result.get("hardware_validation_status",
                summary.get("hardware_validation_status", "legacy_continuous_screen")),
           "requirements_met": met, "run_dir": str(run), "model_file": str(model)}
    return conditions, row


def _link(label, path, folder):
    try:
        relative = os.path.relpath(path, folder).replace("\\", "/")
    except ValueError:
        relative = Path(path).as_posix()
    return f"[{label}](<{relative}>)"


def _detail_rows(rows, conditions):
    """Aggregate saved supplier selections and loads; never rerun simulation."""
    required = ("evaluated_inputs.json", "pose_results.csv")
    if not all((Path(row["run_dir"])/name).is_file() for row in rows for name in required):
        return None
    hardware, loads = [], []
    motor_fields = {
        "motor_name": "name", "rated_voltage_V": "rated_voltage_V",
        "rated_torque_Nm": "rated_torque_Nm", "rated_speed_rpm": "rated_speed_rpm",
        "rated_power_W": "rated_power_W", "rated_current_A": "rated_current_A",
        "motor_mass_kg": "mass_kg", "motor_radius_mm": "radius_mm",
        "motor_body_length_mm": "length_mm", "motor_price_usd": "price_usd",
        "motor_source_url": "source_url", "published_rotor_inertia_kg_m2": "rotor_inertia_kg_m2",
        "rotor_inertia_upper_bound_kg_m2": "rotor_inertia_upper_bound_kg_m2",
        "rotor_inertia_basis": "rotor_inertia_basis",
        "operating_voltage_V": "operating_voltage_V",
        "motor_peak_torque_Nm": "peak_torque_Nm",
        "motor_peak_duration_s": "peak_duration_s",
        "motor_peak_curve_basis": "peak_curve_basis",
    }
    reducer_fields = {
        "reducer_name": "name", "reduction_ratio": "reduction_ratio",
        "reducer_mass_kg": "mass_kg", "reducer_rated_output_torque_Nm": "continuous_output_torque_Nm",
        "reducer_rated_input_speed_rpm": "rated_input_speed_rpm",
    }
    for row in rows:
        run = Path(row["run_dir"])
        try:
            inputs, design = _json(run/"evaluated_inputs.json"), _json(run/"best_design.json")
            motors = {value["motor_id"]: value for value in inputs["motors"]}
            reducers = {value["reducer_id"]: value for value in inputs["reducers"]}
            with (run/"pose_results.csv").open(encoding="utf-8-sig", newline="") as stream:
                poses = [pose for pose in csv.DictReader(stream)
                         if pose.get("reachable", "").lower() in ("true", "1")]
            if len(design["motor_ids"]) != 6 or len(design["reducer_ids"]) != 6:
                raise ValueError("six selected motors and reducers are required")
            for joint, (mid, rid) in enumerate(zip(design["motor_ids"], design["reducer_ids"]), 1):
                motor, reducer = motors[mid], reducers[rid]
                common = {"layout_number": row["layout_number"], "joint": joint,
                          "required_payload_g": conditions["payload_upper_g"]}
                hardware.append({**common, "motor_id": mid, "reducer_id": rid,
                                 **{key: motor.get(source) for key, source in motor_fields.items()},
                                 **{key: reducer.get(source) for key, source in reducer_fields.items()}})
                peak = lambda column, absolute=False: max(
                    (abs(float(pose[column])) if absolute else float(pose[column]))
                    for pose in poses) if poses else None
                output_limit = float(reducer["continuous_output_torque_Nm"])
                repeated_limit = float(reducer.get("peak_output_torque_Nm", output_limit))
                if reducer.get("max_input_torque_Nm") is not None:
                    input_limit = float(reducer["max_input_torque_Nm"])*float(reducer["reduction_ratio"])
                    output_limit, repeated_limit = min(output_limit, input_limit), min(repeated_limit, input_limit)
                dynamic = peak(f"peak_joint_load_j{joint}_Nm_at_required_payload")
                gravity = peak(f"gravity_j{joint}_Nm_at_required_payload", True)
                motion_limit = repeated_limit if conditions["reducer_load_policy"] == "hold_and_repeated_peak" else output_limit
                loads.append({**common, "reachable_pose_count": len(poses),
                    "max_gravity_torque_Nm": gravity,
                    "max_dynamic_joint_torque_Nm": dynamic,
                    "reducer_output_limit_Nm": motion_limit,
                    "reducer_holding_limit_Nm": output_limit,
                    "reducer_repeated_peak_limit_Nm": repeated_limit,
                    "max_reducer_utilization_ratio": (max(dynamic/motion_limit, gravity/output_limit)
                        if dynamic is not None else None),
                    "max_motor_torque_Nm": peak(f"peak_motor_torque_j{joint}_Nm_at_required_payload"),
                    "motor_continuous_torque_Nm": motor.get("rated_torque_Nm"),
                    "motor_peak_torque_Nm": motor.get("peak_torque_Nm"),
                    "motor_load_policy": conditions["motor_load_policy"],
                    "max_holding_motor_torque_Nm": gravity/float(reducer["reduction_ratio"]),
                    "max_motor_utilization_ratio": peak(f"peak_motor_utilization_j{joint}_ratio")})
        except (OSError, KeyError, TypeError, ValueError) as exc:
            raise InputError(f"Cannot aggregate saved hardware and joint loads in {run}: {exc}") from exc
    return hardware, loads


def _j2_voltage_rows(rows):
    """Retain the best sampled geometry for every J2 electrical alternative."""
    result_rows = []
    for row in rows:
        run = Path(row["run_dir"])
        result = _json(run/"result.json")
        coverage = result.get("hardware_enumeration")
        if not coverage or not (run/"evaluated_inputs.json").exists():
            return None
        motors = {item["motor_id"]: item for item in _json(run/"evaluated_inputs.json")["motors"]}
        groups = {}
        for item in coverage["tuples"]:
            groups.setdefault(item["motor_ids"][1], []).append(item)
        for motor_id, tuples in groups.items():
            supported = [item for item in tuples if item.get("evaluations", 0) > 0
                         and item.get("best_design") is not None and item.get("best_pmax_kg") is not None]
            motor = motors[motor_id]
            def rank(item):
                payload = round(item["best_pmax_kg"], RANK_SCORE_DECIMALS)
                required = item.get("best_peak_required_payload_utilization")
                known = item.get("best_pmax_known_constraints_kg")
                return (payload, -round(math.inf if required is None else required,
                                        RANK_SCORE_DECIMALS) if payload <= 0 else 0.,
                        round(known or 0., RANK_SCORE_DECIMALS) if payload <= 0 else 0.,
                        -round(item["best_arm_mass_kg"], RANK_SCORE_DECIMALS))
            best = max(supported, key=rank) if supported else None
            result_rows.append({"layout_number": row["layout_number"], "J2_motor_id": motor_id,
                "rated_voltage_V": motor.get("rated_voltage_V"),
                "listed_rated_current_A": motor.get("rated_current_A"),
                "current_convention": "Unpublished: supply versus phase RMS/peak not established",
                "rated_torque_Nm": motor.get("rated_torque_Nm"),
                "motor_mass_kg": motor.get("mass_kg"), "motor_body_length_mm": motor.get("length_mm"),
                "source_url": motor.get("source_url"), "allocated_hardware_tuples": len(tuples),
                "load_evaluated_hardware_tuples": len(supported),
                "allocated_evaluations": sum(item["allocated_evaluations"] for item in tuples),
                "actual_evaluations": sum(item["evaluations"] for item in tuples),
                "best_tuple_index": None if best is None else best["tuple_index"],
                "best_conservative_pmax_kg": None if best is None else best["best_pmax_kg"],
                "best_pmax_known_constraints_kg": None if best is None else best.get("best_pmax_known_constraints_kg"),
                "best_gravity_score_JG": None if best is None else best.get("best_gravity_score_JG"),
                "best_required_payload_utilization": None if best is None else best.get("best_peak_required_payload_utilization"),
                "best_arm_mass_kg": None if best is None else best.get("best_arm_mass_kg"),
                "best_motor_ids": "" if best is None else ";".join(best["motor_ids"]),
                "best_lengths_m": "" if best is None else ";".join(map(str, best["best_design"]["lengths_m"]))})
    return result_rows


def _report(rows, conditions, folder, *, details=False, voltage_details=False):
    speed_text = ", ".join(f"{value:.5g}" for value in conditions["joint_speed_limits_rad_s"])
    peak_policy = conditions.get("motor_load_policy") == "hold_and_peak_envelope"
    capacity_label = "Transient-envelope Pmax" if peak_policy else "Conservative Pmax"
    policy_note = ("Pmax uses continuous holding and speed-dependent motor transient envelopes; "
        "unverified pulse duration, thermal duty and extrapolated voltage curves remain explicit assumptions. "
        if peak_policy else "Pmax retains instantaneous motor continuous limits. ")
    text = [
        "# Independent configuration comparison", "",
        f"Required fully extended shoulder-to-TCP reach: **{conditions['reach_lower_mm']:g}–"
        f"{conditions['reach_upper_mm']:g} mm**. Required payload: **"
        f"{conditions['payload_lower_g']:g}–{conditions['payload_upper_g']:g} g**.", "",
        f"Each configuration was searched independently with {conditions['cycles']:g} allocated evaluations "
        f"with the same {conditions['pose_count']:g}-state basis, seed {conditions['seed']:g}, "
        f"joint speeds J1–J6 [{speed_text}] rad/s and acceleration "
        f"{conditions['joint_accel_rad_s2']:g} rad/s².", "",
        f"Common J5-to-J6 spacing: {conditions['wrist2_length_m']*1000:g} mm; short L2 coupling: 90 mm.", "",
        f"Reducer rating policy: **{conditions['reducer_load_policy']}**. "
        + policy_note + "Conditional known-constraint capacity omits unverified motor "
        "transient limits; it does not establish a certified payload or duty/lifetime pass.", "",
        "These are the best tested candidates under the recorded ranking: higher selected-policy Pmax; "
        "when Pmax rounds to zero, lower worst required-payload load-to-rating ratio, then higher "
        "conditional known-constraint capacity; lower arm mass breaks remaining ties. Scores are compared "
        "at nine decimal places. If every Pmax is zero, selection therefore minimizes the worst overload "
        "at the required payload. Conditional capacities are measurements of the selected designs, "
        "not independently maximized payloads, and may decrease as overload improves. J_G is retained "
        "for comparison rather than optimized.", "",
        f"| Configuration | Evaluated / allocated | Extension (mm) | L1 / L2 / L3 (mm) | Structural L1 / L2 / L3 (g) | Arm mass (kg) | {capacity_label} (kg) | Conditional capacity (kg) | J_G | Sampled requirements met |",
        "|---|---:|---:|---|---|---:|---:|---:|---:|---|",
    ]
    for row in rows:
        lengths = " / ".join(f"{row[f'L{i}_mm']:.1f}" for i in (1, 2, 3))
        masses = " / ".join(f"{row[f'L{i}_mass_g']:.1f}" for i in (1, 2, 3))
        jg = "unavailable" if row["gravity_score_JG"] is None else f"{row['gravity_score_JG']:.5g}"
        known = "unavailable" if row["pmax_known_constraints_kg"] is None else f"{row['pmax_known_constraints_kg']:.4g}"
        text.append(f"| {row['layout_number']:02d} | {row['evaluations']:g} / {row['allocated_evaluations']:g} | "
                    f"{row['extended_reach_mm']:.1f} | {lengths} | {masses} | "
                    f"{row['arm_mass_kg']:.3f} | {row['pmax_kg']:.4g} | {known} | {jg} | "
                    f"{'Yes' if row['requirements_met'] else 'No'} |")
    text += ["", "Structural masses follow the approved geometry and exclude catalog motor/reducer "
             "cases. Arm mass includes all modeled stock and catalog hardware. Pmax covers the recorded "
             "finite basis; it is not a proof of a global optimum or a supplier rating.", "",
             "The full 3D CoM and inertia of every structural assembly are in each run's link table.", ""]
    for row in rows:
        run = Path(row["run_dir"])
        text.append(f"- **{row['layout_number']:02d}:** " + ", ".join((
            _link("MJCF model", row["model_file"], folder),
            _link("link mass / CoM / inertia", run/"best_links.csv", folder),
            _link("hardware and placement", run/"best_joints.csv", folder),
            _link("full result", run/"RESULT.md", folder))))
    if details:
        text += ["", f"Saved hardware and peak loads at the required upper payload of {conditions['payload_upper_g']:g} g:",
                 "", "- [Selected motor and reducer specifications](selected_hardware.csv)",
                 "- [Gravity, dynamic torque and rating utilization by joint](joint_loads.csv)",
                 "", "Load tables aggregate reachable pose rows only; their count is recorded in each row."]
    if voltage_details:
        text += ["", "- [Best sampled design for each J2 voltage/current alternative](J2_voltage_tradeoff.csv)",
                 "", "The voltage table uses the same conservative-payload/required-load ranking within each J2 model. "
                 "Listed current is not a phase-RMS or motor-heating measurement; no electrical-loss objective is invented."]
    return "\n".join(text)+"\n"


def write_comparison(run_dirs, output_root):
    """Validate three comparable completed runs, then append a new CSV/report.

    Original records and models are read only. Validation finishes before an
    output folder is created; reruns always get a new timestamp/UUID directory.
    """
    sources = list(run_dirs)
    if len(sources) != 3:
        raise InputError("Comparison requires three independent runs, one each for 07, 08 and 09")
    parsed = [_read_run(path) for path in sources]
    if {row["layout_number"] for _, row in parsed} != {7, 8, 9}:
        raise InputError("Comparison requires one distinct run each for configurations 07, 08 and 09")
    common = parsed[0][0]
    for conditions, row in parsed[1:]:
        differences = [name for name in common if conditions[name] != common[name]]
        if differences:
            raise InputError(f"Cannot compare configuration {row['layout_number']:02d}: mismatched "
                             f"study conditions ({', '.join(differences)})")
    rows = sorted((row for _, row in parsed), key=lambda row: row["layout_number"])
    details = _detail_rows(rows, common)
    voltage_details = _j2_voltage_rows(rows)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    folder = Path(output_root).resolve()/f"{timestamp}_{uuid4().hex[:8]}"
    folder.mkdir(parents=True, exist_ok=False)
    store = RunStore(folder)
    store.write_csv("comparison.csv", COMPARISON_COLUMNS, rows)
    if details:
        for filename, records in zip(("selected_hardware.csv", "joint_loads.csv"), details):
            store.write_csv(filename, records[0], records)
    if voltage_details:
        store.write_csv("J2_voltage_tradeoff.csv", voltage_details[0], voltage_details)
    store.write_text("comparison.md", _report(rows, common, folder, details=details is not None,
                                               voltage_details=bool(voltage_details)))
    store.write_json("comparison.json", {"study_conditions": common, "results": rows,
                                         "retention": "Original runs are unchanged; no automatic deletion."})
    return folder

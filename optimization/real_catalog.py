"""Prepare the selected real-hardware study in the established input directory.

Run ``python -m optimization.real_catalog``, then ``run_optimization.py --real``.
Supplier research stays in catalogs/<supplier>; generated real inputs stay in
data/real. General data templates and completed run snapshots are unchanged.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
from uuid import uuid4

from .inputs import DATA_DIR, MOTOR_COLUMNS, REDUCER_COLUMNS, REQUIREMENT_COLUMNS


CATALOG_DIR = Path(__file__).resolve().parent / "catalogs"
REAL_DATA_DIR = DATA_DIR / "real"
PROJECT = "SO_30_550"
MOTOR_FIELDS = (*MOTOR_COLUMNS, "capacity_model", "rotor_inertia_upper_bound_kg_m2",
                "rotor_inertia_basis", "rated_voltage_V", "rated_current_A", "price_usd", "source_url",
                "selectable_joints", "selection_note")
REDUCER_FIELDS = (*REDUCER_COLUMNS, "rated_input_speed_rpm", "allowed_joints", "source_url")


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def compile_inputs(catalog_dir=CATALOG_DIR, *, study="stepperonline", j3_ratio=30):
    if study == "locked-gl":
        from .locked_catalog import compile_locked_inputs
        return compile_locked_inputs(catalog_dir, j3_ratio=j3_ratio)
    if study != "stepperonline":
        raise ValueError("Unknown real-hardware study")
    catalog_dir = Path(catalog_dir)
    source_paths = {"stepperonline/motors.json": catalog_dir / "stepperonline/motors.json",
                    "stepperonline/motor_selection.json": catalog_dir / "stepperonline/motor_selection.json",
                    "sitodrive/reducer_selection.json": catalog_dir / "sitodrive/reducer_selection.json"}
    research = json.loads(source_paths["stepperonline/motors.json"].read_text(encoding="utf-8"))
    screen = json.loads(source_paths["stepperonline/motor_selection.json"].read_text(encoding="utf-8"))
    selection = json.loads(source_paths["sitodrive/reducer_selection.json"].read_text(encoding="utf-8"))
    identities = {row["motor_id"] for row in research["motors"]}
    selected = screen["selectable_motor_ids"]
    if len(selected) != len(set(selected)) or not set(selected) <= identities:
        raise ValueError("Manual motor selection contains duplicate or unknown IDs")
    if set(screen["motor_decisions"]) != identities:
        raise ValueError("Every researched motor needs a recorded manual screening decision")
    joint_options = screen["motor_candidates_by_joint"]
    if set(joint_options) != {"J1", "J2", "J3"} or any(
            not choices or len(set(choices)) != len(choices)
            for choices in joint_options.values()):
        raise ValueError("Manual selection needs nonempty distinct J1/J2/J3 lists")
    used = {motor for choices in joint_options.values() for motor in choices}
    used.add(screen["shared_wrist_motor_id"])
    if used != set(selected):
        raise ValueError("Selectable motors must match the J1-J3 choices and fixed common wrist")
    motors, excluded, sources = [], [], []
    source_eligible_count = 0
    for source in research["motors"]:
        reasons = list(source.get("active_study_exclusion_reasons", []))
        if source.get("rated_voltage_V") not in (24, 36):
            reasons.append("this study includes only published 24 V and 36 V windings")
        price = source.get("price_USD")
        if price is None or not 0 < price <= 50:
            reasons.append("motor-only price unavailable or above the user's USD 50 limit")
        for key in ("rated_torque_Nm", "rated_speed_rpm", "mass_kg", "body_length_mm",
                    "derived_circumscribed_radius_mm"):
            value = source.get(key)
            if value is None or not math.isfinite(value) or value <= 0:
                reasons.append(f"missing published/derived positive {key}")
        if source.get("body_length_mm", 100) >= 100:
            reasons.append("motor body is not shorter than 100 mm")
        if not source.get("hall_sensors_included") or source.get("driver_included"):
            reasons.append("this study requires complete bare Hall-equipped BLDC motors")
        if not reasons:
            source_eligible_count += 1
        decision = screen["motor_decisions"][source["motor_id"]]
        if source["motor_id"] not in selected:
            reasons.append(f"manual {decision['status']}: {decision['reason']}")
        elif decision["status"] != "selectable":
            raise ValueError(f"Selected motor {source['motor_id']} is not marked selectable")
        if reasons:
            if source["motor_id"] in selected:
                raise ValueError(f"Selected motor {source['motor_id']} fails source validation: {'; '.join(reasons)}")
            excluded.append({"motor_id": source["motor_id"], "model": source["model"],
                             "reasons": "; ".join(dict.fromkeys(reasons)), "source_url": source["source_url"]})
            continue
        radius = source["derived_circumscribed_radius_mm"]
        actual_inertia = source.get("rotor_inertia_kg_m2")
        upper = source["mass_kg"] * (radius / 1000) ** 2 if actual_inertia is None else None
        motors.append({
            "motor_id": source["motor_id"],
            "name": f"STEPPERONLINE {source['model']} ({source['rated_voltage_V']:g} V; assembled BLDC)",
            "rated_torque_Nm": source["rated_torque_Nm"], "rated_speed_rpm": source["rated_speed_rpm"],
            "rated_power_W": source.get("rated_power_W"), "stall_torque_Nm": source.get("stall_torque_Nm"),
            "no_load_speed_rpm": source.get("no_load_speed_rpm"), "max_speed_rpm": source.get("max_speed_rpm"),
            "mass_kg": source["mass_kg"], "radius_mm": radius, "length_mm": source["body_length_mm"],
            "rotor_inertia_kg_m2": actual_inertia, "peak_torque_Nm": source.get("peak_torque_Nm"),
            "peak_duration_s": source.get("peak_duration_s"), "capacity_model": "rated_continuous",
            "rotor_inertia_upper_bound_kg_m2": upper,
            "rotor_inertia_basis": "mass_radius_upper_bound" if actual_inertia is None else "published",
            "rated_voltage_V": source["rated_voltage_V"], "rated_current_A": source.get("rated_current_A"),
            "price_usd": price, "source_url": source["source_url"],
            "selectable_joints": ";".join(
                [joint for joint, choices in joint_options.items() if source["motor_id"] in choices] +
                (["J4", "J5", "J6"] if source["motor_id"] == screen["shared_wrist_motor_id"] else [])),
            "selection_note": decision["reason"],
        })
        sources.append(source)
    if not motors:
        raise ValueError("Manual screening produced no source-complete selectable motors")
    expected = {"SITO_HD-14_30": (1,), "SITO_CD-20_30": (2, 3),
                "SITO_CS-11-2XH-A_30": (4, 5, 6)}
    if {row["reducer_id"]: tuple(row["allowed_joints"]) for row in selection["reducers"]} != expected:
        raise ValueError("Reducer source selection does not match the user's exact three models/joints")
    reducers = []
    for row in selection["reducers"]:
        if row["reduction_ratio"] != 30:
            raise ValueError("All selected reducers must be exactly 30:1")
        reducers.append({**{key: row.get(key) for key in REDUCER_COLUMNS},
                         "rated_input_speed_rpm": row["rated_input_speed_rpm"],
                         "allowed_joints": ";".join(map(str, row["allowed_joints"])),
                         "source_url": row["source_url"]})
    requirements = [{"project_id": PROJECT, "payload_lower_g": 500, "payload_upper_g": 500,
                     "reach_lower_mm": 550, "reach_upper_mm": 550}]
    study = {"joint_speed_limits_deg_s": [180, 180, 180, 300, 300, 300],
             "joint_accel_rad_s2": 6, "wrist2_length_mm": 110,
             "reducer_load_policy": "hold_and_repeated_peak",
             "motor_candidates_by_joint": joint_options,
             "shared_wrist_motor_id": screen["shared_wrist_motor_id"]}
    assumptions = {
        "study": PROJECT, "retrieved_client_date": "2026-11-04",
        "catalog_source_sha256": {name: _sha(path) for name, path in source_paths.items()},
        "source_files": {name: {"path": str(path.resolve()), "sha256": _sha(path)}
                         for name, path in source_paths.items()},
        "motor_count": len(motors), "reducer_count": len(reducers),
        "source_eligible_motor_count_before_manual_screen": source_eligible_count,
        "raw_motor_record_count": len(research["motors"]),
        "hardware_combinations_per_layout": math.prod(len(choices) for choices in joint_options.values()),
        "motor_exclusions": len(excluded), "reducer_exclusions": 0,
        "motor_voltage_counts": {str(v): sum(row["rated_voltage_V"] == v for row in motors) for v in (24, 36)},
        "policies": {
            "hardware_scope": "Manually screened assembled STEPPERONLINE 24/36 V Hall BLDC motors at <= USD 50. Only J1-J3 motor choices vary; J4-J6 use three copies of one fixed motor model. Only the user's three 30:1 reducers on their specified joints.",
            "manual_screening": screen["policy"],
            "motor_rating": "Published continuous torque from rest through published rated rpm is assumed; no extrapolation above rated speed. No measured stall thermal map.",
            "motor_rotor_inertia": "Published rotor inertia remains null if unavailable. Simulation uses the explicit upper bound I <= M_motor * R_envelope^2, treating all motor mass at the envelope radius. This is a conservative bound, not a manufacturer rotor-inertia specification.",
            "motor_mass": "Complete assembled motor net mass. Drivers, encoders, wiring and custom shaft adapters are not assigned invented masses.",
            "motor_geometry": "Published body length; conservative circular radius encloses round body or square flange diagonal. Front/rear shaft dimensions are retained in source facts; exact reducer-specific shaft insertion/coupling requires mechanical design.",
            "reducer_geometry": "Exact manufacturer model-specific drawings. CS-11-2XH-A uses nominal diameter 58 mm bounding flange circle and 32 mm axial envelope; generic 82.5 mm page summary is retained as a discrepancy.",
            "reducer_rating": "Gravity holding is screened against rated output torque; instantaneous motion loads are screened against the published repeated-peak output torque, never the momentary emergency rating. Evaluation input speed remains <=2000 rpm. Untimed basis states do not establish cycle-average load or life.",
            "payload_scores": "Primary Pmax uses continuous motor torque throughout the dynamic basis and the explicit rotor-inertia upper bound, so it is a conservative screen. Auxiliary pmax_known_constraints_kg uses holding motor ratings and reducer holding/repeated-peak limits without assigning an unpublished motor transient limit; it is conditional mechanical capacity, not verified actuator payload capacity.",
            "gravity_score": "Active J_G uses the same reducer rated-torque denominators for every design. The motor-capacity-normalized gravity score is retained separately as gravity_capacity_score_JG.",
            "unpublished_reducer_input_torque": "Unknown; output limits enforced, separate input-shaft constraint remains unverified.",
            "transmission": "Ideal lossless reducer and 1:1 local input belts, as in the existing optimization protocol. Published efficiency, friction and backdrive data unavailable; no 75% efficiency is silently substituted.",
            "electrical_objective": "Pmax is a mechanical rating-limited objective. Voltage/current are retained; electrical current, copper loss and common-bus selection are not optimized. Mixed nominal voltages require suitable supplies/drives.",
            "motion": study,
            "geometry_and_mass": "Approved base edge-frame/mount topology and geometry-derived structural mass/CoM/inertia. L2 fixed 90 mm, J5-to-J6 spacing 110 mm. Belt side rails derive the needed clearance from the actual motor body length, within a 110 mm half-width envelope, rather than an arbitrary 82 mm cap. Physical contact, frame depth and stock-span checks remain enforced. Link lengths compensate to preserve exactly 550 mm reach. Unlimited joints; global self-collision checks off by user request.",
        },
        "motor_sources": sources, "reducer_sources": selection, "manual_motor_selection": screen,
    }
    tables = {
        "motor.csv": {"columns": list(MOTOR_FIELDS), "rows": motors},
        "reducer.csv": {"columns": list(REDUCER_FIELDS), "rows": reducers},
        "requirement.csv": {"columns": list(REQUIREMENT_COLUMNS), "rows": requirements},
    }
    return {"tables": tables, "study": study, "assumptions": assumptions, "excluded": excluded}


def install_metadata(bundle, destination=REAL_DATA_DIR):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    names = (*bundle["tables"], "motor_curve.csv", "link_bounds.csv", "study.json", "input_assumptions.json",
             "excluded_hardware.csv", "hardware_provenance.csv", "study_assumptions.csv")
    existing = [destination/name for name in dict.fromkeys(names) if (destination/name).is_file()]
    if existing:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ") + "_" + uuid4().hex[:8]
        archive = destination / "archive" / stamp
        archive.mkdir(parents=True, exist_ok=False)
        for original in existing:
            shutil.copyfile(original, archive/original.name)
    # An archived curve from a different catalog must not leak into a new study.
    if "motor_curve.csv" not in bundle["tables"] and (destination/"motor_curve.csv").exists():
        (destination/"motor_curve.csv").unlink()
    for name, content in (("study.json", bundle["study"]), ("input_assumptions.json", bundle["assumptions"])):
        (destination/name).write_text(json.dumps(content, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    _write_csv(destination/"excluded_hardware.csv", ("kind", "id", "model", "reasons", "source_url"),
               [{"kind": "motor", "id": row["motor_id"], **{key: row[key] for key in
                 ("model", "reasons", "source_url")}} for row in bundle["excluded"]])
    assumptions = bundle["assumptions"]
    provenance = []
    for kind, rows in (("motor", assumptions["motor_sources"]),
                       ("reducer", assumptions["reducer_sources"]["reducers"])):
        for row in rows:
            provenance.append({"kind": kind, "id": row[f"{kind}_id"],
                "source_url": row["source_url"],
                "secondary_source_url": row.get("drawing_url", ""),
                "source_locator": row.get("source_snapshot", row.get("drawing_local_path", "")),
                "source_details_json": json.dumps(row, sort_keys=True)})
    _write_csv(destination/"hardware_provenance.csv", ("kind", "id", "source_url",
               "secondary_source_url", "source_locator", "source_details_json"), provenance)
    _write_csv(destination/"study_assumptions.csv", ("assumption", "value"),
               [{"assumption": key, "value": (value if isinstance(value, str) else json.dumps(value))}
                for key, value in assumptions["policies"].items()])
    return destination


def _write_csv(path, columns, rows):
    with Path(path).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def build_real_catalog(catalog_dir=CATALOG_DIR, output_dir=REAL_DATA_DIR, *, study="stepperonline", j3_ratio=30):
    """Regenerate the selected real inputs; archive every replaced input first."""
    bundle = compile_inputs(catalog_dir, study=study, j3_ratio=j3_ratio)
    destination = install_metadata(bundle, output_dir)
    for name, table in bundle["tables"].items():
        _write_csv(destination/name, table["columns"], table["rows"])
    source = DATA_DIR/"link_bounds.csv"
    target = destination/"link_bounds.csv"
    if source.resolve() != target.resolve():
        shutil.copyfile(source, target)
    if not (destination/"README.md").exists():
        (destination/"README.md").write_text(
            "# Real-hardware study inputs\n\n"
            "Generated by `python -m optimization.real_catalog`. Run from the repository "
            "root with `uv run python run_optimization.py --real`.\n\n"
            f"Selected hardware: {len(bundle['tables']['motor.csv']['rows'])} manually screened "
            "assembled STEPPERONLINE 24/36 V BLDC motors and the "
            "three prescribed 30:1 reducers. Requirements: 500 g payload, 550 mm reach. "
            "Motion and wrist spacing are in study.json; sources and model policies "
            "are in input_assumptions.json and hardware_provenance.csv. "
            "Previous generated inputs are retained in archive/. General data templates "
            "and completed runs are unchanged.\n", encoding="utf-8")
    return bundle["assumptions"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, help="Write the compiled table payload without installing active CSVs")
    parser.add_argument("--catalog-dir", type=Path, default=CATALOG_DIR)
    parser.add_argument("--output-dir", type=Path, default=REAL_DATA_DIR)
    parser.add_argument("--study", choices=("locked-gl", "stepperonline"), default="locked-gl")
    parser.add_argument("--j3-ratio", type=int, choices=(30,), default=30)
    args = parser.parse_args(argv)
    if args.bundle:
        bundle = compile_inputs(args.catalog_dir, study=args.study, j3_ratio=args.j3_ratio)
        args.bundle.parent.mkdir(parents=True, exist_ok=True)
        args.bundle.write_text(json.dumps(bundle, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(args.bundle.resolve())
        return 0
    assumptions = build_real_catalog(args.catalog_dir, args.output_dir, study=args.study, j3_ratio=args.j3_ratio)
    print(f"Installed {assumptions['motor_count']} motors and {assumptions['reducer_count']} "
          f"fixed reducers in {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

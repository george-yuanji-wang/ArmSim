"""Compile the user's fixed lightweight GL / SITO selection.

Source research remains under catalogs/. This adapter creates the same CSV
bundle consumed by the existing real-study workflow; it does not select motors.
"""

import json
import math
from pathlib import Path

from .inputs import MOTOR_COLUMNS, REDUCER_COLUMNS, REQUIREMENT_COLUMNS
from .real_catalog import _sha


PROJECT = "GL_CD20_550"


def compile_locked_inputs(catalog_dir, *, j3_ratio=30):
    if j3_ratio != 30:
        raise ValueError("The current study fixes CD-20 at 50:1 on J2 and 30:1 on J3")
    catalog_dir = Path(catalog_dir)
    paths = {
        "lightweight_motor_survey/smc_cubemars.json": catalog_dir / "lightweight_motor_survey/smc_cubemars.json",
        "lightweight_motor_survey/gl_torque_curves.json": catalog_dir / "lightweight_motor_survey/gl_torque_curves.json",
        "sitodrive/corrected_gl_hardware.json": catalog_dir / "sitodrive/corrected_gl_hardware.json",
    }
    raw, curves, gear_source = (json.loads(path.read_text(encoding="utf-8")) for path in paths.values())
    motors, motor_sources, curve_rows = [], [], []
    assignments = {"GL40_KV210": (1, 2, 3), "GL30_KV290": (4, 5, 6)}
    for model, winding in (("GL40", "KV210"), ("GL30", "KV290")):
        source = next(row for row in raw["motors"] if row["model"] == model and row["winding"].startswith(winding))
        key = f"{model}_{winding}"
        motor_id = f"CUBEMARS_{key}"
        curve = curves["motors"][key]
        row = {"motor_id": motor_id, "name": f"CubeMars {model} {winding} (assembled motor)",
            "rated_torque_Nm": source["torque_Nm"], "rated_speed_rpm": source["rated_speed_rpm"],
            "rated_power_W": source["torque_Nm"] * source["rated_speed_rpm"] * 2 * math.pi / 60,
            "stall_torque_Nm": None, "no_load_speed_rpm": source["no_load_speed_rpm"],
            "max_speed_rpm": None, "mass_kg": source["mass_g"] / 1000,
            "radius_mm": source["diameter_mm"] / 2, "length_mm": source["length_mm"],
            "rotor_inertia_kg_m2": source["rotor_inertia_kg_m2"],
            "peak_torque_Nm": source["peak_torque_Nm"], "peak_duration_s": None,
            "capacity_model": "rated_continuous", "rotor_inertia_basis": "published",
            "rated_voltage_V": source["rated_voltage_V"], "operating_voltage_V": 16,
            "rated_current_A": 4.9 if model == "GL40" else 2.13,
            "price_usd": source["price"], "source_url": source["source_url"],
            "selectable_joints": ";".join(map(str, assignments[key])),
            "selection_note": "Fixed by user; no hardware search. Rated watts are derived shaft power, not electrical consumption."}
        motors.append(row)
        motor_sources.append({**source, **row, "curve": curve})
        for rpm, torque in curve["peak_torque_speed_curve"]:
            curve_rows.append({"motor_id": motor_id, "speed_rpm": rpm,
                "available_torque_Nm": torque, "basis": curve["basis"],
                "operating_voltage_V": 16, "source_voltage_V": curve["source_voltage_V"],
                "source_url": curve["source_url"]})
    wanted = ["SITO_HD-14_50", "SITO_CD-20_50", "SITO_CD-20_30", "SITO_CS-11_30_USER_ENVELOPE"]
    reducers, reducer_sources = [], []
    by_id = {row["reducer_id"]: row for row in gear_source["reducers"]}
    for reducer_id in dict.fromkeys(wanted):
        source = by_id[reducer_id]
        joints = [1] if reducer_id == wanted[0] else [4, 5, 6] if reducer_id == wanted[3] else [
            joint for joint, name in ((2, wanted[1]), (3, wanted[2])) if name == reducer_id]
        reducers.append({**{key: source.get(key) for key in REDUCER_COLUMNS},
            "rated_input_speed_rpm": source["rated_input_speed_rpm"],
            "allowed_joints": ";".join(map(str, joints)), "source_url": source["source_url"]})
        reducer_sources.append({**source, "allowed_joints": joints})
    study = {"joint_speed_limits_deg_s": [180, 180, 180, 300, 300, 300],
        "joint_accel_rad_s2": 6, "wrist2_length_mm": 110,
        "reducer_load_policy": "hold_and_repeated_peak",
        "motor_load_policy": "hold_and_peak_envelope",
        "motor_candidates_by_joint": {f"J{joint}": ["CUBEMARS_GL40_KV210"] for joint in (1, 2, 3)},
        "shared_wrist_motor_id": "CUBEMARS_GL30_KV290"}
    policies = {
        "hardware_scope": "User-locked GL40 KV210 J1-J3; GL30 KV290 J4-J6; HD-14 50:1 J1, CD-20 50:1 J2, CD-20 30:1 J3, CS-11 series 30:1 J4-J6 with user-defined diameter43 x32 mm/200 g envelope. Exactly one hardware tuple per configuration.",
        "motor_rating": "Holding uses original continuous torque; dynamic capacity uses the speed-dependent transient characteristic capped at original peak torque. No increase in continuous rating. Continuous-only capacity is retained separately.",
        "motor_rotor_inertia": "Manufacturer rotor inertia: GL40 7.4e-6 kg m^2 and GL30 2.42e-6 kg m^2; included once in motor acceleration load.",
        "motor_geometry": "Complete bearing-supported motor case dimensions and net masses; custom adapters, encoders, drivers and wiring have no invented masses.",
        "reducer_geometry": "HD-14/CD-20 use retained exact model-specific drawing envelopes. CS-11 uses the user's explicit circular diameter43 x32 mm/200 g proxy, with commercial variant unspecified. Details are retained in corrected_gl_hardware.json; no manufacturer measurement is overwritten.",
        "reducer_rating": "Static holding against rated output torque; dynamic peaks against repeated peak, not emergency/momentary torque. No undocumented speed uprating. Timed average load and life remain unverified.",
        "GL30_operating_voltage": "16 V bus is an explicit engineering assumption, not manufacturer-qualified GL30 operation. Original 12 V curve shifted by measured Kv255 * 4 V = 1020 rpm; original 0.08 Nm continuous and 0.28 Nm peak caps retained.",
        "peak_duration_and_duty": "Unpublished. Pmax is conditional transient-envelope payload capacity, not verified continuous-cycle payload. No fabricated peak duration or RMS duty is supplied.",
        "transmission": "Ideal lossless reducers and 1:1 local belts; no unpublished gearbox friction/efficiency inserted. Unknown reducer input-shaft torque limits remain unverified.",
        "gravity_score": "Record J_G using reducer rated-torque denominators and its separate motor-capacity normalization. Changed reducer ratings mean raw torques should accompany comparisons with older hardware.",
        "geometry_and_mass": "Approved edge-frame configuration topology; geometry-derived aluminium stock mass/CoM/inertia, fixed short 90 mm L2, 110 mm J5-J6 spacing, exact 550 mm extended shoulder-to-TCP reach. Global collisions/joint stops ignored as requested; assembly mounting constraints remain enforced.",
        "motion": study,
    }
    fields = (*MOTOR_COLUMNS, "capacity_model", "rotor_inertia_basis", "rated_voltage_V", "operating_voltage_V",
              "rated_current_A", "price_usd", "source_url", "selectable_joints", "selection_note")
    return {"tables": {
        "motor.csv": {"columns": fields, "rows": motors},
        "motor_curve.csv": {"columns": ("motor_id", "speed_rpm", "available_torque_Nm", "basis", "operating_voltage_V", "source_voltage_V", "source_url"), "rows": curve_rows},
        "reducer.csv": {"columns": (*REDUCER_COLUMNS, "rated_input_speed_rpm", "allowed_joints", "source_url"), "rows": reducers},
        "requirement.csv": {"columns": REQUIREMENT_COLUMNS, "rows": [{"project_id": PROJECT,
            "payload_lower_g": 500, "payload_upper_g": 500, "reach_lower_mm": 550, "reach_upper_mm": 550}]},
        }, "study": study, "excluded": [], "assumptions": {
            "study": PROJECT, "retrieved_client_date": "2026-11-05",
            "motor_count": len(motors), "reducer_count": len(reducers),
            "hardware_combinations_per_layout": 1,
            "catalog_source_sha256": {name: _sha(path) for name, path in paths.items()},
            "source_files": {name: {"path": str(path.resolve()), "sha256": _sha(path)} for name, path in paths.items()},
            "policies": policies, "motor_sources": motor_sources,
            "reducer_sources": {"reducers": reducer_sources}, "locked_j3_ratio": j3_ratio}}

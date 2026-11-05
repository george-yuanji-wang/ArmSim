"""Normalize captured STEPPERONLINE product specifications without filling gaps.

The source snapshots are the complete rendered public product-page text returned
by the web retrieval tool. Direct HTTP requests were refused with 403 responses
on the retrieval date. Rebuilding uses these retained snapshots and performs no
network requests. It does not modify the optimizer's active input files.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parent
RETRIEVED_DATE = "2026-11-04"
SNAPSHOTS = ROOT / "source_snapshots" / "20261104"


def first_number(text):
    if text is None:
        return None
    value = re.search(r"[-+]?\d+(?:\.\d+)?", text)
    return float(value.group()) if value else None


def field(section, label):
    found = re.search(r"(?:^|\n)\s*\*?\s*" + label + r"\s*:?\s*([^\n]+)",
                      section, re.IGNORECASE)
    return found.group(1).strip() if found else None


def number(section, label):
    return first_number(field(section, label))


def parse_snapshot(path):
    raw = path.read_text(encoding="utf-8")
    text = re.sub(r"^L\d+(?:@P\d+)?:\s?", "", raw, flags=re.MULTILINE)
    model = path.stem
    if "### Electrical Specification" not in text or "### Physical Specification" not in text:
        raise ValueError(f"No complete specifications in {path}")
    electrical = text.split("### Electrical Specification", 1)[1].split("### Physical Specification", 1)[0]
    physical = text.split("### Physical Specification", 1)[1].split("### Connection", 1)[0]
    observed_model = field(electrical, "Manufacturer Part Number")
    if observed_model != model:
        raise ValueError(f"Model mismatch in {path}: {observed_model!r} != {model!r}")
    url = re.search(r"Redirected to URL:\s*(https://\S+?);", raw)
    if url is None:
        url = re.search(r"\((https://www\.omc-stepperonline\.com/[^\s)]+)\)", raw)
    source_url = url.group(1) if url else None
    price_match = re.search(r"(?:^|\n)from\s*\n+\s*\$([\d.]+)", text, re.IGNORECASE)
    if price_match is None:
        price_match = re.search(r"(?:^|\n)\$([\d.]+)\s*(?:\n|$)", text)
    if price_match is None:
        raise ValueError(f"No own-product unit price in {path}")
    frame = field(physical, "Frame Size")
    dimensions = [float(x) for x in re.findall(r"\d+(?:\.\d+)?", frame or "")]
    if len(dimensions) == 1:
        shape, diameter = "round", dimensions[0]
        width = height = diameter
        radius = diameter / 2
    elif len(dimensions) == 2:
        shape, diameter = "square", None
        width, height = dimensions
        radius = math.hypot(width, height) / 2
    else:
        raise ValueError(f"Unknown physical frame dimensions in {path}: {frame!r}")
    mass_text = field(physical, "Weight")
    mass = first_number(mass_text)
    if mass is not None and "kg" not in mass_text.lower():
        if re.search(r"\bg\b", mass_text, re.IGNORECASE) or mass_text.endswith("g"):
            mass /= 1000
        else:
            raise ValueError(f"Unknown mass unit: {mass_text!r}")
    front = number(physical, "Shaft Length")
    rear = number(physical, "Rear Shaft Length")
    body = number(physical, "(?:Body Length|Motor Height)")
    own_header = text.split("## Downloads", 1)[0]
    minimum = re.search(r"Min\. order\s*\n\s*(\d+)", own_header, re.IGNORECASE)
    preorder = bool(re.search(r"China\s+(?:Pre-order|Backorder)", own_header, re.IGNORECASE))
    record = {
        "motor_id": "SO_" + model,
        "manufacturer": "STEPPERONLINE",
        "model": model,
        "name": "STEPPERONLINE " + model + " complete framed BLDC motor",
        "source_url": source_url,
        "retrieved_date": RETRIEVED_DATE,
        "source_snapshot": str(path.relative_to(ROOT)).replace("\\", "/"),
        "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "source_format": "web-tool-rendered-product-page-text",
        "assembly_type": "complete framed motor with shaft and bearings; no reducer",
        "driver_included": False,
        "position_encoder_included": False,
        "hall_sensors_included": "Hall" in physical or bool(re.search(r"\+5V\s*\|\s*GND", text)),
        "price_USD": float(price_match.group(1)),
        "price_scope": "one motor; China listing; excludes driver, encoder, freight and tax",
        "minimum_order_qty_published": int(minimum.group(1)) if minimum else None,
        "preorder_or_backorder_observed": preorder,
        "rated_voltage_V": number(electrical, r"Rated\s*Voltage"),
        "rated_torque_Nm": number(electrical, "Rated Torque"),
        "rated_speed_rpm": number(electrical, "Rated Speed"),
        "rated_power_W": number(electrical, "Rated Power"),
        "rated_current_A": number(electrical, "Rated Current"),
        "rated_current_published_text": field(electrical, "Rated Current"),
        "rated_current_convention": "Product page does not distinguish DC, phase RMS or phase peak current",
        "no_load_speed_rpm": number(electrical, "No[- ]?load Speed"),
        "pole_count": number(electrical, "Poles"),
        "stall_torque_Nm": None,
        "peak_torque_Nm": None,
        "peak_duration_s": None,
        "max_speed_rpm": None,
        "rotor_inertia_kg_m2": None,
        "rotor_inertia_status": "not published in captured product-page specifications",
        "body_shape": shape,
        "body_diameter_mm": diameter,
        "body_width_mm": width,
        "body_height_mm": height,
        "body_length_mm": body,
        "mass_kg": mass,
        "mass_published_text": mass_text,
        "mass_scope": "net motor mass from Physical Specification; gross shipping mass is never substituted",
        "front_shaft_diameter_mm": number(physical, "Shaft Diameter"),
        "front_shaft_length_mm": front,
        "rear_shaft_diameter_mm": number(physical, "Rear Shaft Diameter"),
        "rear_shaft_length_mm": rear,
        "bearing_published_text": field(physical, "Bearing Type"),
        "derived_circumscribed_radius_mm": radius,
        "derived_circumscribed_radius_definition": "round: diameter/2; square: sqrt(width^2+height^2)/2",
        "derived_named_axial_span_lower_bound_mm": body + (front or 0) + (rear or 0),
        "axial_span_scope": "body + named front/rear shaft lengths; drawing shoulder/cover protrusions not fully certified",
        "notes": [],
    }
    if shape == "square":
        record["notes"].append("Square frame width is not its circular envelope diameter; use the derived circumscribed radius for cylinder models.")
    if model == "57BLR70-24-02":
        record["notes"].append("Product states updated model: current physical body length 74 mm overrides legacy 69 mm URL slug; shape is round despite a generic NEMA23 square category label.")
    if model in ("42BLS40-24-01", "42BLS60-24-01"):
        record["notes"].append("Net mass absent from captured HTML; gross product/shipping mass is excluded. Drawing review is required before optimizer import.")
    if model == "42BYA73-24-01":
        record["notes"].append("Current product net mass is 0.42 kg; an earlier indexed drawing reported 0.40 kg. Retain the current larger value and verify supplied drawing revision.")
    if model == "57BLR50-24-01":
        record["notes"].append("Current physical specification uses 49.1 mm and 0.50 kg; older translated listings used approximately 50 mm and 0.52 kg. Verify supplied revision.")
    if record["rated_voltage_V"] == 36:
        record["notes"].append("This is a separately listed 36 V winding/operating point, not an extrapolated 24 V motor rating.")
    if model in ("57BYA94-36-01", "57BSD96-36-01"):
        record.update({
            "phase_resistance_ohm": number(electrical, "Resistance/phase"),
            "phase_inductance_mH": number(electrical, "Inductance/phase"),
            "back_emf_published_V_per_krpm": number(electrical, "BACK EMF"),
            "back_emf_convention": "Product page does not explicitly identify line/phase or RMS/peak convention",
        })
        record["notes"].append(
            "36 V lower-listed-current J2 alternative retained alongside the 24 V motor. "
            "Rated current is not established as supply, phase RMS or phase peak current; "
            "winding resistance and amperes alone do not certify motor heating or efficiency.")
    reasons = []
    if record["price_USD"] <= 0 or record["price_USD"] > 50:
        reasons.append("price outside positive USD 50 ceiling")
    if record["rated_voltage_V"] not in (24, 36):
        reasons.append("optional voltage outside active 24/36 V study")
    if mass is None:
        reasons.append("net motor mass unpublished in captured HTML")
    if body is None or body >= 100:
        reasons.append("body length is not strictly below 100 mm")
    if minimum and int(minimum.group(1)) > 1:
        reasons.append("published minimum order exceeds one motor")
    record["active_study_exclusion_reasons"] = reasons
    record["active_study_source_fields_complete_except_inertia"] = not reasons
    return record


def build():
    records = [parse_snapshot(path) for path in sorted(SNAPSHOTS.glob("*.txt"))]
    if not records:
        raise ValueError("No captured primary source snapshots")
    ids = [record["motor_id"] for record in records]
    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate motor IDs")
    catalog = {
        "schema_version": 1,
        "retrieved_date": RETRIEVED_DATE,
        "manufacturer": "STEPPERONLINE",
        "source_policy": "Published complete-motor facts only. Unknown numeric values are null; no winding, thermal, peak or maximum-speed ratings are extrapolated.",
        "retrieval_policy": "Retained web-tool product-page snapshots. Direct urllib requests returned HTTP 403; these are rendered source captures, not claimed raw HTML.",
        "optimizer_policy": "Catalog does not assign rotor inertia. The active study must explicitly choose and record a handling policy for missing inertia before import.",
        "motors": records,
    }
    (ROOT / "motors.json").write_text(json.dumps(catalog, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    audit = {
        "record_count": len(records),
        "source_fields_complete_except_inertia": sum(r["active_study_source_fields_complete_except_inertia"] for r in records),
        "voltage_counts": {str(int(v)): sum(r["rated_voltage_V"] == v for r in records) for v in sorted({r["rated_voltage_V"] for r in records})},
        "missing_net_mass_models": [r["model"] for r in records if r["mass_kg"] is None],
        "missing_rotor_inertia_models": [r["model"] for r in records if r["rotor_inertia_kg_m2"] is None],
        "excluded_from_active_study": [{"model": r["model"], "reasons": r["active_study_exclusion_reasons"]} for r in records if r["active_study_exclusion_reasons"]],
        "refresh": "Recapture source pages into a new dated snapshot directory and update the builder date/path deliberately; running this builder never refetches or mutates retained snapshots.",
    }
    (ROOT / "catalog_audit.json").write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(audit, indent=2))
    return catalog


if __name__ == "__main__":
    build()

"""Capture current official product ratings; dimensions use inspected 2025 drawings."""

from concurrent.futures import ThreadPoolExecutor
import csv
import html
import json
from pathlib import Path
import re
import urllib.request

ROOT = Path(__file__).resolve().parent
SOURCES = ROOT / "sources"
COLUMNS = "manufacturer model motor_id construction voltage_V winding rated_torque_Nm rated_speed_rpm rated_power_W peak_torque_Nm peak_duration_s stall_torque_Nm no_load_speed_rpm max_speed_rpm mass_kg outer_diameter_mm body_length_mm frame_width_mm frame_height_mm cylindrical_envelope_diameter_mm envelope_basis rotor_inertia_kg_m2 rated_current_A peak_current_A torque_constant_Nm_A phase_resistance_ohm phase_inductance_mH nominal_size_mm temperature_conditions availability source_url secondary_source_url source_locator retrieved_date notes total_axial_envelope_mm".split()
FAMILIES = (
    (range(539472, 539476), 30, 30, 42, 77, 302, "9402695024670"),
    (range(539480, 539485), 30, 45, 42, 77, 303, "9402695155742"),
    (range(539476, 539480), 30, 50, 64, 99, 304, "9402695221278"),
    ((449463, 449464), 40, 50, 26, 64.5, 306, "9402695352350"),
)


def clean(value):
    return re.sub(r"\s+", " ", html.unescape(re.sub(r"<[^>]+>", " ", value))).strip()


def capture(item):
    number, diameter, power, length, total, page, token = item
    url = f"https://www.maxongroup.com/maxon/view/product/{number}"
    raw = urllib.request.urlopen(url, timeout=45).read().decode("utf-8", errors="replace")
    (SOURCES / f"{number}.html").write_text(raw, encoding="utf-8")
    table = {}
    for row in re.findall(r"<tr\b[^>]*>(.*?)</tr>", raw, re.S | re.I):
        cells = re.findall(r"<td\b[^>]*>(.*?)</td>", row, re.S | re.I)
        if len(cells) == 2:
            table[clean(cells[0])] = clean(cells[1])

    def value(label, factor=1.):
        text = table[label]
        numeric = re.search(r"[-+]?\d[\d'’]*(?:\.\d+)?", text)
        if numeric is None:
            raise ValueError(f"No numeric {label} on {url}")
        return float(numeric[0].replace("'", "").replace("’", ""))*factor

    product = clean(re.search(r"<h1\b[^>]*>(.*?)</h1>", raw, re.S | re.I)[1])
    assert f"{diameter}" in product and "brushless" in product.lower(), product
    row = dict.fromkeys(COLUMNS, "")
    row.update({
        "manufacturer": "maxon", "model": f"EC-i {diameter} {power}W {number}",
        "motor_id": f"MAXON_{number}", "construction": "framed BLDC; Hall sensors; external controller",
        "voltage_V": value("Nominal voltage"), "winding": f"part {number}",
        "rated_torque_Nm": value("Nominal torque (max. continuous torque)", .001),
        "rated_speed_rpm": value("Nominal speed"), "rated_power_W": power,
        "stall_torque_Nm": value("Stall torque", .001),
        "no_load_speed_rpm": value("No load speed"), "max_speed_rpm": value("Max. speed"),
        "mass_kg": value("Weight", .001), "outer_diameter_mm": diameter,
        "body_length_mm": length, "total_axial_envelope_mm": total,
        "cylindrical_envelope_diameter_mm": diameter,
        "envelope_basis": "drawing body OD and maximum body length; two shaft protrusions listed separately",
        "rotor_inertia_kg_m2": value("Rotor inertia", 1e-7),
        "rated_current_A": value("Nominal current (max. continuous current)"),
        "torque_constant_Nm_A": value("Torque constant", .001),
        "phase_resistance_ohm": value("Terminal resistance"),
        "phase_inductance_mH": value("Terminal inductance"),
        "nominal_size_mm": diameter,
        "temperature_conditions": "manufacturer continuous-operation envelope; rated torque depends on mounting and thermal conditions",
        "availability": "listed in official online catalog; stock and lead time not verified",
        "source_url": url,
        "secondary_source_url": f"https://www.maxongroup.com.tw/medias/sys_master/root/{token}/Cataloge-Page-EN-{page}.pdf",
        "source_locator": f"online specifications; March 2025 catalog printed p.{page}, dimension drawing",
        "retrieved_date": "2026-11-01",
        "notes": "Online numerical ratings take precedence over provisional catalog values where they differ. Assigned family power is retained as published, not recalculated. Peak torque/duration are unpublished; stall torque is a distinct rating. Dimensions omit connectors/cable bend space. 1 g cm^2 = 1e-7 kg m^2.",
    })
    for key in ("rated_torque_Nm", "mass_kg", "rotor_inertia_kg_m2"):
        assert row[key] > 0, (number, key)
    return row


def main():
    SOURCES.mkdir(parents=True, exist_ok=True)
    items = [(n, d, p, length, total, page, token)
             for numbers, d, p, length, total, page, token in FAMILIES for n in numbers]
    with ThreadPoolExecutor(max_workers=3) as pool:
        rows = sorted(pool.map(capture, items), key=lambda row: row["motor_id"])
    (ROOT / "motors.json").write_text(json.dumps(rows, indent=2)+"\n", encoding="utf-8")
    with (ROOT / "motors.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Captured {len(rows)} maxon winding variants")


if __name__ == "__main__":
    main()

"""Retain reproducible samples from CubeMars' raster torque/speed charts.

Pixel coordinates are the explicitly inspected axes, not a fitted motor model.
The 16 V GL30 curve is an engineering voltage-shift assumption, kept separate
from its original 12 V observations and its unchanged thermal/current ratings.
"""

from pathlib import Path
import hashlib
import json

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parent
SOURCES = {
    "GL30_KV290": {
        "file": "GL30_KV290_12V.jpg", "voltage_V": 12,
        "url": "https://store.tmotor.com/images/202408/121723431229683127.jpg",
        "axes": (265, 898, 200, 710, .300, 3500),
        "torques_Nm": [.01, .03, .05, .08, .10, .12, .15, .18, .21, .24, .27],
        "peak_torque_Nm": .28, "kv_rpm_per_V": 255,
    },
    "GL40_KV210": {
        "file": "GL40_KV210_16V.jpg", "voltage_V": 16,
        "url": "https://store.tmotor.com/images/202408/121723430944196572.jpg",
        "axes": (258, 900, 1103, 1620, .800, 3500),
        "torques_Nm": [.01, .08, .16, .24, .32, .40, .48, .56, .64, .72, .75],
        "peak_torque_Nm": .75, "kv_rpm_per_V": 195,
    },
}


def sample(source):
    path = ROOT / "source_curves" / source["file"]
    pixels = np.asarray(Image.open(path).convert("RGB"))
    x0, x1, y0, y1, torque_max, rpm_max = source["axes"]
    measurements = []
    for torque in source["torques_Nm"]:
        x = int(round(x0 + torque / torque_max * (x1 - x0)))
        region = pixels[y0+3:y1-3, x-1:x+2].astype(int)
        mask = ((region[:, :, 0] > 180) & (region[:, :, 1] > 90)
                & (region[:, :, 1] < 190) & (region[:, :, 2] < 100))
        ys = np.where(mask)[0]
        if not len(ys):
            raise ValueError(f"No orange speed trace at {path.name}, torque={torque}")
        y = float(np.median(ys)) + y0 + 3
        rpm = round((y1 - y) / (y1 - y0) * rpm_max)
        measurements.append({"torque_Nm": torque, "speed_rpm": rpm,
                             "pixel_x": x, "pixel_y": y})
    return {"source_url": source["url"], "source_file": str(path.relative_to(ROOT)),
            "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "source_voltage_V": source["voltage_V"],
            "axes": source["axes"], "observations": measurements}


def main():
    records = {}
    for model, source in SOURCES.items():
        record = sample(source)
        shift = source["kv_rpm_per_V"] * (16 - source["voltage_V"])
        # Each curve is a motor characteristic, not a continuous-duty envelope.
        points = [[0, source["peak_torque_Nm"]]]
        points.extend([row["speed_rpm"] + shift, row["torque_Nm"]]
                      for row in reversed(record["observations"]))
        # The exact no-load specification supplies a zero-torque endpoint.
        no_load = 3060 if model == "GL30_KV290" else 3120
        points.append([no_load + shift, 0])
        record.update(operating_voltage_V=16, speed_shift_rpm=shift,
            peak_torque_speed_curve=points,
            basis=("voltage_shift_estimate_from_official_12V_curve" if shift else
                   "digitized_official_16V_curve"),
            qualification=("16 V operation and shifted curve unverified by manufacturer" if shift else
                           "manufacturer curve; transient duration/thermal duty unspecified"))
        records[model] = record
    output = ROOT / "gl_torque_curves.json"
    output.write_text(json.dumps({"date": "2026-11-05", "motors": records,
        "method": "Orange trace sampled at fixed torque coordinates; approximate raster readings. Original observations retained. No quantified validation error is available for the GL30 16 V extrapolation. No continuous rating increase or peak-current increase.",
        "voltage_shift_reference": "https://support.maxongroup.com/hc/en-us/articles/360012252780-What-supply-voltage-is-required-by-a-controller"},
        indent=2) + "\n", encoding="utf-8")
    print(output)
    for model, record in records.items():
        points = np.asarray(record["peak_torque_speed_curve"])
        print(model, "available at 1500rpm:", float(np.interp(1500, points[:, 0], points[:, 1])))


if __name__ == "__main__":
    main()

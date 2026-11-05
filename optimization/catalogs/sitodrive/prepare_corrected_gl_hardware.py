"""Retain the approved CD-20 selection and the user's CS-11 model envelope.

Manufacturer observations in the earlier source files are read only. The
43 x 32 mm wrist envelope is explicitly a user modeling instruction, not a
new measurement of a particular CS-11 commercial variant.
"""

from copy import deepcopy
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def main():
    prior = json.loads((ROOT/"locked_gl_hardware.json").read_text(encoding="utf-8"))
    originals = json.loads((ROOT/"reducer_selection.json").read_text(encoding="utf-8"))
    catalog = json.loads((ROOT/"catalog_pdf_facts.json").read_text(encoding="utf-8"))
    hd = deepcopy(next(row for row in prior["reducers"] if row["model"] == "HD-14"))
    cd_source = next(row for row in originals["reducers"] if row["model"] == "CD-20")
    cd_ratings = next(row for row in catalog if row["model"] == "CD-20")["ratings"]
    reducers = [hd]
    for ratio, joint in ((50, 2), (30, 3)):
        row = deepcopy(cd_source)
        ratings = cd_ratings[str(ratio)]
        row.update(ratings)
        row.update(reducer_id=f"SITO_CD-20_{ratio}",
            name=f"SITO CD-20 {ratio}:1 (fixed GL study; drawing-verified envelope)",
            reduction_ratio=ratio, allowed_joints=[joint],
            continuous_output_torque_Nm=ratings["rated_output_torque_at_2000rpm_Nm"],
            peak_output_torque_Nm=ratings["repeated_peak_output_torque_Nm"],
            source_rating_check="Retained model-specific manufacturer CD-20 table: 50:1 on J2; 30:1 on J3.")
        reducers.append(row)
    cs_source = next(row for row in originals["reducers"] if row["model"] == "CS-11-2XH-A")
    cs = {key: deepcopy(value) for key, value in cs_source.items()
          if not key.startswith("drawing_") and key not in {"interface_notes", "retained_source_discrepancy"}}
    cs.update(reducer_id="SITO_CS-11_30_USER_ENVELOPE", model="CS-11 series (variant unspecified)",
        name="SITO CS-11 series 30:1 (user model: dia43 x32 mm; 200 g)",
        allowed_joints=[4, 5, 6], radius_mm=21.5, diameter_mm=43, length_mm=32,
        mass_kg=.2, mass_qualifier="user_assumed", geometry_basis="user_defined_cylindrical_simulation_envelope",
        geometry_envelope_status="User instructs nominal circular diameter 43 mm, axial length 32 mm and mass 200 g. Exact commercial variant is not specified; this is not a claim about CS-11-1U-E or its flange corners.",
        source_rating_model="CS mini-11 shared published 30:1 ratings",
        modeling_instruction_date="2026-11-05",
        source_rating_check="Use CS mini-11 30:1 family torque/speed data; geometry/mass follow the user's explicit simulation assumption.")
    reducers.append(cs)
    selection = {"J1": hd["reducer_id"], "J2": "SITO_CD-20_50", "J3": "SITO_CD-20_30",
                 **{f"J{j}": cs["reducer_id"] for j in (4,5,6)}}
    result = {"schema_version": 1, "study": "GL_CD20_550", "client_date": "2026-11-05",
        "purpose": "User-corrected fixed selection; prior manufacturer/source records remain unchanged.",
        "selection": selection, "reducers": reducers,
        "source_files": ["locked_gl_hardware.json", "reducer_selection.json", "catalog_pdf_facts.json"]}
    output = ROOT/"corrected_gl_hardware.json"
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()

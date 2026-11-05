# Input CSV columns

This folder contains the general editable input templates: header-only motor
and reducer catalogs, the `EXAMPLE` requirements row, and the legacy link schema.
The current **SO_30_550** real-hardware study has its own generated inputs in
[`real/`](real/README.md). Supplier research belongs in `../catalogs/`, and
completed results belong in `../runs/`.

Run from the repository root:

```powershell
uv run python run_optimization.py --real
```

The schemas below apply to `motor.csv`, `reducer.csv`, `requirement.csv` and
`link_bounds.csv` in any selected input folder. An optional `study.json` supplies
motion and geometry overrides for that folder.
Numeric cells contain numbers only: write `500`, not
`500g`. Units are in the headers. Equal lower/upper bounds fix a value. IDs must
be unique; project IDs also become output folder names. UTF-8 CSVs with
an Excel-style BOM are accepted.

## motor.csv

| Column | Meaning |
|---|---|
| motor_id | Your unique stable identifier |
| name | Readable product/model name |
| capacity_model | `rated_continuous` in the real study; rated torque is assumed from rest through rated rpm |
| rated_torque_Nm | Continuous/rated motor-shaft torque, N·m |
| rated_speed_rpm | Speed at the rated operating point, rpm |
| rated_power_W | Optional rated mechanical output power, W; blank allowed |
| stall_torque_Nm | Stall torque, N·m; not treated as continuous torque |
| no_load_speed_rpm | No-load speed at the intended drive conditions, rpm |
| max_speed_rpm | Mechanical/allowed speed limit, rpm; distinct from no-load speed |
| mass_kg | Complete motor mass, kg |
| radius_mm | Circumscribed body radius, mm; square frames use their diagonal |
| length_mm | Published motor body length, mm; shaft dimensions remain in the source catalog |
| rotor_inertia_kg_m2 | Rotor inertia about its spinning shaft, kg·m² |
| peak_torque_Nm | Optional permitted peak torque, N·m; currently stored only |
| peak_duration_s | Required if peak torque exceeds rated torque; currently stored only |
| rotor_inertia_upper_bound_kg_m2 | Explicit simulation bound when published rotor inertia is unavailable |
| rotor_inertia_basis | `published` or `mass_radius_upper_bound` |
| rated_voltage_V, rated_current_A | Source electrical operating point; current convention retained in source notes |
| price_usd, source_url | Motor-only source price in USD and primary product page |
| selectable_joints | Joint roles permitted by the current manual selection; generated real-study catalogs only |
| selection_note | Recorded package-screening reason; does not assert simulation feasibility |

Rated power, voltage and current are retained but are not separate optimization
objectives. The active `rated_continuous` policy uses published rated torque
only through published rated speed; it does not extrapolate a missing stall
curve. Unpublished stall torque and mechanical maximum speed stay blank. The
legacy `legacy_linear` policy still requires stall/no-load/maximum-speed data.

The selected STEPPERONLINE motors in `real/` do not publish rotor inertia.
Their `rotor_inertia_kg_m2` stays blank;
the separately labeled upper bound is `mass_kg * (radius_mm / 1000)^2`. This
treats all motor mass as if rotating at the outer radius and is a conservative
simulation bound, not a supplier specification. It can substantially reduce
the predicted fast-motion capacity. Motors use their complete net mass;
drivers, encoders and custom adapters are not assigned invented masses.

## reducer.csv

| Column | Meaning |
|---|---|
| reducer_id | Your unique stable identifier |
| name | Readable product/model name |
| reduction_ratio | Motor revolutions per output revolution; e.g. `100` |
| mass_kg | Complete supplied reducer mass, kg |
| radius_mm | Outer envelope radius, mm |
| length_mm | Axial envelope length, mm |
| continuous_output_torque_Nm | Continuous output torque, N·m |
| peak_output_torque_Nm | Permitted peak output torque, N·m; currently stored only |
| max_input_torque_Nm | Allowed input torque, N·m |
| max_input_speed_rpm | Allowed input speed, rpm |
| rated_input_speed_rpm | Speed reference for the rated torque; 2000 rpm in this study |
| allowed_joints | Semicolon-separated joint numbers; `1`, `2;3`, or `4;5;6` in this study |
| source_url | Primary manufacturer product/drawing source |

Unpublished `max_input_torque_Nm` may be blank. The published output torque
rating remains enforced. Peak ratings are retained but do not increase the
continuous score. Local mounts and hardware envelopes must pass assembly
checks; catalog IDs do not establish exact shaft/interface compatibility.

## requirement.csv — one row per independent project

Only these five columns belong in this file:

```csv
project_id,payload_lower_g,payload_upper_g,reach_lower_mm,reach_upper_mm
EXAMPLE,500,1000,550,550
```

| Columns | Meaning |
|---|---|
| project_id | Unique filename-safe project identifier |
| payload_lower_g, payload_upper_g | Required payload interval, grams; Pmax must meet the upper bound |
| reach_lower_mm, reach_upper_mm | Bounds on maximum fully extended shoulder-to-TCP distance, mm |

One row contains **both** payload and reach requirements for the same project.
Adding another row creates another project; it does not add a condition to the
previous row. Requirements never silently change while a run is executing:
the run stores copies of the CSVs and the resolved settings.

Every row in the selected input folder is active. Simulation/search defaults
live in `../settings.py`. The current real study fixes both payload bounds at
500 g and reach at 550 mm in `real/requirement.csv`. Its `real/study.json`
supplies these overrides without adding requirements columns:

```json
{
  "joint_speed_limits_deg_s": [180, 180, 180, 300, 300, 300],
  "joint_accel_rad_s2": 6,
  "wrist2_length_mm": 110
}
```

Both the original CSVs and resolved motion settings are snapshotted per run.

For configurations 07-09, L2 is fixed at 90 mm and the exact extension is
`L1 + sqrt(90^2 + L3^2) + 190` mm in this study. The final 190 mm is the
110 mm J5-J6 wrist plus 80 mm tool offset, allowing space for complete motors;
pedestal height is excluded. The current `550,550` bounds fix this maximum at
550 mm. An interval would constrain the maximum rather than define a Cartesian
shell of required poses.

Optimization runs 07, 08 and 09 independently by default, creating project IDs
with `_07`, `_08` and `_09` suffixes. Use `--layout 7`, `--layout 8` or
`--layout 9` to run only that configuration. Each search has 200 candidate
evaluations over the 1,008-state joint-space basis.

After optimization, `best_model.xml` in the new run folder contains the best arm
as complete standalone MJCF, including frame/mount geometry, selected drivetrain,
actuators, sensors and coordinate sites. The result folder also includes named
link and joint CSVs.

## link_bounds.csv — legacy compatibility input

Keep the existing six rows and columns so saved input schemas remain readable.
The file is parsed and snapshotted, but **the current builder does not use its
old mass, CoM, spread or radius budgets to determine physical mass properties**.

| Column | Meaning |
|---|---|
| body_name | Existing structural body identifier; keep all six rows |
| mass_lower_kg, mass_upper_kg | Bounds on structural mass, excluding catalog hardware |
| com_fraction_lower, com_fraction_upper | Bounds on COM position along the body, fraction of length |
| spread_fraction_lower, spread_fraction_upper | Bounds on longitudinal variance divided by length²; 0…0.25 |
| radius_mm | Fixed equivalent cross-section radius used for inertia, mm |

`upper_arm_link`, `forearm_link`, `wrist_1_link` correspond to L1, L2, L3.
L2 is fixed at 90 mm. `shoulder_link`, `wrist_2_link`, `tool_flange_link` have
fixed lengths of 120, 110 and 80 mm in the current real study. General defaults
and original numbered base models use 70 mm for `wrist_2_link`. Structural
mass, local XYZ CoM and full inertia now come from the approved
edge-frame/mount/output geometry, using
aluminium stock density 2700 kg/m^3 and the base builder's explicit small
transmission-part masses. Catalog motors/reducers retain their catalog masses.
Mass and CoM are derived outputs, not independent optimization variables.

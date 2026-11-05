# Active real-hardware inputs: GL_CD20_550

The [completed GL_CD20_550 comparison](../../runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/comparison.md)
records 200 evaluations over all 1,008 poses for each configuration. Pmax is
**0.9184 / 0.9008 / 0.9098 kg** for 07 / 08 / 09, meeting the sampled 500 g
requirement at 550 mm under the conditional transient-envelope policy and
the GL30 16 V curve estimate. These finite-budget results do not prove a global
optimum. The published `model/best-07-base-6dof.xml`, `best-08-base-6dof.xml`
and `best-09-base-6dof.xml` match their saved scorers; `best_model.xml` selects 07.
The [independent audit](../../runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/independent_audit.json)
passed. A [same-geometry recheck with the official GL30 12 V curve](../../runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/voltage_sensitivity.csv)
also meets 500 g for all three, without reoptimization or thermal-duty verification.

The [completed GL_FIXED_550 comparison](../../runs/comparisons/GL_FIXED_550/20261105T083535_687425Z_21e4c5fd/comparison.md)
is historical: it used CD-17 50:1 on both J2 and J3 and a conservative
CS-11-1U-E drawing envelope. Its scores do not describe this corrected setup.

This generated folder contains **two fixed CubeMars motor models and four
SITO reducer model/ratio records**, for independent optimization of configurations
07, 08 and 09. No other catalog hardware is selectable:

| Joints | Motor | Reducer | Ratio |
|---|---|---|---:|
| J1 | GL40 KV210 | HD14_50 | 50:1 |
| J2 | GL40 KV210 | CD20_50 | 50:1 |
| J3 | GL40 KV210 | CD20_30 | 30:1 |
| J4/J5/J6 | GL30 KV290 | CS11 series (variant unspecified) | 30:1 |

The joint ratios are **50/50/30/30/30/30**, with a common **16 V bus**.
The CS11 model uses the user-specified **43 mm diameter × 32 mm axial envelope
and 200 g mass**; its exact product variant is unspecified. Each layout has one
hardware tuple. Its search adjusts link allocation and permitted mounting
offsets, using **200 load evaluations per layout**, all **1,008 reference
poses**, and checkpoints every **20** evaluations.

From the repository root:

```powershell
uv run python -m optimization.real_catalog --study locked-gl
uv run python run_optimization.py --real --cycles 200 --checkpoint-every 20
```

Regeneration archives previous generated inputs before replacing them. Add
`--layout 7` to the run command for 07 alone. General editable templates remain
in the parent `optimization/data/` folder. Retained manufacturer sources live
under `optimization/catalogs/`; previous results stay in their own run folders.

The requirement file keeps its existing five columns and units:

```csv
project_id,payload_lower_g,payload_upper_g,reach_lower_mm,reach_upper_mm
GL_CD20_550,500,500,550,550
```

`motor.csv` contains specifications and published rotor inertias; `reducer.csv`
contains the selected model/ratio records and the assumed CS11 envelope.
`motor_curve.csv` contains speed-dependent
transient torque points. `study.json` sets **180 deg/s on J1-J3**, **300 deg/s
on J4-J6**, acceleration corners at **±6 rad/s^2**, and **110 mm J5-J6 spacing**. L2 stays
at 90 mm and link allocation maintains exactly 550 mm extended shoulder-to-TCP
reach. Requirements do not acquire extra motion or hardware columns.

Primary Pmax uses `hold_and_peak_envelope`: continuous holding limits and
speed-dependent motor transient torque, with reducer repeated-peak limits
during motion. `pmax_continuous_screen_kg` retains a separate continuous-only
motor diagnostic. Original continuous motor ratings remain **0.25 Nm for GL40**
and **0.08 Nm for GL30**; raising the operating bus does not raise these ratings.

GL40 uses the official 16 V curve. **GL30 at 16 V is an explicit unqualified
engineering assumption**, made by shifting its official 12 V curve by 1,020 rpm
using published Kv255. At 1,500 rpm the recorded dynamic limits are about
0.4914 Nm for GL40 and 0.2353 Nm for the assumed GL30 curve. Original peak caps
remain 0.75 / 0.28 Nm. Manufacturer overload duration and timed duty are unknown;
`duty_cycle_verified` and `thermal_duty_verified` remain false. These are
conditional instantaneous capacity tests, not an RMS, heating or lifetime pass.

Published motor rotor inertia is included once. Reducer rotating inertia and
input torque limits remain unknown. Transmission is ideal and lossless; no
unpublished efficiency or friction is invented. The requested CS11 envelope
is a modeling input, not verified CS-11-1U-E drawing data. Historical
GL_FIXED_550 snapshots retain the latter's 40 mm square flange, 56.5 mm total
axial shaft envelope and 56.57 mm bounding circle. Sources and all assumptions
are retained in `input_assumptions.json` and `study_assumptions.csv`.

Source records are in
[the lightweight motor survey](../../catalogs/lightweight_motor_survey/README.md),
[gl_torque_curves.json](../../catalogs/lightweight_motor_survey/gl_torque_curves.json)
and [locked_gl_hardware.json](../../catalogs/sitodrive/locked_gl_hardware.json).
Structural mass, CoM and inertia follow the approved generated stock geometry.
Local assembly gates stay active; global travel collisions and mechanical joint
stops remain outside the search.

Runs are stored under `optimization/runs/GL_CD20_550_07`, `_08` and `_09`, with
immutable input snapshots and a complete final `best_model.xml`. Historical
`GL_FIXED_550_*`, `SO_30_550_*` and `REAL_550_*` runs retain their original hardware and policies.
Their scores are not the active GL study's results. To regenerate the earlier
industrial-motor palette deliberately, use
`uv run python -m optimization.real_catalog --study stepperonline`.

See the [workflow](../../README.md) and [CSV column guide](../README.md).

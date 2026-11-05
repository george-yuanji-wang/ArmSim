# Headless arm optimization

The active **GL_CD20_550** study searches configurations **07, 08 and 09
separately**, at **550 mm fully extended reach and exactly 500 g payload**.
Hardware is fixed; the search adjusts approved link dimensions and permitted
local motor placements within each configuration. Each completed search keeps
its own result folder and complete, standalone `best_model.xml`.

| Joints | Fixed motor | Fixed reducer | Ratio |
|---|---|---|---:|
| J1 | CubeMars GL40 KV210 | SITO HD14_50 | 50:1 |
| J2 | CubeMars GL40 KV210 | SITO CD20_50 | 50:1 |
| J3 | CubeMars GL40 KV210 | SITO CD20_30 | 30:1 |
| J4/J5/J6 | CubeMars GL30 KV290 | SITO CS11 series (variant unspecified) | 30:1 |

There is **one hardware tuple per layout**, with joint ratios
**50/50/30/30/30/30** and a common **16 V bus**. The CS11 model uses the
user-specified **43 mm diameter × 32 mm axial envelope and 200 g mass**;
the exact variant is unspecified. Other catalog products are not selectable
in this study. Motion targets are **180 deg/s on J1-J3**, **300 deg/s on
J4-J6**, and acceleration corners at **±6 rad/s^2**.

## Inputs and commands

The [completed GL_CD20_550 comparison](runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/comparison.md)
records these best tested candidates:

| Configuration | Pmax (kg; transient envelope) | J_G (recorded) | Modeled mass (kg) |
|---|---:|---:|---:|
| 07 | 0.91844 | 0.24818 | 3.02541 |
| 08 | 0.90080 | 0.24205 | 3.12416 |
| 09 | 0.90984 | 0.24266 | 3.09285 |

Each completed **200 load evaluations over all 1,008 poses** and met the sampled
**500 g requirement at 550 mm reach** under the conditional transient-envelope
policy. The finite search budget does not prove a global optimum. The separate
continuous-only motor diagnostic is **0.3983 / 0.3938 / 0.3986 kg** for
07 / 08 / 09; the primary pass depends on the recorded transient curve policy
and the explicit GL30 16 V curve estimate.

Complete winners are [`model/best-07-base-6dof.xml`](../model/best-07-base-6dof.xml),
[`model/best-08-base-6dof.xml`](../model/best-08-base-6dof.xml) and
[`model/best-09-base-6dof.xml`](../model/best-09-base-6dof.xml).
[`model/best_model.xml`](../model/best_model.xml) selects 07 by the saved
payload/mass ranking. The published models match their saved scorers; previous
public exports are archived. Historical GL_FIXED_550 scores and models remain
attached to their original inputs; see
[historical comparisons](#historical-comparisons-and-synthetic-examples).

The [independent audit](runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/independent_audit.json)
passed mass-matrix, gravity and velocity-load checks at all 1,008 poses,
550 mm extension checks, and full-rank six-dimensional tool Jacobians at ten
generic poses per winner. In each design, J5's CS11 repeated-peak limit of
4.5 Nm sets the Pmax boundary; adding 1 g fails that boundary check.
[Published-model verification](runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/public_model_verification.json)
passed byte identity, metadata and six-actuator checks, and rendered QA passed.

A [same-geometry 12 V wrist recheck](runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/voltage_sensitivity.csv)
using the official GL30 12 V curve gives Pmax **0.9059 / 0.8855 / 0.8850 kg**
for 07 / 08 / 09. All still meet 500 g on the finite basis. No geometry was
reoptimized for this sensitivity check; transient duty, thermal verification
and ideal transmission assumptions remain unchanged.

The established folders have distinct roles:

```text
catalogs/<supplier>/    Retained manufacturer facts, drawings and curves
data/                  General editable CSV templates
data/real/             Generated active real-hardware inputs
examples/synthetic/    Explicitly invented demonstration inputs
runs/<project>/        Immutable timestamped results
```

From the repository root, regenerate the fixed-hardware inputs and run all
three independent searches:

```powershell
uv run python -m optimization.real_catalog --study locked-gl
uv run python run_optimization.py --real --cycles 200 --checkpoint-every 20
```

Add `--layout 7` to select 07 alone. The current full run budgets **200 distinct
load evaluations per configuration**, with checkpoints every **20** evaluations.
Cheap rejected geometries and duplicate physical candidates do not consume a
load evaluation. A reduced seed grid finding no valid assembly is recorded as
a seed failure, not proof that every possible geometry is impossible.

The generator archives previous files in `data/real/` before replacing them.
The general `data/` templates and manufacturer research are not overwritten.
Each run then snapshots its exact inputs, source hashes, assumptions and settings.

The generated files are:

- [motor.csv](data/real/motor.csv): two motor models with published masses,
  dimensions, continuous ratings and rotor inertias.
- [reducer.csv](data/real/reducer.csv): four reducer model/ratio records,
  including the explicitly assumed CS11 series envelope.
- [motor_curve.csv](data/real/motor_curve.csv): speed-dependent transient
  torque points and operating-voltage evidence.
- [requirement.csv](data/real/requirement.csv): project goals only.
- [study.json](data/real/study.json): motion limits, hardware assignments and
  the `hold_and_peak_envelope` motor policy.
- `input_assumptions.json` and `study_assumptions.csv`: source provenance,
  unknowns and explicit simulation assumptions.

`requirement.csv` retains its five columns:

```csv
project_id,payload_lower_g,payload_upper_g,reach_lower_mm,reach_upper_mm
GL_CD20_550,500,500,550,550
```

Each row is an independent project. Bounds may be equal. Motion and hardware
policies stay outside this file. For manual catalogs:

```powershell
uv run python -m optimization validate --data-dir optimization/data/real
uv run python -m optimization prepare --data-dir optimization/data/real --project GL_CD20_550
```

See the [CSV column guide](data/README.md) for general input fields. To restore
the earlier STEPPERONLINE input palette deliberately, use:

```powershell
uv run python -m optimization.real_catalog --study stepperonline
```

This changes the active generated inputs while preserving their archive and
all previous runs. `--study locked-gl` restores the fixed GL palette.

## Torque policy and operating assumptions

Primary **Pmax is conditional transient-envelope payload capacity** over the
finite test basis. Static gravity holding uses motor continuous and reducer
rated torque. During motion, motor capacity is interpolated from its recorded
torque-speed curve and capped by its published peak torque. Reducer motion
uses its **repeated-peak** rating; emergency/momentary ratings are not used.

GL40 KV210 uses the official **16 V** curve. GL30 KV290 has an official **12 V**
curve; operation at **16 V is an engineering assumption**, not manufacturer
qualification. Its curve is shifted by `Kv255 * (16 - 12) = 1020 rpm` and retains
the original **0.08 Nm continuous / 0.28 Nm peak** limits. GL40 retains
**0.25 Nm continuous / 0.75 Nm peak**. At 1,500 rpm, the digitized GL40 curve
allows about **0.4914 Nm**; the assumed GL30 16 V curve gives about **0.2353 Nm**.
These are transient envelope points, not new continuous ratings.

The diagnostic `pmax_continuous_screen_kg` applies the original continuous motor
limits throughout the same motion basis. It is recorded separately from primary
Pmax. `pmax_known_constraints_kg` omits the dynamic motor transient constraint
and is an additional diagnostic, not the selected motor-qualified capacity.

Manufacturer peak duration and a timed duty cycle are unavailable. Outputs
explicitly record `duty_cycle_verified = false` and
`thermal_duty_verified = false`. No RMS heating, cycle-average reducer load,
bearing life or complete actuator validation is inferred from these untimed
stress tests. Transmission is ideal and lossless, with 1:1 local input belts;
unpublished efficiency, friction and input-shaft torque limits remain unknown.

Published rotor inertias are **7.4e-6 kg m^2 for GL40** and **2.42e-6 kg m^2 for
GL30**, included once in motor acceleration demand. The previous whole-motor
`M * R^2` upper bounds are not used for these motors. Reducer rotating inertias
remain unassigned because the inspected source does not establish the shaft
reference. Retained source facts are in
[the lightweight motor survey](catalogs/lightweight_motor_survey/README.md),
[gl_torque_curves.json](catalogs/lightweight_motor_survey/gl_torque_curves.json)
and [locked_gl_hardware.json](catalogs/sitodrive/locked_gl_hardware.json).

## Reach, geometry and design variables

**Reach is maximum fully extended shoulder-to-TCP distance**, measured from J2
and excluding pedestal height. The 550 mm requirement is a length constraint,
not a Cartesian target shell. For 07-09, L2 remains the fixed **90 mm J3/J4
coupling**, perpendicular to the J4 axle. J5-J6 spacing is **110 mm**, and the
tool offset is **80 mm**:

```text
R = L1 + sqrt(90^2 + L3^2) + 190       (mm)
L3 = sqrt((360 - L1)^2 - 90^2)        when R = 550 mm
```

Fixed reach therefore leaves **one independent link-length allocation**.
L2 is never removed or optimized independently. Named extension poses verify
the radius through MuJoCo forward kinematics. Original base scenes retain their
70 mm wrist; earlier runs retain their own geometry and input snapshots.

Motor placement freedom follows the selected layout: coaxial motors are fixed;
offset J2 motors have two bounded coplanar coordinates; other offset motors
follow their existing mounting lines. The search does not introduce remote
motors, crossed-joint belts, extra reducers or replacement linkage topologies.

Scoring and export share `approved-base-template-v8` in `model_factory.py`.
The approved edge frames, mid-height case attachments, moving output surrounds
and input drives are rebuilt from each candidate. Local mounting and own-joint
assembly checks remain active. The 07 case-port inset adapts to smaller motor
cases while preserving its connected bracket, and the bounded seed grid
searches for a fitting starting geometry. Unsupported assemblies are rejected.

The active **CS11 series 30:1** model uses the user-specified **43 mm diameter,
32 mm axial envelope and 200 g mass**. The exact product variant remains
unspecified, so these dimensions are an explicit modeling assumption rather
than verified CS-11-1U-E drawing facts. Historical GL_FIXED_550 snapshots retain
their conservative CS-11-1U-E geometry: a 40 mm square flange, 39 mm round body,
56.5 mm total axial shaft envelope and 56.57 mm bounding circle.

Structural mass, full 3D CoM and inertia follow the generated aluminium stock
at **2700 kg/m^3**, rather than independent mass/CoM variables. Motor/reducer
mass is separate; small transmission pieces retain explicit base estimates.
Longer links therefore carry their added mass and inertia. The legacy
`data/link_bounds.csv` is parsed and snapshotted for compatibility, but its old
mass/CoM budgets do not determine structural properties.

## Simulation basis and selection

Each candidate is tested at **1,008 shared joint states**:

- 24 named references: zero, signed single-axis quarter turns, six fully extended
  world directions and combined folded/wrist states.
- 984 deterministic six-dimensional Halton samples in `[-pi, pi)`.

MuJoCo supplies gravity, the mass matrix and 21 quadratic velocity coefficients
for the empty arm and unit payload. At each pose, evaluation combines **37
velocity states** with all **64 acceleration-box corners**. The active six-joint
speed vector is `180/180/180/300/300/300 deg/s` and acceleration is `6 rad/s^2`.
This finite velocity basis is not every possible simultaneous joint velocity.
Reference definitions are saved in `reference_poses.csv`; resolved poses and
loads are saved in `pose_results.csv`.

Selection maximizes zero-to-Pmax capacity over this finite basis, subject to
reach and local assembly validity. If Pmax is zero, ranking first minimizes
worst required-payload utilization, then uses known-constraint capacity.
Lower mass breaks remaining ties. Scores are compared at nine decimal places.
**J_G is recorded, not optimized**: it averages squared gravity torques
normalized by reducer rated limits at the required payload. Raw joint gravity
loads and the separate capacity normalization accompany comparisons because
changing reducer ratings changes the normalized score.

The 200-evaluation geometry search is finite and does not prove a global optimum.
Global travel collisions and mechanical joint stops remain disabled as requested;
local mounting constraints remain enforced. No elastic stiffness or precision
score is included. Payload inertia is a TCP-centered 10 mm sphere.

## Results and retention

Every invocation creates a unique folder. Completed output is organized as:

```text
optimization/runs/
  GL_CD20_550_07/               Independent project; _08 and _09 are separate
    <UTC timestamp + suffix>/
      run.json                  Settings, versions and hashes
      inputs/                   Exact input snapshots
      evaluated_inputs.json     Parsed inputs used by this process
      reference_poses.csv       Shared reference definitions
      hardware_coverage.*       Fixed tuple and actual evaluation coverage
      hallmarks.csv             Best-so-far saved milestones
      checkpoints/              Design and RNG state
      RESULT.md                 Readable final result
      result.json / result.csv  Scores, assumptions and diagnostics
      best_design.json          Lengths, hardware and placements
      best_links.csv            Mass, local XYZ CoM and full inertia
      best_joints.csv           Hardware and component placements
      best_model.xml            Complete standalone viewer-ready MJCF
      pose_results.csv          Final tested poses and loads
      basis_coefficients.csv    Final empty/unit-payload basis
```

With 200 completed evaluations and checkpoint interval 20, the ordinary saved
milestones are cycle 1 and every 20 evaluations. Detailed basis data are stored
once for the final winner. Failed or exhausted runs retain their inputs and
checkpoints with the failure reason; no result folder is erased automatically.
The comparison folder under `runs/comparisons/GL_CD20_550/` is produced when all
three independent projects complete. Its scores must be read with the transient
and operating-voltage assumptions above.

Open or re-export a completed current-contract result:

```powershell
uv run python view_models.py --file optimization/runs/GL_CD20_550_07/RUN_FOLDER/best_model.xml
uv run python -m optimization export --run optimization/runs/GL_CD20_550_07/RUN_FOLDER
```

Re-export appends a new export. An explicit `--output model/best-07.xml --replace`
archives the existing output first. Older geometry contracts require a new
search; old scores cannot be transferred onto newly generated models. The
conceptual stock model does not establish stiffness or manufacturing readiness.

## Historical comparisons and synthetic examples

The [GL_FIXED_550 comparison](runs/comparisons/GL_FIXED_550/20261105T083535_687425Z_21e4c5fd/comparison.md)
used GL40/GL30 motors, CD-17 50:1 reducers on both J2 and J3, and the larger
CS-11-1U-E drawing envelope. These are **historical results**, not results for
the corrected GL_CD20_550 hardware:

| Configuration | Historical Pmax (kg; transient envelope) | Historical J_G | Historical modeled mass (kg) |
|---|---:|---:|---:|
| 07 | 0.93154 | 0.29953 | 2.70595 |
| 08 | 0.91863 | 0.28183 | 2.80475 |
| 09 | 0.90627 | 0.27833 | 2.77339 |

Each historical search completed 200 evaluations on all 1,008 poses and met
the 500 g sampled requirement at 550 mm reach. Its saved-basis/dynamics audit
passed, including six-dimensional tool Jacobians. The [same-geometry 12 V wrist recheck](runs/comparisons/GL_FIXED_550/20261105T083535_687425Z_21e4c5fd/voltage_sensitivity.csv)
gave Pmax 0.9120 / 0.8997 / 0.9006 kg. It is a voltage sensitivity check on
historical geometry, not another optimization. Both voltage cases remain
conditional on transient duty and ideal transmissions.

The [November 4 SO_30_550 comparison](runs/comparisons/SO_30_550/20261104T185825_693954Z_e63aa827/comparison.md)
is retained as **historical**, with all original inputs and models. It used
eight STEPPERONLINE motor models, 30:1 reducers, 30 hardware tuples and 600
evaluations per layout under a continuous-motor dynamic screen:

| Layout | Historical continuous-screen Pmax (kg) | Historical known-constraint capacity (kg) | Historical J_G |
|---|---:|---:|---:|
| 07 | 0 | 0.305894 | 0.580434 |
| 08 | 0 | 0.557028 | 0.484303 |
| 09 | 0 | 0.600857 | 0.471606 |

Those figures are not results of GL_CD20_550. Its selected 07 export was chosen
by the zero-Pmax fallback ranking; known-constraint capacity was a diagnostic,
not an independently maximized objective. The [earlier 200-evaluation SO comparison](runs/comparisons/SO_30_550/20261104T083539_503457Z_873144a7/comparison.md)
used a different 19-motor selection and is historical as well.

General editable templates in `data/` retain their `EXAMPLE` requirement row.
The synthetic example uses invented motor/reducer fixtures and a 550 mm reach
with 500-1,000 g payload:

```powershell
uv run python run_optimization.py --demo
uv run python run_optimization.py --demo --layout 7
```

The default creates separate `SYNTHETIC_DEMO_07`, `_08` and `_09` projects.
For a quick wiring check, use `--cycles 3 --pose-limit 24`; reduced pose runs
are marked and must not be compared with full-suite results. These commands
do not open or modify the GUI.

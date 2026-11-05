# Manufacturer motor and reducer research

Retained source facts belong in the supplier/research folders below. Generated
simulation inputs belong in [`../data/real/`](../data/real/README.md); general
editable templates remain in `../data/`. Updating an active selection does not
rewrite the source surveys or previous optimization results.

The active **GL_CD20_550** palette contains only:

| Joints | Motor | Reducer | Ratio |
|---|---|---|---:|
| J1 | CubeMars GL40 KV210 | SITO HD14_50 | 50:1 |
| J2 | CubeMars GL40 KV210 | SITO CD20_50 | 50:1 |
| J3 | CubeMars GL40 KV210 | SITO CD20_30 | 30:1 |
| J4/J5/J6 | CubeMars GL30 KV290 | SITO CS11 series (variant unspecified) | 30:1 |

There is one fixed hardware tuple per configuration, with 550 mm extended
reach and 500 g payload. Other research records are retained but are not
selectable in this study. Ratios are **50/50/30/30/30/30**. The common operating
bus is **16 V**, with the GL30 curve estimate described below.

The [completed GL_CD20_550 comparison](../runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/comparison.md)
records 200 evaluations per configuration over all 1,008 poses. All three meet
the sampled 500 g requirement at 550 mm under the conditional transient-envelope
policy and the GL30 16 V estimate. These finite-budget results do not prove a
global optimum; supplier facts and study assumptions remain distinct below.
The [independent audit](../runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/independent_audit.json)
passed. The [official-12 V wrist sensitivity recheck](../runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/voltage_sensitivity.csv)
also meets the sampled requirement for all three saved geometries, without
reoptimization or thermal-duty verification.

## Current GL source records

- [lightweight_motor_survey/README.md](lightweight_motor_survey/README.md): the
  45-record lightweight motor survey, including source conditions, unknowns
  and the filterable workbook. These are research candidates, not 45 qualified
  actuators.
- [smc_cubemars.json](lightweight_motor_survey/smc_cubemars.json): original
  CubeMars/SMC manufacturer facts, including GL30/GL40 mass, dimensions,
  electrical ratings and published rotor inertias.
- [gl_torque_curves.json](lightweight_motor_survey/gl_torque_curves.json):
  digitized official GL30 12 V and GL40 KV210 16 V characteristics, retained
  source-image hashes and operating-voltage assumptions. Original images are
  under `lightweight_motor_survey/source_curves/`.
- [locked_gl_hardware.json](sitodrive/locked_gl_hardware.json): selected
  reducer model/ratio records, joint assignments, torque definitions, retained
  drawing facts and the explicit CS11 model envelope assumption. Original PDFs
  are under `sitodrive/reducer_sources/`.

GL30 operation at **16 V is a study assumption**, derived from its official
12 V curve by a 1,020 rpm shift using Kv255. It is not manufacturer-qualified
16 V operation or a new continuous rating. GL30/GL40 retain their original
0.08/0.25 Nm continuous ratings and 0.28/0.75 Nm peak caps. Curve points describe
instantaneous torque-speed screening; permissible overload duration remains
unknown. Published rotor inertia is used rather than a whole-motor mass/radius
bound. Unknown source values remain null or blank.

The active CS11 series reducer uses the user-specified **43 mm diameter,
32 mm axial envelope and 200 g mass**. Its exact variant is unspecified;
these dimensions are model inputs rather than verified CS-11-1U-E drawing
facts. The historical GL_FIXED_550 study instead used the CS-11-1U-E drawing's
**40 mm square flange, 39 mm round body and 56.5 mm total axial shaft envelope**,
with a conservative 56.57 mm circle enclosing the flange corners. Its retained
source record remains evidence for that exact variant. Efficiency, backdrive
torque, independent reducer input-torque limits and bearing load ratings are
not invented where absent.

## Broader retained motor tables

- [motors.csv](motors.csv) and [motors.json](motors.json): combined legacy
  manufacturer records with source URLs, units and rating definitions.
- [motors_optimizer_ready.csv](motors_optimizer_ready.csv): the original
  complete subset for the legacy linear torque-speed policy; it is not the
  active GL palette.
- [catalog_audit.json](catalog_audit.json): source hashes, size checks and
  missing-field audits for those combined tables.
- [bldc/](bldc/README.md), [maxon/](maxon/README.md) and
  [stepperonline/](stepperonline/README.md): supplier-specific records,
  snapshots and supplements. Later standalone surveys can contain records
  not included in the older combined tables.

The former STEPPERONLINE screen and its reasons remain in
[motor_selection.json](stepperonline/motor_selection.json). Its eight-model,
30-tuple palette and the earlier nineteen-model study are historical; neither
is silently mixed into the fixed GL study.

Dimensions retain their physical meaning. Across-flats size of a square flange
is not a cylindrical diameter; body length and full shaft envelope are separate.
Frameless motor mass excludes custom bearings/housing, whereas assembled motor
mass retains the manufacturer's stated scope. Rated torque/current depend on
the stated mounting and thermal conditions. Rated shaft watts are not treated
as measured electrical consumption.

Blank numeric cells mean **not established by the source**, not zero. Peak and
stall torque are kept separate; no-load speed is not a mechanical speed limit.
Torque measured at one speed/voltage is not combined with a separate unloaded
speed as if they were one rated point. Motor peak duration stays unknown where
the manufacturer does not provide it.

## Broader SITO reducer inventory

The retained supplier inventory separates model/ratio records and definitions:

- [reducer_models.csv](sitodrive/reducer_models.csv): public-store model inventory.
- [reducer_variants.csv](sitodrive/reducer_variants.csv): one row per model/ratio.
- [coverage.json](sitodrive/coverage.json): coverage counts and unresolved fields.
- [source_conflicts.csv](sitodrive/source_conflicts.csv): differing official values.
- [catalog_only.csv](sitodrive/catalog_only.csv): catalog-only entries.
- [reducer_selection.json](sitodrive/reducer_selection.json): retained **historical
  30:1 selection** for the STEPPERONLINE study.
- [locked_gl_hardware.json](sitodrive/locked_gl_hardware.json): the active fixed GL
  reducer palette.

Torque at 2,000 rpm input, average load, repeated peak and momentary emergency
peak remain separate fields. Dividing a published output rating by ratio does
not create a manufacturer-specified maximum input-torque rating. An inertia
value without a verified shaft reference is retained as source data and is not
automatically reflected through the gear ratio.

## Generate inputs and retain results

From the repository root:

```powershell
uv run python -m optimization.real_catalog --study locked-gl
uv run python run_optimization.py --real --cycles 200 --checkpoint-every 20
```

The adapter archives previous `data/real/` inputs before writing the selected
two motors, four reducer model/ratio records, five-column requirements, motor curves and study
assumptions. Each optimization run snapshots those exact files into its own
immutable timestamped folder. Historical `GL_FIXED_550_*`, `REAL_550_*` and `SO_30_550_*` results
remain attached to their original inputs and rating policies.

To regenerate the earlier industrial palette deliberately:

```powershell
uv run python -m optimization.real_catalog --study stepperonline
```

The older combined motor tables can still be rebuilt from captured sources:

```powershell
uv run python -m optimization.catalogs.build_catalogs
```

Rebuilding derived tables does not refetch supplier pages. Refresh a source
collector only when a deliberate new retrieval is wanted. Synthetic hardware
stays confined to the synthetic example folder.

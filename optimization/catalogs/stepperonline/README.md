# Complete STEPPERONLINE BLDC motor research

The active **SO_30_550** inputs now import **eight manually screened complete
motor models under USD 50 each**. J4/J5/J6 use the same fixed **42BSB41-24-01**
(0.063 Nm, 3,000 rpm, 41 mm body, 262 g each). J1-J3 have 2 / 3 / 5 allowed
choices, giving **30 hardware tuples per configuration**. The current comparison
allocates 600 evaluations per configuration, 20 to each tuple. The intended
study remains configurations 07, 08 and 09 at **500 g payload and 550 mm reach**.
The three reducer models and their joint assignments
are fixed: HD-14 on J1, CD-20 on J2/J3, and CS-11-2XH-A on J4/J5/J6, all 30:1.
The active speed limits are 180 deg/s on J1-J3 and 300 deg/s on J4-J6, with
6 rad/s^2 acceleration. Run it from the repository root:

```powershell
uv run python run_optimization.py --real --cycles 600
```

Captured **2026-11-04** from STEPPERONLINE's public product pages. The catalog
contains **28 voltage-specific models** of assembled motors with housing, shaft
and bearings. Drivers, position encoders and reducers are separate purchases.

`motors.json` contains normalized specifications and provenance. Each row retains
its exact model, voltage, rated torque/speed/current, price, net motor mass,
body and shaft dimensions, source URL and source snapshot hash. Published Hall
sensors provide commutation feedback; they are not an included position encoder.

The current study filter accepts positive unit prices up to USD 50, separately
published 24 V or 36 V models, known net mass, and body length strictly below
100 mm. It excludes products with an explicitly published minimum order above
one motor. This leaves **21 candidates**, including **seven 36 V models**. Four
48 V variants remain in the research catalog for reference. Preorder/backorder
status is retained and is not a claim of immediate stock availability.

That is the retained source-eligibility survey, not the current selectable set.
The manual screen in [motor_selection.json](motor_selection.json) records a
decision and reason for every one of the 28 source rows. No source records or
previous results are deleted.

## Manual selectable set

| Model | Torque (Nm) | Rated rpm | Rated power (W) | Body length (mm) | Net mass (g) | Role |
|---|---:|---:|---:|---:|---:|---|
| 42BSB41-24-01 | 0.063 | 3000 | 20 | 41 | 262 | Fixed shared wrist |
| 42BSA62-36-01 | 0.16 | 3000 | 50 | 62 | 430 | J3, lighter 0.16 package |
| 57BYA54-24-01 | 0.16 | 3000 | 50 | 53.5 | 470 | J3, shorter 3000 rpm package |
| 57BLY54-24-01 | 0.16 | 4000 | 67 | 53.5 | 450 | J3, same length and 20 g lighter |
| 57BLR50-24-01 | 0.23 | 3500 | 84 | 49.1 | 500 | J1/J3 |
| 57BLR70-24-02 | 0.47 | 3500 | 172 | 74 | 650 | J1/J2/J3 |
| 57BYA94-36-01 | 0.45 | 3000 | 141 | 93.5 | 1030 | J2, 5.4 A listed current |
| 57BSD96-36-01 | 0.45 | 3000 | 141 | 96 | 810 | J2, 5.4 A listed current |

The 36 V J2 models remain candidates despite their extra length/mass because
their listed rated current is 5.4 A, versus 10.4 A for BLR70-24. Current
conventions are unpublished; these are not verified phase-RMS values or a
motor-heating comparison. No current cap, supply bus or efficiency is invented.

The two 0.16 Nm short packages illustrate a real tradeoff: the 3000 rpm BYA54
has a lower rated-power label but is 20 g heavier than the 4000 rpm BLY54.
Both exceed the required 900 / 1500 motor rpm. Rated mechanical power follows
torque times rated angular speed; it does not establish electrical consumption
at the arm's actual operating point. No efficiency advantage is invented.
The 24 V and 36 V BSA62 windings have identical modeled mass, geometry, torque
and rated speed; the 36 V row represents that mechanical choice, while the
24 V source record remains available.

The .08 Nm BSA46/BYA51 packages remain documented step-up alternatives rather
than active shared-wrist choices. The 450-470 g .16 Nm motors are not assigned
to the wrist. The 262 g BSB41 is the compact, verified common baseline; its
fast-motion capacity still needs evaluation with the corrected load policy.
See [compact_wrist_followup.json](compact_wrist_followup.json) for the lighter
product follow-up: BLR43 has MOQ 30, BLS40 lacks verified net mass, and the
36 mm BLR65 is discontinued. Shipping weights were not substituted.

## Source limitations

The 36 V models have their own published ratings. In particular, 57BLR50-36-01
is 0.11 Nm at 4,000 rpm, while 57BLR50-24-01 is 0.23 Nm at 3,500 rpm. Applying
36 V to the 24 V model is not how these catalog rows were produced.

Unknown stall torque, peak torque/duration, mechanical maximum speed and rotor
inertia remain `null`. None of the captured product-page specification sections
publishes rotor inertia. In the active CSV, the published inertia field remains
blank and a separate `rotor_inertia_upper_bound_kg_m2` field stores
`M_motor * R_envelope^2`. The simulation uses that conservative physical bound
with the `mass_radius_upper_bound` label; it is not a manufacturer measurement.
The research catalog retains `null` and never silently assigns zero.
Rated current conventions are preserved as unspecified
where the source does not distinguish DC, phase RMS or phase peak current.

`derived_circumscribed_radius_mm` is a geometry conversion, separate from the
published frame dimensions. A 57 x 57 mm square motor needs a 40.31 mm circular
envelope radius; its radius is not 28.5 mm. Body lengths exclude shafts.
`derived_named_axial_span_lower_bound_mm` adds the named shaft extensions and
is a lower bound, rather than a certified complete drawing envelope.

Known source issues remain visible:

- 57BLR70-24-02: current body length is 74 mm despite the old 69 mm URL slug.
  Physical dimensions describe a round motor, while a generic category field
  incorrectly describes a square NEMA23 frame.
- 57BLR50-24-01: the current page states 49.1 mm and 0.50 kg; older translated
  listings used approximately 50 mm and 0.52 kg. Verify the supplied revision.
- 42BYA73-24-01: the current page states 0.42 kg; an earlier indexed drawing
  stated 0.40 kg. The current larger page value is retained.
- 42BLS40-24-01 and 42BLS60-24-01: the captured page does not state net motor
  mass. Gross shipping mass is not substituted; both need drawing review before
  import under the current filter.
- 42BLR43-24-01: the current main product page specifies preorder with MOQ 30.
  Related-product tables can show a different stock summary.

Retained sources are under `source_snapshots/20261104/`. They are rendered text
captures returned by the web retrieval tool, not claimed raw HTML. Direct
`urllib` requests returned HTTP 403 on the retrieval date.

Rebuild normalized JSON and the audit without fetching or changing sources:

```powershell
uv run python -m optimization.catalogs.stepperonline.build_catalog
```

This supplier builder changes only this research folder. The combined research
tables are rebuilt with `python -m optimization.catalogs.build_catalogs`.
To regenerate real-study CSVs under `optimization/data/real/` from this motor
research and `../sitodrive/reducer_selection.json`, archiving previous inputs:

```powershell
uv run python -m optimization.real_catalog
```

The optimizer retains the approved mounting topology and local assembly gates.
Its J5-J6 wrist span is 110 mm for this complete-motor study, while L2 remains
90 mm. Saved earlier models and scores retain their original geometry.
The `--real` launcher reads this current study from `optimization/data/real/`.
General input templates stay in `optimization/data/`; SITO-specific selected
facts and drawing files stay in `optimization/catalogs/sitodrive/`.

Prices are motor-only China listing prices before freight/tax. The catalog is a
source inventory; accepting a row does not establish that it fits an arm layout
or passes a load test.

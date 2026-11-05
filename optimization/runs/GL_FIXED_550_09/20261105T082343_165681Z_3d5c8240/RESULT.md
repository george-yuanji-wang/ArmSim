# GL_FIXED_550_09

Conditional transient-envelope sampled requirements: **MET**.

- Reference poses: 1008
- Distinct load evaluations: 200/200 requested
- Proposal attempts: 201
- Search termination: evaluation_budget_reached
- Required payload: 500–500 g
- Required fully extended shoulder-to-TCP reach: 550–550 mm
- Fixed wrist2 span: 110 mm
- Fully extended reach: 550 mm
- Reachable targets: 1008/1008
- Conditional transient-envelope Pmax: 0.906271 kg
- Continuous-only screen: 0.403234 kg
- Motor rating policy: hold_and_peak_envelope
- Conditional known-constraint capacity: 0.906271 kg
- Reducer rating policy: hold_and_repeated_peak
- Hardware validation status: conditional_transient_envelope
- Arm mass: 2.77339 kg
- Gravity loading score J_G: 0.278327
- Capacity-normalized gravity score: 0.278327

Primary Pmax checks continuous holding and speed-dependent motor transient curves, plus reducer holding/repeated-peak limits. Motor pulse durations and timed thermal duty remain unverified. Any voltage-extrapolated motor curve is an explicit assumption, not a manufacturer-qualified operating point. The conditional known-constraint capacity checks continuous holding ratings and reducer repeated peaks but omits unverified motor transient limits. It is not a certified payload: motor pulse capability and cycle-average thermal/lifetime limits remain unverified.

Read `pose_results.csv` for per-pose loads, `best_links.csv` for named structural mass properties, and `best_joints.csv` for selected hardware and local placements. `basis_coefficients.csv` stores the final empty/unit-payload load basis. Input CSVs are copied under `inputs/`.

Scoring and export use the same parameterized approved base geometry. Unsupported local case mounts, hardware envelopes, link spans and own-joint assembly interference are rejected before load simulation. The exported model includes edge frames, mounts, flanges, belts, sites, sensors, catalog-limited servos and reflected rotor inertia. Structural mass, 3D COM and inertia are calculated from the approved base stock geometry; they are not free optimization variables. The study uses unlimited joints and no global self-collision checks. Precision, elastic stiffness, peak-duration thermal behavior and task-specific forward validation are outside this run. The finite velocity probes do not certify every possible motion.

## Motor tuple coverage

Attempted 1/1 motor tuples; load-evaluated 1/1. Categorical coverage complete: True. Each tuple has its own bounded geometry search. Continuous global optimality is not proven.

See [hardware coverage](hardware_coverage.csv) and [detailed coverage](hardware_coverage.json).

# REAL_550_08

Requirements: **MET**.

- Reference poses: 1008
- Distinct load evaluations: 200/200 requested
- Proposal attempts: 207
- Search termination: evaluation_budget_reached
- Required payload: 500–1000 g
- Required fully extended shoulder-to-TCP reach: 550–550 mm
- Fully extended reach: 550 mm
- Reachable targets: 1008/1008
- Sampled-basis Pmax: 6.59602 kg
- Arm mass: 6.01055 kg
- Gravity score J_G: 0.0256252

Read `pose_results.csv` for per-pose loads, `best_links.csv` for named structural mass properties, and `best_joints.csv` for selected hardware and local placements. `basis_coefficients.csv` stores the final empty/unit-payload load basis. Input CSVs are copied under `inputs/`.

Scoring and export use the same parameterized approved base geometry. Unsupported local case mounts, hardware envelopes, link spans and own-joint assembly interference are rejected before load simulation. The exported model includes edge frames, mounts, flanges, belts, sites, sensors, catalog-limited servos and reflected rotor inertia. Structural mass, 3D COM and inertia are calculated from the approved base stock geometry; they are not free optimization variables. The study uses unlimited joints and no global self-collision checks. Precision, elastic stiffness, peak-duration thermal behavior and task-specific forward validation are outside this run. The finite velocity probes do not certify every possible motion.

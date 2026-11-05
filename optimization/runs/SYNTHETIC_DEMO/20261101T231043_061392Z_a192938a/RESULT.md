# SYNTHETIC_DEMO

Requirements: **MET**.

- Reference poses: 1008
- Candidate evaluations: 200
- Required payload: 500–1000 g
- Required shoulder-to-TCP reach: 450–600 mm
- Reachable targets: 1008/1008
- Sampled-basis Pmax: 12.6937 kg
- Arm mass: 7.97521 kg
- Gravity score J_G: 0.0364259

Read `pose_results.csv` for per-pose loads, `best_links.csv` for named structural mass properties, and `best_joints.csv` for selected hardware and local placements. `basis_coefficients.csv` stores the final empty/unit-payload load basis. Input CSVs are copied under `inputs/`.

This is an ideal rigid-body conceptual study with unlimited joints and no collision checks. Continuous torque limits are used conservatively; precision, elastic stiffness, peak-duration thermal behavior and forward-controller validation are outside this run. The finite velocity probes do not certify every possible motion.

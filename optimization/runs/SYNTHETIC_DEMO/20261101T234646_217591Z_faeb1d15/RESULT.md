# SYNTHETIC_DEMO

Requirements: **MET**.

- Reference poses: 1008
- Candidate evaluations: 1
- Required payload: 500–1000 g
- Required shoulder-to-TCP reach: 450–600 mm
- Reachable targets: 1008/1008
- Sampled-basis Pmax: 9.22599 kg
- Arm mass: 10.515 kg
- Gravity score J_G: 0.0862668

Read `pose_results.csv` for per-pose loads, `best_links.csv` for named structural mass properties, and `best_joints.csv` for selected hardware and local placements. `basis_coefficients.csv` stores the final empty/unit-payload load basis. Input CSVs are copied under `inputs/`.

The exported model includes edge frames, mounts, flanges, belts, sites, sensors, catalog-limited servos and reflected rotor inertia. Its explicit optimized rigid-body mass properties remain authoritative; the frame geometry depicts a conceptual structure. The study uses unlimited joints and no collision checks. Precision, elastic stiffness, peak-duration thermal behavior and task-specific forward validation are outside this run. The finite velocity probes do not certify every possible motion.

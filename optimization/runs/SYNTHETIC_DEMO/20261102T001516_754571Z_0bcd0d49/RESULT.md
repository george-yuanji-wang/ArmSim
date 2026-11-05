# SYNTHETIC_DEMO

Requirements: **MET**.

- Reference poses: 1008
- Candidate evaluations: 200
- Required payload: 500–1000 g
- Required shoulder-to-TCP reach: 450–600 mm
- Reachable targets: 1008/1008
- Sampled-basis Pmax: 4.16759 kg
- Arm mass: 4.73673 kg
- Gravity score J_G: 0.132006

Read `pose_results.csv` for per-pose loads, `best_links.csv` for named structural mass properties, and `best_joints.csv` for selected hardware and local placements. `basis_coefficients.csv` stores the final empty/unit-payload load basis. Input CSVs are copied under `inputs/`.

Scoring and export use the same parameterized approved base geometry. Unsupported local case mounts, hardware envelopes, link spans and own-joint assembly interference are rejected before load simulation. The exported model includes edge frames, mounts, flanges, belts, sites, sensors, catalog-limited servos and reflected rotor inertia. Its explicit optimized rigid-body mass properties remain authoritative; the frame geometry depicts a conceptual structure. The study uses unlimited joints and no global self-collision checks. Precision, elastic stiffness, peak-duration thermal behavior and task-specific forward validation are outside this run. The finite velocity probes do not certify every possible motion.

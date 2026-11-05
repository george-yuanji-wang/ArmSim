# Independent configuration comparison

Completed verification:

- [Independent saved-dynamics and capacity audit](independent_audit.json): passed for all three winners, including exact 550 mm extension and full-rank six-axis tool Jacobians at generic poses.
- [Published model verification](public_model_verification.json): the GUI-ready models are byte-identical to their scored winners, with six axes, six actuators and validated drivetrain metadata.
- [Same-geometry wrist voltage sensitivity](voltage_sensitivity.csv): at the published GL30 12 V curve, Pmax is 0.9120 / 0.8997 / 0.9006 kg for 07 / 08 / 09. All still meet 500 g. These are rechecks, not independent 12 V optimizations.
- [Public model and alias manifest](export_manifest.json): `model/best_model.xml` selects 07; previous public exports were archived.

The test suite completed 183 tests: 182 passed, one skipped. Primary capacities below are conditional transient-envelope results. Motor heating/duty, reducer lifetime, transmission losses and manufacturer qualification of GL30 at 16 V are not established by these tests. The explicit `motor_load_policy` and `summary.score_definition` govern the scores; legacy objective prose in the immutable run metadata still uses the word conservative.

Required fully extended shoulder-to-TCP reach: **550–550 mm**. Required payload: **500–500 g**.

Each configuration was searched independently with 200 allocated evaluations with the same 1008-state basis, seed 42, joint speeds J1–J6 [3.1416, 3.1416, 3.1416, 5.236, 5.236, 5.236] rad/s and acceleration 6 rad/s².

Common J5-to-J6 spacing: 110 mm; short L2 coupling: 90 mm.

Reducer rating policy: **hold_and_repeated_peak**. Pmax uses continuous holding and speed-dependent motor transient envelopes; unverified pulse duration, thermal duty and extrapolated voltage curves remain explicit assumptions. Conditional known-constraint capacity omits unverified motor transient limits; it does not establish a certified payload or duty/lifetime pass.

These are the best tested candidates under the recorded ranking: higher selected-policy Pmax; when Pmax rounds to zero, lower worst required-payload load-to-rating ratio, then higher conditional known-constraint capacity; lower arm mass breaks remaining ties. Scores are compared at nine decimal places. If every Pmax is zero, selection therefore minimizes the worst overload at the required payload. Conditional capacities are measurements of the selected designs, not independently maximized payloads, and may decrease as overload improves. J_G is retained for comparison rather than optimized.

| Configuration | Evaluated / allocated | Extension (mm) | L1 / L2 / L3 (mm) | Structural L1 / L2 / L3 (g) | Arm mass (kg) | Transient-envelope Pmax (kg) | Conditional capacity (kg) | J_G | Sampled requirements met |
|---|---:|---:|---|---|---:|---:|---:|---:|---|
| 07 | 200 / 200 | 550.0 | 247.1 / 90.0 / 68.1 | 87.4 / 74.1 / 57.1 | 2.706 | 0.9315 | 0.9315 | 0.29953 | Yes |
| 08 | 200 / 200 | 550.0 | 205.0 / 90.0 / 126.2 | 115.0 / 74.1 / 96.8 | 2.805 | 0.9186 | 0.9186 | 0.28183 | Yes |
| 09 | 200 / 200 | 550.0 | 195.0 / 90.0 / 138.2 | 113.8 / 74.1 / 98.1 | 2.773 | 0.9063 | 0.9063 | 0.27833 | Yes |

Structural masses follow the approved geometry and exclude catalog motor/reducer cases. Arm mass includes all modeled stock and catalog hardware. Pmax covers the recorded finite basis; it is not a proof of a global optimum or a supplier rating.

The full 3D CoM and inertia of every structural assembly are in each run's link table.

- **07:** [MJCF model](<../../../GL_FIXED_550_07/20261105T082343_159550Z_2f971611/best_model.xml>), [link mass / CoM / inertia](<../../../GL_FIXED_550_07/20261105T082343_159550Z_2f971611/best_links.csv>), [hardware and placement](<../../../GL_FIXED_550_07/20261105T082343_159550Z_2f971611/best_joints.csv>), [full result](<../../../GL_FIXED_550_07/20261105T082343_159550Z_2f971611/RESULT.md>)
- **08:** [MJCF model](<../../../GL_FIXED_550_08/20261105T082343_201966Z_fb54fecc/best_model.xml>), [link mass / CoM / inertia](<../../../GL_FIXED_550_08/20261105T082343_201966Z_fb54fecc/best_links.csv>), [hardware and placement](<../../../GL_FIXED_550_08/20261105T082343_201966Z_fb54fecc/best_joints.csv>), [full result](<../../../GL_FIXED_550_08/20261105T082343_201966Z_fb54fecc/RESULT.md>)
- **09:** [MJCF model](<../../../GL_FIXED_550_09/20261105T082343_165681Z_3d5c8240/best_model.xml>), [link mass / CoM / inertia](<../../../GL_FIXED_550_09/20261105T082343_165681Z_3d5c8240/best_links.csv>), [hardware and placement](<../../../GL_FIXED_550_09/20261105T082343_165681Z_3d5c8240/best_joints.csv>), [full result](<../../../GL_FIXED_550_09/20261105T082343_165681Z_3d5c8240/RESULT.md>)

Saved hardware and peak loads at the required upper payload of 500 g:

- [Selected motor and reducer specifications](selected_hardware.csv)
- [Gravity, dynamic torque and rating utilization by joint](joint_loads.csv)

Load tables aggregate reachable pose rows only; their count is recorded in each row.

- [Best sampled design for each J2 voltage/current alternative](J2_voltage_tradeoff.csv)

The voltage table uses the same conservative-payload/required-load ranking within each J2 model. Listed current is not a phase-RMS or motor-heating measurement; no electrical-loss objective is invented.

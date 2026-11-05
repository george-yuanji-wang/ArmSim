# Independent configuration comparison

Required fully extended shoulder-to-TCP reach: **550–550 mm**. Required payload: **500–500 g**.

Each configuration was searched independently with 200 allocated evaluations with the same 1008-state basis, seed 42, joint speeds J1–J6 [3.1416, 3.1416, 3.1416, 5.236, 5.236, 5.236] rad/s and acceleration 6 rad/s².

Common J5-to-J6 spacing: 110 mm; short L2 coupling: 90 mm.

Reducer rating policy: **hold_and_repeated_peak**. Pmax uses continuous holding and speed-dependent motor transient envelopes; unverified pulse duration, thermal duty and extrapolated voltage curves remain explicit assumptions. Conditional known-constraint capacity omits unverified motor transient limits; it does not establish a certified payload or duty/lifetime pass.

These are the best tested candidates under the recorded ranking: higher selected-policy Pmax; when Pmax rounds to zero, lower worst required-payload load-to-rating ratio, then higher conditional known-constraint capacity; lower arm mass breaks remaining ties. Scores are compared at nine decimal places. If every Pmax is zero, selection therefore minimizes the worst overload at the required payload. Conditional capacities are measurements of the selected designs, not independently maximized payloads, and may decrease as overload improves. J_G is retained for comparison rather than optimized.

| Configuration | Evaluated / allocated | Extension (mm) | L1 / L2 / L3 (mm) | Structural L1 / L2 / L3 (g) | Arm mass (kg) | Transient-envelope Pmax (kg) | Conditional capacity (kg) | J_G | Sampled requirements met |
|---|---:|---:|---|---|---:|---:|---:|---:|---|
| 07 | 200 / 200 | 550.0 | 252.3 / 90.0 / 59.1 | 98.2 / 83.8 / 41.5 | 3.025 | 0.9184 | 0.9184 | 0.24818 | Yes |
| 08 | 200 / 200 | 550.0 | 202.0 / 90.0 / 129.8 | 124.9 / 83.8 / 82.7 | 3.124 | 0.9008 | 0.9008 | 0.24205 | Yes |
| 09 | 200 / 200 | 550.0 | 209.1 / 90.0 / 121.2 | 125.5 / 83.8 / 81.7 | 3.093 | 0.9098 | 0.9098 | 0.24266 | Yes |

Structural masses follow the approved geometry and exclude catalog motor/reducer cases. Arm mass includes all modeled stock and catalog hardware. Pmax covers the recorded finite basis; it is not a proof of a global optimum or a supplier rating.

The full 3D CoM and inertia of every structural assembly are in each run's link table.

- **07:** [MJCF model](<../../../GL_CD20_550_07/20261105T090917_848639Z_858d5f48/best_model.xml>), [link mass / CoM / inertia](<../../../GL_CD20_550_07/20261105T090917_848639Z_858d5f48/best_links.csv>), [hardware and placement](<../../../GL_CD20_550_07/20261105T090917_848639Z_858d5f48/best_joints.csv>), [full result](<../../../GL_CD20_550_07/20261105T090917_848639Z_858d5f48/RESULT.md>)
- **08:** [MJCF model](<../../../GL_CD20_550_08/20261105T090917_826307Z_9314dc03/best_model.xml>), [link mass / CoM / inertia](<../../../GL_CD20_550_08/20261105T090917_826307Z_9314dc03/best_links.csv>), [hardware and placement](<../../../GL_CD20_550_08/20261105T090917_826307Z_9314dc03/best_joints.csv>), [full result](<../../../GL_CD20_550_08/20261105T090917_826307Z_9314dc03/RESULT.md>)
- **09:** [MJCF model](<../../../GL_CD20_550_09/20261105T090917_839683Z_f97256e1/best_model.xml>), [link mass / CoM / inertia](<../../../GL_CD20_550_09/20261105T090917_839683Z_f97256e1/best_links.csv>), [hardware and placement](<../../../GL_CD20_550_09/20261105T090917_839683Z_f97256e1/best_joints.csv>), [full result](<../../../GL_CD20_550_09/20261105T090917_839683Z_f97256e1/RESULT.md>)

Saved hardware and peak loads at the required upper payload of 500 g:

- [Selected motor and reducer specifications](selected_hardware.csv)
- [Gravity, dynamic torque and rating utilization by joint](joint_loads.csv)

Load tables aggregate reachable pose rows only; their count is recorded in each row.

- [Best sampled design for each J2 voltage/current alternative](J2_voltage_tradeoff.csv)

The voltage table uses the same conservative-payload/required-load ranking within each J2 model. Listed current is not a phase-RMS or motor-heating measurement; no electrical-loss objective is invented.

## Verification and wrist voltage sensitivity

The [independent audit](independent_audit.json) passed mass-matrix, gravity and velocity-load checks at all 1,008 poses, 550 mm extension checks, and full-rank six-dimensional tool Jacobians at ten generic poses per winner. J5's CS11 repeated-peak limit of 4.5 Nm sets each Pmax boundary; adding 1 g fails the boundary check. [Published-model verification](public_model_verification.json) passed byte identity, metadata and six-actuator checks, and rendered QA passed.

The [same-geometry wrist voltage sensitivity check](voltage_sensitivity.csv) uses the official GL30 12 V curve without reoptimization. Pmax is **0.9059 / 0.8855 / 0.8850 kg** for 07 / 08 / 09; all still meet the sampled 500 g requirement. The active common 16 V bus uses an explicit GL30 curve estimate. Both cases remain conditional transient-envelope results without verified thermal duty or transmission losses. The 200-evaluation searches do not prove a global optimum.

# Independent configuration comparison

Required fully extended shoulder-to-TCP reach: **550–550 mm**. Required payload: **500–500 g**.

Each configuration was searched independently with 1 allocated evaluations with the same 1008-state basis, seed 42, joint speeds J1–J6 [3.1416, 3.1416, 3.1416, 5.236, 5.236, 5.236] rad/s and acceleration 6 rad/s².

Common J5-to-J6 spacing: 110 mm; short L2 coupling: 90 mm.

Reducer rating policy: **hold_and_repeated_peak**. Pmax uses continuous holding and speed-dependent motor transient envelopes; unverified pulse duration, thermal duty and extrapolated voltage curves remain explicit assumptions. Conditional known-constraint capacity omits unverified motor transient limits; it does not establish a certified payload or duty/lifetime pass.

These are the best tested candidates under the recorded ranking: higher selected-policy Pmax; when Pmax rounds to zero, lower worst required-payload load-to-rating ratio, then higher conditional known-constraint capacity; lower arm mass breaks remaining ties. Scores are compared at nine decimal places. If every Pmax is zero, selection therefore minimizes the worst overload at the required payload. Conditional capacities are measurements of the selected designs, not independently maximized payloads, and may decrease as overload improves. J_G is retained for comparison rather than optimized.

| Configuration | Evaluated / allocated | Extension (mm) | L1 / L2 / L3 (mm) | Structural L1 / L2 / L3 (g) | Arm mass (kg) | Transient-envelope Pmax (kg) | Conditional capacity (kg) | J_G | Sampled requirements met |
|---|---:|---:|---|---|---:|---:|---:|---:|---|
| 07 | 1 / 1 | 550.0 | 175.5 / 90.0 / 161.1 | 89.8 / 83.8 / 52.9 | 3.028 | 0.8686 | 0.8686 | 0.24006 | Yes |
| 08 | 1 / 1 | 550.0 | 175.5 / 90.0 / 161.1 | 122.6 / 83.8 / 88.1 | 3.130 | 0.8686 | 0.8686 | 0.24098 | Yes |
| 09 | 1 / 1 | 550.0 | 175.5 / 90.0 / 161.1 | 122.6 / 83.8 / 88.1 | 3.096 | 0.8686 | 0.8686 | 0.24098 | Yes |

Structural masses follow the approved geometry and exclude catalog motor/reducer cases. Arm mass includes all modeled stock and catalog hardware. Pmax covers the recorded finite basis; it is not a proof of a global optimum or a supplier rating.

The full 3D CoM and inertia of every structural assembly are in each run's link table.

- **07:** [MJCF model](<../../../GL_CD20_550_07/20261105T090755_257184Z_135ccf02/best_model.xml>), [link mass / CoM / inertia](<../../../GL_CD20_550_07/20261105T090755_257184Z_135ccf02/best_links.csv>), [hardware and placement](<../../../GL_CD20_550_07/20261105T090755_257184Z_135ccf02/best_joints.csv>), [full result](<../../../GL_CD20_550_07/20261105T090755_257184Z_135ccf02/RESULT.md>)
- **08:** [MJCF model](<../../../GL_CD20_550_08/20261105T090803_501753Z_650cef27/best_model.xml>), [link mass / CoM / inertia](<../../../GL_CD20_550_08/20261105T090803_501753Z_650cef27/best_links.csv>), [hardware and placement](<../../../GL_CD20_550_08/20261105T090803_501753Z_650cef27/best_joints.csv>), [full result](<../../../GL_CD20_550_08/20261105T090803_501753Z_650cef27/RESULT.md>)
- **09:** [MJCF model](<../../../GL_CD20_550_09/20261105T090811_398687Z_08836457/best_model.xml>), [link mass / CoM / inertia](<../../../GL_CD20_550_09/20261105T090811_398687Z_08836457/best_links.csv>), [hardware and placement](<../../../GL_CD20_550_09/20261105T090811_398687Z_08836457/best_joints.csv>), [full result](<../../../GL_CD20_550_09/20261105T090811_398687Z_08836457/RESULT.md>)

Saved hardware and peak loads at the required upper payload of 500 g:

- [Selected motor and reducer specifications](selected_hardware.csv)
- [Gravity, dynamic torque and rating utilization by joint](joint_loads.csv)

Load tables aggregate reachable pose rows only; their count is recorded in each row.

- [Best sampled design for each J2 voltage/current alternative](J2_voltage_tradeoff.csv)

The voltage table uses the same conservative-payload/required-load ranking within each J2 model. Listed current is not a phase-RMS or motor-heating measurement; no electrical-loss objective is invented.

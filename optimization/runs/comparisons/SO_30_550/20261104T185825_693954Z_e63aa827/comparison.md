# Independent configuration comparison

Required fully extended shoulder-to-TCP reach: **550–550 mm**. Required payload: **500–500 g**.

Each configuration was searched independently with 600 allocated evaluations with the same 1008-state basis, seed 42, joint speeds J1–J6 [3.1416, 3.1416, 3.1416, 5.236, 5.236, 5.236] rad/s and acceleration 6 rad/s².

Common J5-to-J6 spacing: 110 mm; short L2 coupling: 90 mm.

Reducer rating policy: **hold_and_repeated_peak**. Pmax retains instantaneous motor continuous limits. Conditional capacity omits unverified motor transient limits; it does not establish a certified payload or duty/lifetime pass.

These are the best tested candidates under the recorded ranking: higher conservative Pmax; when Pmax rounds to zero, lower worst required-payload load-to-rating ratio, then higher conditional known-constraint capacity; lower arm mass breaks remaining ties. Scores are compared at nine decimal places. If every Pmax is zero, selection therefore minimizes the worst overload at the required payload. Conditional capacities are measurements of the selected designs, not independently maximized payloads, and may decrease as overload improves. J_G is retained for comparison rather than optimized.

| Configuration | Evaluated / allocated | Extension (mm) | L1 / L2 / L3 (mm) | Structural L1 / L2 / L3 (g) | Arm mass (kg) | Conservative Pmax (kg) | Conditional capacity (kg) | J_G | Conservative requirements met |
|---|---:|---:|---|---|---:|---:|---:|---:|---|
| 07 | 600 / 600 | 550.0 | 244.6 / 90.0 / 72.2 | 97.0 / 84.0 / 58.9 | 5.524 | 0 | 0.3059 | 0.58043 | No |
| 08 | 600 / 600 | 550.0 | 181.8 / 90.0 / 153.8 | 144.5 / 84.0 / 108.4 | 5.535 | 0 | 0.557 | 0.4843 | No |
| 09 | 600 / 600 | 550.0 | 181.8 / 90.0 / 153.8 | 138.8 / 84.0 / 108.4 | 5.466 | 0 | 0.6009 | 0.47161 | No |

Structural masses follow the approved geometry and exclude catalog motor/reducer cases. Arm mass includes all modeled stock and catalog hardware. Pmax covers the recorded finite basis; it is not a proof of a global optimum or a supplier rating.

The full 3D CoM and inertia of every structural assembly are in each run's link table.

- **07:** [MJCF model](<../../../SO_30_550_07/20261104T182726_116834Z_4ccaac15/best_model.xml>), [link mass / CoM / inertia](<../../../SO_30_550_07/20261104T182726_116834Z_4ccaac15/best_links.csv>), [hardware and placement](<../../../SO_30_550_07/20261104T182726_116834Z_4ccaac15/best_joints.csv>), [full result](<../../../SO_30_550_07/20261104T182726_116834Z_4ccaac15/RESULT.md>)
- **08:** [MJCF model](<../../../SO_30_550_08/20261104T182740_610868Z_855632aa/best_model.xml>), [link mass / CoM / inertia](<../../../SO_30_550_08/20261104T182740_610868Z_855632aa/best_links.csv>), [hardware and placement](<../../../SO_30_550_08/20261104T182740_610868Z_855632aa/best_joints.csv>), [full result](<../../../SO_30_550_08/20261104T182740_610868Z_855632aa/RESULT.md>)
- **09:** [MJCF model](<../../../SO_30_550_09/20261104T182740_606750Z_af908b6a/best_model.xml>), [link mass / CoM / inertia](<../../../SO_30_550_09/20261104T182740_606750Z_af908b6a/best_links.csv>), [hardware and placement](<../../../SO_30_550_09/20261104T182740_606750Z_af908b6a/best_joints.csv>), [full result](<../../../SO_30_550_09/20261104T182740_606750Z_af908b6a/RESULT.md>)

Saved hardware and peak loads at the required upper payload of 500 g:

- [Selected motor and reducer specifications](selected_hardware.csv)
- [Gravity, dynamic torque and rating utilization by joint](joint_loads.csv)

Load tables aggregate reachable pose rows only; their count is recorded in each row.

- [Best sampled design for each J2 voltage/current alternative](J2_voltage_tradeoff.csv)

The voltage table uses the same conservative-payload/required-load ranking within each J2 model. Listed current is not a phase-RMS or motor-heating measurement; no electrical-loss objective is invented.

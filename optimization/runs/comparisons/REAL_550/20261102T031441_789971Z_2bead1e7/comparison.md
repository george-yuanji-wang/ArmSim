# Independent configuration comparison

Required fully extended shoulder-to-TCP reach: **550–550 mm**. Required payload: **500–1000 g**.

Each configuration was searched independently for 200 candidates with the same 1008-state basis, seed 42, joint speed 1 rad/s and acceleration 2 rad/s².

| Configuration | Extension (mm) | L1 / L2 / L3 (mm) | Structural L1 / L2 / L3 (g) | Arm mass (kg) | Pmax (kg) | J_G | Requirements met |
|---|---:|---|---|---:|---:|---:|---|
| 07 | 550.0 | 277.9 / 90.0 / 82.5 | 165.2 / 83.1 / 75.9 | 5.636 | 7.946 | 0.022488 | Yes |
| 08 | 550.0 | 230.8 / 90.0 / 143.3 | 194.9 / 83.1 / 114.7 | 6.011 | 6.596 | 0.025625 | Yes |
| 09 | 550.0 | 216.4 / 90.0 / 160.1 | 193.2 / 82.8 / 116.8 | 6.624 | 6.228 | 0.026359 | Yes |

Structural masses follow the approved geometry and exclude catalog motor/reducer cases. Arm mass includes all modeled stock and catalog hardware. Pmax covers the recorded finite basis; it is not a proof of a global optimum or a supplier rating.

The full 3D CoM and inertia of every structural assembly are in each run's link table.

- **07:** [MJCF model](<../../../REAL_550_07/20261102T030516_358757Z_fa672418/best_model.xml>), [link mass / CoM / inertia](<../../../REAL_550_07/20261102T030516_358757Z_fa672418/best_links.csv>), [hardware and placement](<../../../REAL_550_07/20261102T030516_358757Z_fa672418/best_joints.csv>), [full result](<../../../REAL_550_07/20261102T030516_358757Z_fa672418/RESULT.md>)
- **08:** [MJCF model](<../../../REAL_550_08/20261102T030516_358316Z_a6ab8630/best_model.xml>), [link mass / CoM / inertia](<../../../REAL_550_08/20261102T030516_358316Z_a6ab8630/best_links.csv>), [hardware and placement](<../../../REAL_550_08/20261102T030516_358316Z_a6ab8630/best_joints.csv>), [full result](<../../../REAL_550_08/20261102T030516_358316Z_a6ab8630/RESULT.md>)
- **09:** [MJCF model](<../../../REAL_550_09/20261102T030516_346193Z_0404b909/best_model.xml>), [link mass / CoM / inertia](<../../../REAL_550_09/20261102T030516_346193Z_0404b909/best_links.csv>), [hardware and placement](<../../../REAL_550_09/20261102T030516_346193Z_0404b909/best_joints.csv>), [full result](<../../../REAL_550_09/20261102T030516_346193Z_0404b909/RESULT.md>)

# Independent configuration comparison

Required fully extended shoulder-to-TCP reach: **450–600 mm**. Required payload: **500–1000 g**.

Each configuration was searched independently for 200 candidates with the same 1008-state basis, seed 42, joint speed 1 rad/s and acceleration 2 rad/s².

| Configuration | Extension (mm) | L1 / L2 / L3 (mm) | Structural L1 / L2 / L3 (g) | Arm mass (kg) | Pmax (kg) | J_G | Requirements met |
|---|---:|---|---|---:|---:|---:|---|
| 07 | 450.0 | 182.4 / 90.0 / 75.7 | 141.3 / 83.6 / 49.1 | 4.834 | 4.452 | 0.029973 | Yes |
| 08 | 535.9 | 231.7 / 90.0 / 125.3 | 180.5 / 83.6 / 86.7 | 4.594 | 2.66 | 0.10162 | Yes |
| 09 | 513.4 | 211.1 / 90.0 / 122.9 | 178.1 / 83.6 / 86.3 | 4.556 | 2.711 | 0.095143 | Yes |

Structural masses follow the approved geometry and exclude catalog motor/reducer cases. Arm mass includes all modeled stock and catalog hardware. Pmax covers the recorded finite basis; it is not a proof of a global optimum or a supplier rating.

The full 3D CoM and inertia of every structural assembly are in each run's link table.

- **07:** [MJCF model](<../../../SYNTHETIC_DEMO_07/20261102T005029_887555Z_88a9cf2b/best_model.xml>), [link mass / CoM / inertia](<../../../SYNTHETIC_DEMO_07/20261102T005029_887555Z_88a9cf2b/best_links.csv>), [hardware and placement](<../../../SYNTHETIC_DEMO_07/20261102T005029_887555Z_88a9cf2b/best_joints.csv>), [full result](<../../../SYNTHETIC_DEMO_07/20261102T005029_887555Z_88a9cf2b/RESULT.md>)
- **08:** [MJCF model](<../../../SYNTHETIC_DEMO_08/20261102T005029_891036Z_0f8e90ca/best_model.xml>), [link mass / CoM / inertia](<../../../SYNTHETIC_DEMO_08/20261102T005029_891036Z_0f8e90ca/best_links.csv>), [hardware and placement](<../../../SYNTHETIC_DEMO_08/20261102T005029_891036Z_0f8e90ca/best_joints.csv>), [full result](<../../../SYNTHETIC_DEMO_08/20261102T005029_891036Z_0f8e90ca/RESULT.md>)
- **09:** [MJCF model](<../../../SYNTHETIC_DEMO_09/20261102T005029_888307Z_328ae156/best_model.xml>), [link mass / CoM / inertia](<../../../SYNTHETIC_DEMO_09/20261102T005029_888307Z_328ae156/best_links.csv>), [hardware and placement](<../../../SYNTHETIC_DEMO_09/20261102T005029_888307Z_328ae156/best_joints.csv>), [full result](<../../../SYNTHETIC_DEMO_09/20261102T005029_888307Z_328ae156/RESULT.md>)

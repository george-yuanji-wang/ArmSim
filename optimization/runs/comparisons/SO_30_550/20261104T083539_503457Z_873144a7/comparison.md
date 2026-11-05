# Independent configuration comparison

Required fully extended shoulder-to-TCP reach: **550–550 mm**. Required payload: **500–500 g**.

Each configuration was searched independently for 200 candidates with the same 1008-state basis, seed 42, joint speeds J1–J6 [3.1416, 3.1416, 3.1416, 5.236, 5.236, 5.236] rad/s and acceleration 6 rad/s².

Common J5-to-J6 spacing: 110 mm; short L2 coupling: 90 mm.

| Configuration | Extension (mm) | L1 / L2 / L3 (mm) | Structural L1 / L2 / L3 (g) | Arm mass (kg) | Pmax (kg) | J_G | Requirements met |
|---|---:|---|---|---:|---:|---:|---|
| 07 | 550.0 | 124.5 / 90.0 / 217.6 | 84.4 / 84.6 / 75.3 | 5.164 | 0 | 0.47896 | No |
| 08 | 550.0 | 162.2 / 90.0 / 176.2 | 136.3 / 84.0 / 112.1 | 4.803 | 0 | 1.1782 | No |
| 09 | 550.0 | 172.2 / 90.0 / 164.9 | 131.0 / 84.2 / 112.3 | 5.337 | 0 | 0.53495 | No |

Structural masses follow the approved geometry and exclude catalog motor/reducer cases. Arm mass includes all modeled stock and catalog hardware. Pmax covers the recorded finite basis; it is not a proof of a global optimum or a supplier rating.

The full 3D CoM and inertia of every structural assembly are in each run's link table.

- **07:** [MJCF model](<../../../SO_30_550_07/20261104T082459_674180Z_a866ce2a/best_model.xml>), [link mass / CoM / inertia](<../../../SO_30_550_07/20261104T082459_674180Z_a866ce2a/best_links.csv>), [hardware and placement](<../../../SO_30_550_07/20261104T082459_674180Z_a866ce2a/best_joints.csv>), [full result](<../../../SO_30_550_07/20261104T082459_674180Z_a866ce2a/RESULT.md>)
- **08:** [MJCF model](<../../../SO_30_550_08/20261104T082459_670828Z_0dcdd74d/best_model.xml>), [link mass / CoM / inertia](<../../../SO_30_550_08/20261104T082459_670828Z_0dcdd74d/best_links.csv>), [hardware and placement](<../../../SO_30_550_08/20261104T082459_670828Z_0dcdd74d/best_joints.csv>), [full result](<../../../SO_30_550_08/20261104T082459_670828Z_0dcdd74d/RESULT.md>)
- **09:** [MJCF model](<../../../SO_30_550_09/20261104T082459_672228Z_195ad436/best_model.xml>), [link mass / CoM / inertia](<../../../SO_30_550_09/20261104T082459_672228Z_195ad436/best_links.csv>), [hardware and placement](<../../../SO_30_550_09/20261104T082459_672228Z_195ad436/best_joints.csv>), [full result](<../../../SO_30_550_09/20261104T082459_672228Z_195ad436/RESULT.md>)

## Hardware and load diagnostics

Only the selected STEPPERONLINE motor catalog and the three prescribed 30:1 reducers were used.

| Configuration | Peak empty-arm demand / rating | Peak 500 g demand / rating | Failure reason |
|---|---:|---:|---|
| 07 | 1.0769 | 2.2088 | empty_arm_exceeds_basis_limits |
| 08 | 1.6518 | 3.1169 | empty_arm_exceeds_basis_limits |
| 09 | 1.0479 | 2.1425 | empty_arm_exceeds_basis_limits |

A ratio above 1 exceeds a continuous motor or reducer rating somewhere in the suite. Zero-capacity candidates were ranked by their lowest peak empty-arm utilization before mass. They are retained as best tested designs, with requirements_met=false, rather than successful payload designs.

Published rotor inertia was unavailable; all motor simulations use the separately recorded conservative M_motor × R_envelope² upper bound. Transmission efficiency is ideal; friction and reducer rotating inertia remain unmeasured. These assumptions are recorded in each run.

- [Selected motor and reducer specifications](selected_hardware.csv)
- [Maximum gravity and dynamic joint loads at 500 g](joint_loads_at_500g.csv)
- [Exported model locations and outcome](study_outcome.json)

An independent reconstruction from the saved gravity, mass-matrix and velocity
coefficients confirms the 500 g reducer loads without any reflected motor
rotor-inertia term. All three best tested designs exceed the J1, J2, J4 and J5
continuous reducer ratings; 07 also exceeds J3. Thus the 500 g failures do not
depend solely on the conservative motor-inertia bound. Empty-arm reducer loads
already exceed J1/J2 ratings in 07 and 09; 08 passes the reducer-only empty test
but fails motor limits.

[Independent numerical audit](independent_audit.json) records the reconstructed
loads, reach, six-DOF and mass checks. This audits the tested designs and finite
suite; it does not prove that no other design can meet the requirements.

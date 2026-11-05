# base-6dof

[`init_model.xml`](init_model.xml) is the standalone starting model for the next
revision. It uses six centered hollow cylindrical joint cages and rectangular
link edge frames, with the original fixed trapezoidal base frame. J4 rolls the forearm and J6
rolls the tool; the short elbow coupling has the same perpendicular J4
arrangement as 07. Local joint axes are Z X X Z X Z.

Its length names are defined independently of the older models:

| Length | Measurement | Value (mm) |
| --- | --- | ---: |
| L1 | Absolute bottom / floor to J2 center, including the fixed base | 220 |
| L2 | J2 to J3 | 280 |
| L3 | J3 to J4, short elbow coupling | 70 |
| L4 | J4 to J5, forearm | 240 |
| L5 | J5 to J6 | 80 |
| L6 | J6 to tool center / TCP | 60 |

The fixed base's full height is 41.8 mm. J1 is at 60 mm, so the moving
J1-to-J2 column is 160 mm; the base height is already included in L1.
Each physical body has an explicit `<inertial>` block specifying mass, local
CoM, principal moments and their orientation. These preserve the original
solid-prism concept's mass properties, including the full inertia tensor.
`inertiafromgeom="false"` and zero-mass visual geoms keep appearance independent
of dynamics: changing the visual edge thickness does not change simulated mass.
The links have twelve edges each; joint cages use two open annular rings and
four axial rails. All visuals have contacts disabled. Edit body inertial blocks
to change physical assumptions, and frame geometry to change appearance.
The commented XML is self-contained and has no drivetrain catalog or
optimization-export dependency.

Open `view_models.py` and select **File / init_model**, or launch it directly:

```powershell
uv run python view_models.py --file model/init_model.xml
```

The viewer supports pose sliders, coordinate frames, labels and demo animation
for this generic MJCF. It is a pose-inspection model with no actuators.

The published optimization exports are
[`best-07-base-6dof.xml`](best-07-base-6dof.xml),
[`best-08-base-6dof.xml`](best-08-base-6dof.xml), and
[`best-09-base-6dof.xml`](best-09-base-6dof.xml).
[`best_model.xml`](best_model.xml) selects 07. They preserve the approved edge
frames and six axes, and match the saved scorers from the
[completed GL_CD20_550 comparison](../optimization/runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/comparison.md).
Each configuration completed 200 evaluations over all 1,008 poses. Pmax is
0.9184 / 0.9008 / 0.9098 kg for 07 / 08 / 09; all meet the sampled 500 g at
550 mm requirement under the conditional transient speed envelope and GL30
16 V curve estimate. The finite search budget does not prove a global optimum.
The [independent audit](../optimization/runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/independent_audit.json)
and [published-model verification](../optimization/runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/public_model_verification.json)
passed. A [same-geometry official-12 V wrist recheck](../optimization/runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/voltage_sensitivity.csv)
still meets the sampled 500 g requirement for all three without reoptimization.
The active [GL_CD20_550 study](../optimization/README.md) uses
HD14_50, CD20_50 and CD20_30 on J1-J3, and a CS11 series 30:1 wrist model with
the user-specified 43 mm diameter × 32 mm envelope and 200 g mass. The exact
CS11 variant is unspecified. Previous public exports are retained under `archive/`,
and the [historical GL_FIXED_550 comparison](../optimization/runs/comparisons/GL_FIXED_550/20261105T083535_687425Z_21e4c5fd/comparison.md)
remains attached to its original CD-17/CS-11-1U-E hardware and input snapshots.

For the nine motor/reducer packaging variants and interactive viewer, see the
[MJCF layout catalog](MJCF.md).

`base-6dof.urdf` is a generic arm with six revolute joints, primitive visual
and collision geometry, link masses and COM inertias, joint limits, and damping.
It has no external mesh dependencies.

The axes below are expressed in each joint's local frame. All joint origin
rotations are zero, so these axes also match the base frame at zero joint angles.
Upstream joints rotate the downstream axes as the arm moves. J4 is a roll
joint along L3's length, which is local Z in these frames. It remains
perpendicular to J3=X. Its rotation leaves the J4-to-J5 centerline fixed and
spins the link around that line. J6 likewise rolls the tool about its local Z
axle; rotating J6 leaves the tool-tip position fixed. J5 is the intervening
X-axis bend. All active models use this roll-bend-roll wrist.

Models 07-09 use `base-6dof-perpendicular-j4.urdf` as their short-L2 kinematic
reference. Its J4 origin has an X rotation of -90 degrees: the local J4 Z axle
points along parent +Y, perpendicular to L2's parent Z direction. L3 follows
the J4 axle and rolls in place. Local joint axes remain Z X X Z X Z.
At zero angles its tool is at (0, 0.42, 0.63) m.

The URDFs are kinematic references with simple primitive visuals. The viewer's
generated MJCFs contain the edge-only spatial structures and actuator layouts.

| Joint | Parent -> child | Axis | Range (degrees) | Effort (N m) | Speed (rad/s) |
| --- | --- | --- | --- | --- | --- |
| joint_1 | base_link -> shoulder_link | Z | -180 to 180 | 100 | 2 |
| joint_2 | shoulder_link -> upper_arm_link | X | -90 to 90 | 100 | 2 |
| joint_3 | upper_arm_link -> forearm_link | X | -150 to 150 | 80 | 2 |
| joint_4 | forearm_link -> wrist_1_link | Z (L3 roll) | -180 to 180 | 20 | 3 |
| joint_5 | wrist_1_link -> wrist_2_link | X | -90 to 90 | 15 | 3 |
| joint_6 | wrist_2_link -> tool_flange_link | Z (tool roll) | -180 to 180 | 10 | 3 |

The base origin is on its bottom mounting face. In the zero configuration the
arm stands straight along +Z; `tool0` is at (0, 0, 1.05) m relative to `base_link`.
The upper arm is 0.32 m long and the forearm is 0.28 m long. Total modeled mass
is 11.65 kg, including the 4 kg base. Masses and dimensions are placeholders;
the inertias match the corresponding uniform solid boxes and cylinders.
The straight zero configuration is singular; use a bent pose for IK examples.

`tool0` is a fixed, massless attachment frame at the flange tip. There is no
gripper or payload. The base is fixed when imported into MuJoCo. In other
simulators, attach `base_link` to the world or a mounting frame as needed.

From the repository root, preview with:

```powershell
uv run python -m mujoco.viewer --mjcf=model/base-6dof.urdf
```

Or load from Python:

```python
import mujoco

model = mujoco.MjModel.from_xml_path("model/base-6dof.urdf")
data = mujoco.MjData(model)
```

The file includes a small MuJoCo compiler extension to retain visuals, named
fixed frames, and the supplied inertias. The URDF describes the mechanism;
MuJoCo actuators/controllers must be configured separately. URDF effort and
velocity limits are specifications and do not themselves install controllers
or enforce motor limits in MuJoCo.

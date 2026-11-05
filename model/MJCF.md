# Robot-arm packaging catalog

Nine standalone MJCF scenes share a roll-bend-roll wrist. J4 rolls L3 about its
own length, J5 bends the wrist, and J6 rolls the tool about its axle.
**All structural components use thin edges forming spatial frames.**

## Configurations

| Model | L2 / L3 | Motor arrangement | L2 versus J4 axle |
| --- | --- | --- | --- |
| 01 | 280 / 80 mm | All motors coaxial | Parallel |
| 02 | 280 / 80 mm | J3 belt motor; J2 below | Parallel |
| 03 | 280 / 80 mm | Same J3 motor/belt as 02; J2 coaxial | Parallel |
| 04 | 90 / 270 mm | All motors coaxial | Parallel |
| 05 | 90 / 270 mm | J3/J5 belt motors; J2 below | Parallel |
| 06 | 90 / 270 mm | Same J3/J5 motors/belts as 05; J2 coaxial | Parallel |
| 07 | 90 / 270 mm | Same drive choices as 04 | **Perpendicular, 90 degrees** |
| 08 | 90 / 270 mm | Same drive choices as 05 | **Perpendicular, 90 degrees** |
| 09 | 90 / 270 mm | Same drive choices as 06 | **Perpendicular, 90 degrees** |

02/03 differ only at J2: lower belt motor versus coaxial motor. Their other
motor positions, belts and structural geometry match. The same comparison
applies to 05/06 and 08/09. J3 belt motors use a common 145 mm retreat; J5 belt
motors use 130 mm. J2's lower motor sits 57 mm below its joint.

J4/J6 motors are coaxial in every scene. Each reducer is at its joint, with
exactly one reduction per motor. Optional 1:1 input belts stay on one parent
link and do not cross a moving joint or redirect a drive shaft.

## The actual 90-degree family

In **07-09, L2 is perpendicular to the J4 motor and reducer axle**. This changes
the joint frame, not just the support bracket. The J4 origin rotates -90 degrees
about parent X. Its local Z axle therefore points along parent +Y, while L2
extends along parent Z. J3 remains perpendicular to J4.

The motor, reducer, output bearing and L3 all follow that same J4 axle. L3
extends sideways from L2 and rolls about its length. The motor remains coaxial;
there is no bevel drive or right-angle gearing. Rotating J4 leaves both ends
of L3 fixed while its cross-section spins.

`base-6dof-perpendicular-j4.urdf` records the short-L2 right-angle reference.
`base-6dof.urdf` is the original parallel-axis family reference. Both use local
joint axes Z X X Z X Z; the origin rotation accounts for the physical angle.

The parallel short-L2 family has the same tool kinematics as the long family
when L2+L3 stays 360 mm. The perpendicular family has different kinematics and
workspace. Its zero-angle tool position is (0, 0.42, 0.63) m; the parallel
family's is (0, 0, 1.05) m.

## Fixed cases and moving outputs

- **Green:** motor stator case, fixed relative to its parent link.
- **Orange:** reducer case and fixed bearing exterior, fixed to the parent.
- **Grey disks:** output flanges belonging to the moving child link.
- **Purple:** input couplings, pulleys and belts.
- **Blue:** structural edge frames; the base frame is dark grey.

Every green and orange case has **four equal-length attachment edges** ending
at four distinct points on its outer rim at the **middle of its axial height**.
Four separate anchors connect to a rectangular or trapezoidal support base.
Coaxial roll cases use symmetric four-point supports; pitch and offset cases
use balanced saddles. No support converges to a single apex.

Fixed supports attach to the orange/green cases. They do not attach to a grey
moving disk. Each grey output has four rim contacts in its own plane and a
rectangular planar surround. A second surrounding loop and axial posts
make a 3D outgoing frame attached only to the child. Four distinct, equal-length
connectors feed the next link without fan-out from a shared point. The terminal grey tool
disk also has four planar rim connections to its surrounding tool frame.

```text
parent edge cage -> orange case mid-height rim -> reducer / bearing
                 -> grey moving flange -> planar rim surround
                 -> spatial output cage -> outgoing child edge cage
```

Cases and output disks remain solid hardware envelopes. **Structure has no
plates, filled gussets, boxes or mesh surfaces.** Link cages have depth as well
as width and length. Rectangular faces have perimeter edges only, without
diagonals. Structural edge radii are 1.5-1.8 mm. Models contain 316-362 geometry
pieces, with no mesh assets. The terminal tool cage is 48 x 48 mm rather than
66 x 66 mm, and the terminal disk radius is 19.5 mm rather than 27 mm. The
tool attachment location is unchanged. Input belt planes move from 55 to 49 mm
behind their joints, retaining checked clearance to the structure.

## Axes and mobility

The full 6x6 position-and-orientation Jacobian has rank six at the bent pose
and in 500 sampled poses per model. Independent numerical derivatives of URDF
forward kinematics agree with MuJoCo. J4 and J6 are separately checked over
-180 to +180 degrees for the required fixed-centerline roll motion.
The report and model hashes are in [tool-dof-audit.json](tool-dof-audit.json).

Zero pose is singular: rank three for 01-06 and rank five for 07-09. The viewer
shows live mobility and singular/near-singular warnings. Sampling checks local
kinematic freedom, not collision-free reachability of every workspace pose.

## Viewer and verification

```powershell
uv run python view_models.py --model 7
```

The default and Bent pose use a shared **>** silhouette and the same tool pose
across all nine models. Parallel families use joint angles -30, 50, -100, 0,
-10, 0 degrees. The perpendicular family compensates its fixed J4 rotation
with approximately -30, 36.44, -15.73, 0, 9.29, 0 degrees. These poses remain
full rank. The Home camera shows the bend from the common comparison side.
Switching from a default Bent pose selects the new family's matching angles;
manually edited targets remain under user control.

Sliders and numeric fields set targets. Pose mode moves directly; Dynamics uses torque-limited
servos under gravity. Drag to orbit, right-drag to pan and scroll to zoom.
Click a component to select; click again, background, Escape or Deselect to
clear it. The View tab controls edge-support visibility, labels, frames and
render quality. The Drivetrain tab describes interfaces and body ownership.

Edge masses and inertias follow cylindrical aluminum volume at 2700 kg/m3.
Motor/reducer masses are component estimates; the fixed bearing exterior is
included in the reducer estimate. Output mass belongs to the child. Input
couplings and bearings are ideal: compliance, backlash and internal contact
are not modeled. Display geometry has contact disabled for layout exploration.

Tests verify the physical 90-degree relationship, six-axis motion, both rolls,
mid-height case ports, spatial edge-only parts, planar moving output contacts,
fixed-support clearance from grey disks, own-housing clearance, body ownership,
J2-only pair differences, equal four-point attachments, absence of rectangular
face diagonals, matched tool poses, COM integration, torque limits and finite dynamics.
Qt checks all nine scenes, the > silhouette and matching defaults, actual axis directions,
picking/deselection, resizing/DPI, idle redraw and image export.

These are packaging studies. Stiffness, loads, fasteners, tolerances and full
travel all-part clearance need subsequent mechanical design. Reference source
audits are in [references/README.md](../references/README.md).

```powershell
uv run python -m armsim.generate_models
uv run python -m tools.audit_tool_dof
uv run python -m unittest discover -s tests -v
uv run python view_models.py --smoke-test
```

Edit `armsim/catalog.py`, `armsim/mechanics.py`, `armsim/brackets.py` and the
generator. Numbered scenes and the perpendicular URDF are generated outputs.

# ArmSim

Compare nine robot-arm packaging bases: long L2, short L2 with a parallel J4
axle, and short L2 with the J4 axle perpendicular to L2. Every scene has the same
roll-bend-roll wrist: J4 rolls L3, J5 bends the wrist, and J6 rolls the tool.
The Qt desktop viewer uses MuJoCo for rendering and dynamics, with joint sliders,
precise angle fields, component labels, and moving coordinate frames.
Uses the [official MuJoCo Python bindings](https://mujoco.readthedocs.io/en/stable/python.html).

The separate [headless optimization workflow](optimization/README.md) runs
configurations **07, 08 and 09 independently**, using 1,008 reference joint
states and persistent checkpointed results. The active **GL_CD20_550** study
fixes fully extended shoulder-to-tool reach at **550 mm** and payload at
**500 g**. Its hardware is fixed:

| Joints | Motor | Reducer | Ratio |
|---|---|---|---:|
| J1 | CubeMars GL40 KV210 | SITO HD14_50 | 50:1 |
| J2 | CubeMars GL40 KV210 | SITO CD20_50 | 50:1 |
| J3 | CubeMars GL40 KV210 | SITO CD20_30 | 30:1 |
| J4/J5/J6 | CubeMars GL30 KV290 | SITO CS11 series (variant unspecified) | 30:1 |

The active wrist reducer uses the user-specified **43 mm diameter × 32 mm
axial envelope and 200 g mass**. Its exact CS11 variant is unspecified; these
are modeling inputs, not verified CS-11-1U-E drawing dimensions. The joint
ratios are **50/50/30/30/30/30**, on a common **16 V bus**.

The [completed GL_CD20_550 comparison](optimization/runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/comparison.md)
records **200 evaluations per configuration**, each over all **1,008 poses**.
Pmax is **0.9184 / 0.9008 / 0.9098 kg** for 07 / 08 / 09 under the conditional
transient-envelope policy. All three meet the sampled **500 g at 550 mm**
requirement. These are the best tested candidates within the finite budget;
the search does not prove a global optimum. Current complete models are
[`model/best-07-base-6dof.xml`](model/best-07-base-6dof.xml),
[`model/best-08-base-6dof.xml`](model/best-08-base-6dof.xml) and
[`model/best-09-base-6dof.xml`](model/best-09-base-6dof.xml);
[`model/best_model.xml`](model/best_model.xml) selects 07. The published models
match their saved scorers.

The [independent audit](optimization/runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/independent_audit.json)
passed the dynamics, reach and tool-mobility checks. A [12 V wrist sensitivity recheck](optimization/runs/comparisons/GL_CD20_550/20261105T092248_985041Z_a9aba572/voltage_sensitivity.csv)
using the official GL30 curve on the same geometries gives Pmax
**0.9059 / 0.8855 / 0.8850 kg**; all still meet 500 g. This is a sensitivity
check without reoptimization, and retains the transient-duty assumptions.

The [completed GL_FIXED_550 comparison](optimization/runs/comparisons/GL_FIXED_550/20261105T083535_687425Z_21e4c5fd/comparison.md)
is **historical**: its J2/J3 reducers were CD-17 at 50:1, and its wrist used a
conservative CS-11-1U-E drawing envelope. Its Pmax **0.932 / 0.919 / 0.906 kg**
for 07 / 08 / 09 does not describe the corrected GL_CD20_550 hardware.
Its original run models and prior published exports are retained.

Only the approved link dimensions and bounded local motor placements are
searched; the hardware choices and configuration topology stay fixed. The
current run allocates **200 load evaluations per configuration**, with
checkpoints every **20** evaluations. From the repository root:

```powershell
uv run python -m optimization.real_catalog --study locked-gl
uv run python run_optimization.py --real --cycles 200 --checkpoint-every 20
```

The first command regenerates `optimization/data/real/` and archives its previous
generated inputs. The second runs all three configurations; add `--layout 7`
for 07 alone. Requirements stay in the existing five-column `requirement.csv`.
Motor/reducer specs, the motor torque-speed points in `motor_curve.csv`, and
motion settings in `study.json` are separate files.

Motion tests use **180 deg/s on J1-J3**, **300 deg/s on J4-J6**, and
**6 rad/s^2** acceleration. Primary Pmax uses continuous motor/reducer holding
limits and speed-dependent motor transient curves with repeated-peak reducer
limits during motion. A continuous-only motor diagnostic is saved separately.
Pmax is **conditional transient-envelope capacity**: the study does not establish
timed RMS heating, allowable overload duration or reducer lifetime.

GL40 uses its official 16 V curve. GL30 operation on the common **16 V bus is
an explicit engineering estimate**, derived by shifting its official 12 V curve; it is
not a manufacturer-qualified operating point. The original continuous torque
ratings remain **0.25 Nm / 0.08 Nm**, with the original peak caps retained.
Published rotor inertias replace the previous whole-motor upper bounds.
Transmission losses and unpublished reducer rotating inertias remain unmodeled.

The existing file architecture is preserved: retained manufacturer facts in
`optimization/catalogs/`, generated inputs in `optimization/data/real/`, and
immutable timestamped results in `optimization/runs/`. General editable
templates in `optimization/data/` remain separate. Each completed search writes
a complete `best_model.xml` with the approved edge frames, mounted cases,
moving flanges, labels, coordinate sites and servo settings.

The [November 4 STEPPERONLINE comparison](optimization/runs/comparisons/SO_30_550/20261104T185825_693954Z_e63aa827/comparison.md)
is **historical**: it used eight industrial motor candidates, 30:1 reducers,
600 evaluations per layout and a continuous-motor dynamic screen. Its inputs,
scores and exported models are retained; those results do not describe the
active GL study. The legacy palette can still be regenerated with
`uv run python -m optimization.real_catalog --study stepperonline`.

Synthetic demonstration inputs remain available:

```powershell
uv run python run_optimization.py --demo
uv run python run_optimization.py --demo --layout 8
```

The demo creates separate `SYNTHETIC_DEMO_07`, `_08` and `_09` projects, or only
the selected layout. Its invented hardware is labeled as such. Structural
mass, 3D CoM and inertia follow the generated stock geometry rather than
independent mass budgets. This workflow runs independently of the GUI.

Configuration names state **Long L2 (280 mm)** or **Short L2 (90 mm)**, the J2
motor placement, and whether the J4 axle is **parallel or perpendicular to L2**.
L2 connects J3 to J4.
Models 02/03 differ only in J2's lower belt motor versus a coaxial motor;
their J3 motor positions and belts are identical. J4/J6 motors and reducers
are coaxial in every scene.

All nine scenes achieve six independent local tool motions in the bent pose and
500 sampled poses per scene. The viewer displays live **Tool motion: 6/6**
and warns when a pose becomes singular. The straight zero pose is singular;
use Bent pose to restore full mobility. Details and the numerical audit are in
[the layout notes](model/MJCF.md#axes-and-mobility).

## Interactive viewer

```powershell
uv run python view_models.py
```

The viewer starts maximized. Drag the divider to resize the sidebar, and use
**Interface size** to enlarge the controls. The controls scroll in shorter
windows, and Qt handles monitor DPI scaling. The sidebar width and interface
size are remembered between sessions.

Rendering uses **PySide6 (Qt for Python) with a native OpenGL viewport**. MuJoCo
draws directly into the window; the display loop does not copy full images
through Pillow or Tkinter. Static poses redraw on demand. Resizing keeps the
same graphics context, with the camera preserved and correct device-pixel
resolution. Only PNG export reads the rendered image back from the GPU.

Use the model selector or **[ / ]** to switch layouts. Move the six sliders or
type into their angle fields to set targets in degrees. **Pose** mode moves the arm directly; **Dynamics**
mode runs torque-limited position servos under gravity. Start with **Bent pose**
and try **Animate demo**.

Drag the viewport to orbit, right-drag to pan, and use the mouse wheel to zoom.
Click a robot component or press **1-6** to select its joint. Click it again,
click empty space, press **Escape**, or use **Deselect** to clear the selection. The **View** tab controls labels, motor/gearbox
visibility, and XYZ frames (X red, Y green, Z blue). Select **Selected drivetrain**
to inspect motor and gearbox shaft frames. The **Drivetrain** tab describes
the hardware mounting and reduction assumptions.

**Front / Side / Top / Home** set the camera direction. **Balanced** render
quality is the default; **Fast** removes shadows, and **High** adds floor
reflections. Camera actions move into the toolbar overflow menu in narrow windows.

```powershell
uv run python view_models.py --model 7
uv run python view_models.py --list
```

Additional six-axis MJCF files copied into `model/` appear after the nine presets
when the viewer starts. Use **File → Refresh model list (F5)** after copying a
file while the viewer is open, or **Open MJCF (Ctrl+O)** to open one anywhere.
You can also open an optimization result directly:

```powershell
uv run python view_models.py --file model/best_model.xml
```

Optimization exports contain the same edge-frame structures, mounted motor and
reducer cases, moving flanges, belt routes, labels, coordinate sites and servo
dynamics as the base models. Selected catalog ratios and torque/speed limits are
embedded in the XML; the GUI needs no sidecar files. Generic unmarked MJCF files
remain available for pose inspection.

The current export contract is `approved-base-template-v8`. Older optimization
runs need a new search because their scores used different reach definitions,
test suites or mass models; the export
command rejects legacy winners. Local mounting-fit checks are enforced, while
global travel collisions and mechanical joint stops remain outside the search.
The active study fixes the wrist at 110 mm; original base models retain their
70 mm wrist. Study models preserve their connected frame topology and local
mounting checks. The active CS11 series reducer is modeled with the requested
43 mm diameter, 32 mm axial envelope and 200 g mass, pending exact variant
identification. Historical GL_FIXED_550 runs retain their CS-11-1U-E drawing
interpretation: a 40 mm square flange and 56.5 mm total shaft envelope, with a
56.57 mm bounding circle enclosing the square corners.
Structural masses, CoMs and inertias are calculated from the conceptual stock
geometry; manufacturing and stiffness still need detailed validation.

Both J4 and J6 use **local Z**, along the lengths of their respective links.
The default and **Bent pose** form a comparable **>** silhouette. Family-specific
angles compensate for 07-09's perpendicular J4 frame, matching the tool position
and orientation across all nine models. Switching layouts from Bent pose keeps
this comparison posture.

All structural parts are thin-edge **3D frames**, without plates, filled gussets
or mesh surfaces. Every orange reducer case and green motor case has an edge
ending on its rim at the middle of the disk height. These supports belong to
the parent link. Every grey output disk belongs to the moving child and has
a planar rim surround connected to a spatial outgoing frame.

Orange and green mean fixed **relative to their parent link**; grey disks are
moving outputs. Fixed bearing exteriors are orange, and input-drive parts are
purple. Cases have four equal-length mid-height attachment edges with distinct
ports and rectangular/trapezoidal bases. Frames have no face diagonals or
shared apexes. Structural edge radii are 1.5-1.8 mm. The tool cage is 48 mm wide
instead of 66 mm, and input belt planes sit 6 mm closer to the structure.

See the [MJCF layout catalog and modeling notes](model/MJCF.md) and the
[pinned robot/manufacturer reference audit](references/README.md).

## Setup

From this directory, install the locked dependencies with uv:

```powershell
uv sync --locked
```

Python 3.13 is selected by `.python-version`. Dependencies are declared in
`pyproject.toml`, and exact resolved versions are recorded in `uv.lock`.

## Run Python

Run commands inside the environment without activating it:

```powershell
uv run python
```

Or activate the environment in PowerShell:

```powershell
.\.venv\Scripts\Activate.ps1
python
```

If PowerShell blocks activation, use `uv run` or invoke
`.\.venv\Scripts\python.exe` directly.

Check the installed libraries:

```powershell
uv run python -c "import mujoco, numpy, matplotlib; print('MuJoCo:', mujoco.__version__); print('NumPy:', numpy.__version__); print('Matplotlib:', matplotlib.__version__)"
```

Open the MuJoCo viewer:

```powershell
uv run python -m mujoco.viewer
```

## Robot model

The [base-6dof arm](model/README.md) is defined in
[`model/base-6dof.urdf`](model/base-6dof.urdf), with six revolute joint axes
in the order Z, X, X, Z, X, Z. J4 and J6 are longitudinal roll joints.

```powershell
uv run python -m mujoco.viewer --mjcf=model/base-6dof.urdf
```

The numbered `model/1-base-6dof.xml` through `model/9-base-6dof.xml` files are
self-contained MJCF scenes. Models 1-3 preserve the corrected primary URDF joint origins and axes.
Models 4-9 retain all six axes and joint ranges, shorten L2 to 90 mm, and extend
L3 to 270 mm. L2 remains a physical J3/J4 coupling; no joint is removed.
All joints use one reducer each; offset drives use local parallel-shaft input belts.
No belt crosses an intermediate joint. Motor/reducer mass estimates are listed;
structural COMs/inertias follow the modeled edge volumes. Models 07-09 turn
the J4 frame by 90 degrees: L2 is perpendicular to the motor/reducer axle, and
L3 follows that axle. The reference `model/base-6dof-perpendicular-j4.urdf`
records the right-angle family. Previous axis definitions are archived under
`references/previous-axes/`. These are layout studies; full-travel clearance,
load ratings, bearings and fasteners still need detailed design.

## Regenerate and verify

```powershell
uv run python -m armsim.generate_models
uv run python -m tools.audit_tool_dof
uv run python -m unittest discover -s tests -v
```

The generator reads `model/base-6dof.urdf` and the layout definitions in
`armsim/catalog.py`; rerunning it replaces the numbered XML files and
`model/layouts.json`. Tests independently check URDF forward kinematics,
fixed/moving mounts, mid-height case ports, spatial edge-only structures,
planar output surrounds, single coaxial reducers, flange clearance, actual
structural COMs, reflected inertia, torque scaling and limits,
and stable servo motion across all nine models. Short-L2 checks also verify the
revised forward kinematics, mass distribution, COMs and inertias.

To run the Qt/OpenGL integration checks and record local performance:

```powershell
uv run python view_models.py --smoke-test --screenshot .artifacts/qt-viewport.png
```

This checks resizing, device-pixel resolution, model switching, numeric angle
entry, matched comparison poses, deselection, camera drag and component picking, labels, simulation, pause, idle
redraw behavior, and export. Local timing results are written to
`.artifacts/qt-performance.json`.

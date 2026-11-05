# Mechanical and kinematic references

The source audit uses current model definitions from
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), pinned to
commit `b0655057a9df2abe39ea7fc989914ba6f248301c`:

- [Universal Robots UR5e](https://github.com/google-deepmind/mujoco_menagerie/tree/b0655057a9df2abe39ea7fc989914ba6f248301c/universal_robots_ur5e)
- [UFactory Lite6](https://github.com/google-deepmind/mujoco_menagerie/tree/b0655057a9df2abe39ea7fc989914ba6f248301c/ufactory_lite6)
- [Fanuc CRX5iA](https://github.com/google-deepmind/mujoco_menagerie/tree/b0655057a9df2abe39ea7fc989914ba6f248301c/fanuc_crx5ia)

XML definitions, READMEs and their licenses are stored under `menagerie/`.
`menagerie/sources.json` records pinned URLs and SHA-256 checksums. Mesh assets
are not downloaded; these are research fixtures rather than standalone robot
scenes. No external source code is executed. The kinematics audit removes
file-backed visual assets and compiles only the frame/joint definitions.

```powershell
uv run python tools/fetch_references.py
uv run python tools/audit_references.py
```

`fetch_references.py` reuses the saved revision by default. `--latest` explicitly
refreshes the pin from the repository's current main branch. The independent
audit output is `kinematic-audit.json`.

## What the references establish

The robot descriptions supply real joint/body transforms, local axes, link
offsets and inertial conventions. They do not reveal every internal motor,
reducer, bearing or fastener connection. They therefore inform the kinematic
comparison, rather than serve as proof that a newly drawn mechanical assembly
will carry a particular load.

At each source model's zero pose, the audited wrist axes in world coordinates
are approximately:

| Model | J4 | J5 | J6 |
| --- | --- | --- | --- |
| UR5e | -Y | -Z | -Y |
| Lite6 | -Z | +Y | -Z |
| CRX5iA | -X | -Y | -X |

These vectors belong to each source's own base frame. They show why axes must
be interpreted with their body transforms, and why equal J4/J6 axis labels
are not inherently a file error. ArmSim's corrected J4 is a longitudinal L3
roll joint (local Z), perpendicular to J3=X. J6 also rolls its tool link about
local Z, with J5 providing the intervening bend. Earlier transverse-J4 and
transverse-J6 URDFs are archived in `previous-axes/` for reference and excluded
from the active catalog.

## Reducer mounting reference

[Harmonic Drive's NC rotary-table application](https://legacy.harmonicdrive.net/reference/applicationhandbook/display.php?id=mt_mcnrt)
describes a circular spline fixed to the housing, a bearing-supported moving
output flange, and a parallel motor driving the input wave generator through
a toothed belt. This is the mechanical relationship used in ArmSim's joint
modules: one joint-coaxial reducer, a supported fixed housing, a moving output
flange, and optional input-side belt drive.

[Gates' synchronous belt design manual](https://www.gates.com/content/dam/documents-library/catalogs/powergrip-gt3-drive-design-manual-en.pdf)
is the alignment/design reference for the parallel-shaft input belts. Pulley
centers share a plane and remain supported by one parent link. Belt ratings,
pretension, shaft deflection, bearing selection and fastener sizing have not
been inferred from a visual resemblance to the reference robots.

The generated hollow frames are original study geometry. Manufacturer meshes
have not been copied into the custom arm or represented as newly designed CAD.

"""Shared layout descriptions. All dimensions are in SI units."""

from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "model"
URDF_PATH = MODEL_DIR / "base-6dof.urdf"
AXES = np.array([[0, 0, 1], [1, 0, 0], [1, 0, 0], [0, 0, 1], [1, 0, 0], [0, 0, 1]], dtype=float)
AXIS_NAMES = ("Z", "X", "X", "Z", "X", "Z")
LINKS = ("base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "tool_flange_link")
PARENTS = LINKS[:-1]
OFFSETS = np.array([0.10, 0.12, 0.32, 0.28, 0.08, 0.07])
SHORT_L2_M = 0.09
EXTENDED_L3_M = 0.36 - SHORT_L2_M
BENT_POSE = np.radians([-30.,50.,-100.,0.,-10.,0.])
MOTOR_COLOR = (0.16, 0.76, 0.62, 1)
GEAR_COLOR = (1.0, 0.65, 0.22, 1)
LINK_COLOR = (0.20, 0.43, 0.76, 1)
BELT_COLOR = (0.72, 0.48, 0.96, 1)
WRIST_PROFILES = {
    "reference": ("Roll-bend-roll wrist: J4 Z / J5 X / J6 Z", "Z", "Z"),
}
DEFAULT_WRIST = 'reference'


def axes_for(profile="reference"):
    axes = AXES.copy()
    for index, name in zip((3, 5), WRIST_PROFILES[profile][1:]):
        axes[index] = np.eye(3)["XYZ".index(name)]
    if abs(float(axes[2] @ axes[3])) > 1e-12:
        raise ValueError('J4 must be orthogonal to J3')
    if not np.allclose(axes[3], [0, 0, 1]):
        raise ValueError('J4 must roll L3 about its longitudinal local Z axis')
    if not np.allclose(axes[5], [0, 0, 1]):
        raise ValueError('J6 must roll the tool link about its longitudinal local Z axis')
    return axes


@dataclass(frozen=True)
class Layout:
    number: int
    title: str
    slug: str
    description: str
    drives: tuple[str, ...]
    ratios: tuple[float, ...]
    l2_length: float = 0.28
    l3_length: float = 0.08
    j4_mount: str = 'inline'
    wrist2_length: float = 0.07

    @property
    def short_l2(self) -> bool:
        return self.l2_length < 0.28

    @property
    def joint_offsets(self) -> np.ndarray:
        offsets = OFFSETS.copy()
        offsets[3:5] = (self.l2_length, self.l3_length)
        offsets[5] = self.wrist2_length
        return offsets

    @property
    def perpendicular_j4(self) -> bool:
        return self.j4_mount == 'perpendicular_axis'

    @property
    def j4_rotation(self) -> np.ndarray:
        return np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]]) if self.perpendicular_j4 else np.eye(3)

    def parent_axes(self, profile='reference') -> np.ndarray:
        axes=axes_for(profile)
        axes[3]=self.j4_rotation@axes[3]
        return axes

    @property
    def bent_pose(self) -> np.ndarray:
        """Comparable > posture with the same tool position and orientation."""
        pose=BENT_POSE.copy()
        if not self.perpendicular_j4:return pose
        upper,forearm=0.32,0.36
        horizontal=upper*np.sin(pose[1])+forearm*np.sin(pose[1]+pose[2])
        vertical=upper*np.cos(pose[1])+forearm*np.cos(pose[1]+pose[2])
        effective=np.hypot(self.l2_length,self.l3_length)
        elbow=-np.arccos(np.clip((horizontal**2+vertical**2-upper**2-effective**2)/(2*upper*effective),-1,1))
        shoulder=np.arctan2(horizontal,vertical)-np.arctan2(effective*np.sin(elbow),upper+effective*np.cos(elbow))
        pose[1]=shoulder
        pose[2]=elbow+np.arctan2(self.l3_length,self.l2_length)
        pose[4]=sum(BENT_POSE[[1,2,4]])-(pose[1]+pose[2]-np.pi/2)
        return pose

    @property
    def filename(self) -> str:
        return f"{self.number}-base-6dof.xml"

    @property
    def path(self) -> Path:
        return MODEL_DIR / self.filename

    def path_for(self, profile="reference") -> Path:
        return self.path if profile == "reference" else MODEL_DIR / f"{self.number}-base-6dof-{profile}.xml"

RATIOS = (80, 100, 80, 50, 50, 40)
COAXIAL = ('coaxial',)*6
LAYOUTS = (
    Layout(1, 'Long L2 / all motors at joints', 'long-all-coaxial',
           'L2: 280 mm. All motors share their reducer/joint shafts. J4 rolls L3; J6 rolls the tool. No belts.', COAXIAL, RATIOS),
    Layout(2, 'Long L2 / J3 belt, J2 below', 'long-j2-below',
           'L2: 280 mm. J3 has an in-link belt motor. J2 has a lower belt motor. J4 and J6 are coaxial. Identical to 03 except for the J2 drive.',
           ('coaxial','input_belt','input_belt','coaxial','coaxial','coaxial'), RATIOS),
    Layout(3, 'Long L2 / J3 belt, J2 at joint', 'long-j2-coaxial',
           'L2: 280 mm. Same J3 motor position and belt as 02; J2 motor is coaxial at its joint. J4 and J6 are coaxial. Only the J2 drive differs from 02.',
           ('coaxial','coaxial','input_belt','coaxial','coaxial','coaxial'), RATIOS),
    Layout(4, 'Short L2 / all motors at joints', 'short-all-coaxial',
           'L2: 90 mm; L3: 270 mm. All motors are coaxial. J4 housing uses the inline support; L3 stays in line with L2.', COAXIAL, RATIOS, SHORT_L2_M, EXTENDED_L3_M),
    Layout(5, 'Short L2 / J3+J5 belts, J2 below', 'short-j2-below',
           'L2: 90 mm; L3: 270 mm. In-link belt motors at J3/J5, lower belt motor at J2. Same offsets as 06. J4/J6 are coaxial; inline J4 support.',
           ('coaxial','input_belt','input_belt','coaxial','input_belt','coaxial'), RATIOS, SHORT_L2_M, EXTENDED_L3_M),
    Layout(6, 'Short L2 / J3+J5 belts, J2 at joint', 'short-j2-coaxial',
           'L2: 90 mm; L3: 270 mm. Same J3/J5 motors and belts as 05; J2 motor is coaxial. J4/J6 are coaxial; inline J4 support.',
           ('coaxial','coaxial','input_belt','coaxial','input_belt','coaxial'), RATIOS, SHORT_L2_M, EXTENDED_L3_M),
)
LAYOUTS += tuple(replace(base, number=base.number+3,
                        title='Short L2 / J4 axle at 90° / '+('all at joints','J2 below','J2 at joint')[index],
                        slug=base.slug+'-perpendicular-j4', j4_mount='perpendicular_axis',
                        description='L2: 90 mm; L3: 270 mm. L2 is perpendicular to the coaxial J4 motor/reducer axle. L3 follows that axle and rolls about it. '+
                        ('All motors are coaxial.', 'J3/J5 belt motors; J2 below. Same placements as 09 except J2.',
                         'Same J3/J5 belt motors as 08; J2 coaxial.')[index])
                 for index,base in enumerate(LAYOUTS[3:6]))


def layout_by_number(number: int) -> Layout:
    return next(layout for layout in LAYOUTS if layout.number == number)

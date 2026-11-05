"""Common joint-state basis for comparing arms with different dimensions.

Reach bounds constrain the arm's maximum extension, not a Cartesian workspace
shell. The suite contains 24 named reference states and 984 deterministic
six-dimensional Halton states. Named extension tests resolve to each design's
fully straightened posture; the other joint angles are identical for every arm.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Protocol

import numpy as np


DEFAULT_POSE_COUNT = 1008
REFERENCE_POSE_COUNT = 24
SHOULDER_ORIGIN_M = np.array([0.0, 0.0, 0.22])
SAMPLING_DESCRIPTION = (
    "24 named joint-state references (zero, signed single-axis quarter turns, "
    "six fully extended world directions, combined folded/wrist states) plus "
    "984 deterministic Halton states across six joint angles in [-pi, pi). "
    "Joint states are common to every design; fully extended tests resolve "
    "analytically for each design. Reach bounds constrain maximum shoulder-to-TCP "
    "extension, not a workspace shell. Global collision and joint-stop checks disabled."
)


class PoseRequirement(Protocol):
    reach_lower_mm: float
    reach_upper_mm: float


@dataclass(frozen=True)
class Pose:
    pose_id: str
    pose_kind: str
    reference_name: str
    joint_positions_rad: np.ndarray | None
    hold_weight: float
    extension_direction: str = ""


def rotation_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Convert an orthogonal 3x3 matrix to a canonical unit WXYZ quaternion."""
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = np.sqrt(1.0 + trace) * 2.0
        result = np.array([
            scale / 4.0,
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
        ])
    else:
        axis = int(np.argmax(np.diag(matrix)))
        second, third = (axis + 1) % 3, (axis + 2) % 3
        scale = np.sqrt(1.0 + matrix[axis, axis] - matrix[second, second]
                        - matrix[third, third]) * 2.0
        result = np.empty(4)
        result[0] = (matrix[third, second] - matrix[second, third]) / scale
        result[axis + 1] = scale / 4.0
        result[second + 1] = (matrix[second, axis] + matrix[axis, second]) / scale
        result[third + 1] = (matrix[third, axis] + matrix[axis, third]) / scale
    result /= np.linalg.norm(result)
    nonzero = np.flatnonzero(np.abs(result) > 1e-12)
    if len(nonzero) and result[nonzero[0]] < 0:
        result *= -1
    result[np.abs(result) < 1e-15] = 0.0
    return result


def cube_orientations() -> tuple[np.ndarray, ...]:
    """Six tool-axis directions, each with four quarter-turn tool rolls."""
    rotations = []
    directions = (*np.eye(3), *(-np.eye(3)))
    for tool_axis in directions:
        reference = np.eye(3)[int(np.argmin(np.abs(tool_axis)))]
        frame = np.column_stack((reference, np.cross(tool_axis, reference), tool_axis))
        for quarter_turn in range(4):
            cosine = (1, 0, -1, 0)[quarter_turn]
            sine = (0, 1, 0, -1)[quarter_turn]
            roll = np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])
            rotations.append(frame @ roll)
    return tuple(rotations)


def _halton(index: int, base: int) -> float:
    result, factor = 0., 1./base
    while index:
        index, digit = divmod(index, base)
        result += digit*factor
        factor /= base
    return result


def _references():
    yield "zero", np.zeros(6), ""
    for joint in range(6):
        for sign in (-1, 1):
            q = np.zeros(6)
            q[joint] = sign*np.pi/2
            yield f"J{joint+1}_{'minus' if sign < 0 else 'plus'}_90_deg", q, ""
    for direction in ("+X", "-X", "+Y", "-Y", "+Z", "-Z"):
        yield f"fully_extended_{direction}", None, direction
    for name, degrees in (
        ("positive_wrist_quarter_turns", (0, 90, 0, 90, 90, 90)),
        ("negative_wrist_quarter_turns", (0, -90, 0, -90, -90, -90)),
        ("folded_elbow_positive", (0, -90, 180, 0, 90, 0)),
        ("folded_elbow_negative", (0, 90, -180, 0, -90, 0)),
        ("mixed_pitch_and_roll", (90, 45, -90, 90, -90, -90)),
    ):
        yield name, np.radians(degrees), ""


def generate_poses(requirement: PoseRequirement, limit: int | None = None) -> list[Pose]:
    """Build a fixed configuration basis; dimensions never change the sample set."""
    if limit is not None and (isinstance(limit, bool) or not isinstance(limit, int) or limit < 1):
        raise ValueError("Pose limit must be a positive integer.")
    lower, upper = requirement.reach_lower_mm, requirement.reach_upper_mm
    if not np.isfinite([lower, upper]).all() or lower < 0 or lower > upper:
        raise ValueError("Reach bounds must be finite, nonnegative and ordered.")
    poses = []
    for name, q, direction in _references():
        poses.append(Pose(f"pose_{len(poses)+1:04d}",
                          "fully_extended" if direction else "joint_reference",
                          name, q, 0., direction))
    assert len(poses) == REFERENCE_POSE_COUNT
    for index in range(1, DEFAULT_POSE_COUNT-REFERENCE_POSE_COUNT+1):
        unit = np.array([_halton(index, base) for base in (2, 3, 5, 7, 11, 13)])
        poses.append(Pose(f"pose_{len(poses)+1:04d}", "halton_joint_state",
                          f"halton_{index:04d}", 2*np.pi*(unit-.5), 0.))
    if limit is not None and limit < len(poses):
        poses = [poses[index] for index in np.linspace(0, len(poses)-1, limit, dtype=int)]
    return [replace(pose, hold_weight=1./len(poses)) for pose in poses]


def resolve_joint_positions(pose, candidate):
    """Resolve named extension tests; return None for a legacy Cartesian target."""
    get = (lambda name, default=None: pose.get(name, default)) if isinstance(pose, dict) else (
        lambda name, default=None: getattr(pose, name, default))
    direction = get("extension_direction", "")
    if direction:
        from .reach import maximum_reach_pose
        q = np.asarray(maximum_reach_pose(candidate), dtype=float).copy()
        yaw, shoulder = {
            "+X": (np.pi/2, np.pi/2), "-X": (-np.pi/2, np.pi/2),
            "+Y": (0., -np.pi/2), "-Y": (0., np.pi/2),
            "+Z": (0., 0.), "-Z": (0., np.pi),
        }[direction]
        q[:2] = yaw, shoulder
        return q
    q = get("joint_positions_rad")
    if q is None:
        return None
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError("A joint-state reference needs six finite angles.")
    return q.copy()


POSE_COLUMNS = (
    "pose_id", "pose_kind", "reference_name", "extension_direction",
    *(f"q{j}_rad" for j in range(1, 7)), "hold_weight",
)


def pose_rows(poses: list[Pose]):
    """Named extension angles are blank until the candidate dimensions are known."""
    for pose in poses:
        q = pose.joint_positions_rad if pose.joint_positions_rad is not None else ("",)*6
        yield dict(zip(POSE_COLUMNS, (
            pose.pose_id, pose.pose_kind, pose.reference_name, pose.extension_direction,
            *q, pose.hold_weight,
        )))


def write_poses(path: str | Path, poses: list[Pose]) -> None:
    """Write a human-readable CSV, refusing to overwrite an existing target set."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=POSE_COLUMNS)
        writer.writeheader()
        writer.writerows(pose_rows(poses))

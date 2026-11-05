"""Imported MJCF descriptors, separate from the nine generated layout presets."""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
from pathlib import Path

import mujoco
import numpy as np

from .catalog import BENT_POSE, LAYOUTS, MODEL_DIR


@dataclass(frozen=True)
class ImportedLayout:
    source_path: Path
    number: int = 0
    l1_length: float = .32
    l2_length: float = .28
    l3_length: float = .08
    j4_mount: str = "custom"
    ratios: tuple[float, ...] = (1.,) * 6
    drives: tuple[str, ...] = ("unspecified",) * 6
    ratios_known: bool = False
    metadata: dict | None = None

    @property
    def model_kind(self):
        return "optimized" if self.metadata is not None else "imported"

    @property
    def title(self):
        if self.metadata is not None:
            return self.metadata.get("title", f"Optimized arm · {self.source_path.stem}")
        return f"Imported · {self.source_path.stem}"

    @property
    def description(self):
        if self.metadata is not None:
            return self.metadata.get("description", "Optimized ArmSim model with catalog drivetrain and torque-limited servos.")
        return (f"Imported MJCF: {self.source_path.name}. Pose exploration uses the "
                "file's geometry, mass properties and joint axes. No actuator "
                "controller is assumed for imported files.")

    @property
    def path(self):
        return self.source_path

    @property
    def filename(self):
        return self.source_path.name

    @property
    def slug(self):
        return f"imported-{self.source_path.stem}"

    def path_for(self, profile="reference"):
        return self.source_path

    @property
    def short_l2(self):
        return self.l2_length < .28

    @property
    def perpendicular_j4(self):
        return self.j4_mount == "perpendicular_axis"

    @property
    def bent_pose(self):
        if self.metadata is not None:
            return np.asarray(self.metadata["inspect_pose_radians"], dtype=float).copy()
        pose = BENT_POSE.copy()
        if self.perpendicular_j4:
            # Rotate the elbow's effective forearm vector toward the same
            # chevron direction without changing any joint-axis definitions.
            pose[2] += np.arctan2(self.l3_length, self.l2_length)
            pose[4] += np.pi/2 - np.arctan2(self.l3_length, self.l2_length)
        return pose


def custom_text(model, name):
    """Read an embedded MJCF custom text from the compiled model."""
    index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, name)
    if index < 0:
        return None
    start, size = int(model.text_adr[index]), int(model.text_size[index])
    return model.text_data[start:start+size].split(b"\0", 1)[0].decode("utf-8")


def custom_numeric(model, name):
    index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_NUMERIC, name)
    if index < 0:
        return None
    start, size = int(model.numeric_adr[index]), int(model.numeric_size[index])
    return model.numeric_data[start:start+size].copy()


def export_metadata(model):
    """Read the explicit versioned export contract; generic files stay passive."""
    version = custom_text(model, "armsim_export_version")
    kind = custom_text(model, "armsim_model_kind")
    if version is None and kind is None:
        return None
    if version != "1" or kind != "optimized":
        raise ValueError("Unsupported ArmSim export contract; expected optimized export version 1.")
    try:
        result = json.loads(custom_text(model, "armsim_metadata") or "")
        if not isinstance(result, dict):
            raise ValueError("metadata must be a JSON object")
        drives = result["drives"]
        if not isinstance(drives, list) or len(drives) != 6:
            raise ValueError("exactly six drivetrain records are required")
        policy = result.get("reducer_load_policy", "continuous")
        if policy not in {"continuous", "hold_and_repeated_peak"}:
            raise ValueError(f"unknown reducer load policy: {policy}")
        motor_policy = result.get("motor_load_policy", "continuous")
        if motor_policy not in {"continuous", "hold_and_peak_envelope"}:
            raise ValueError(f"unknown motor load policy: {motor_policy}")
        positive = ("ratio", "continuous_output_torque_Nm",
                    "max_reducer_input_speed_rpm", "motor_rated_torque_Nm")
        for index, drive in enumerate(drives, 1):
            if drive["joint"] != index:
                raise ValueError("drivetrain records must be ordered J1 through J6")
            for key in positive:
                value = float(drive[key])
                if not np.isfinite(value) or value <= 0:
                    raise ValueError(f"J{index} {key} must be positive and finite")
            rotor = float(drive["rotor_inertia_kg_m2"])
            if not np.isfinite(rotor) or rotor < 0:
                raise ValueError(f"J{index} rotor inertia must be nonnegative and finite")
            if drive.get("max_input_torque_Nm") is not None:
                value = float(drive["max_input_torque_Nm"])
                if not np.isfinite(value) or value <= 0:
                    raise ValueError(f"J{index} input torque must be positive and finite")
            from .drivetrain_limits import (joint_torque_limit, motor_speed_limit,
                                            reducer_speed_limit, reducer_torque_limit)
            motor_speed_limit(drive)
            reducer_speed_limit(drive)
            if drive.get("reducer_load_policy", "continuous") != policy:
                raise ValueError(f"J{index} reducer load policy differs from the model policy")
            if drive.get("motor_load_policy", "continuous") != motor_policy:
                raise ValueError(f"J{index} motor load policy differs from the model policy")
            for key, expected in (
                    ("instantaneous_reducer_output_torque_limit_Nm", reducer_torque_limit(drive)),
                    ("instantaneous_joint_torque_limit_Nm", joint_torque_limit(drive))):
                if key in drive or policy == "hold_and_repeated_peak":
                    if not np.isclose(float(drive[key]), expected, atol=1e-9, rtol=1e-9):
                        raise ValueError(f"J{index} {key} differs from its selected ratings")
        pose = np.asarray(result["inspect_pose_radians"], dtype=float)
        if pose.shape != (6,) or not np.isfinite(pose).all():
            raise ValueError("inspection pose must contain six finite angles")
        lengths = np.array([result["link_lengths_m"][name] for name in ("L1", "L2", "L3")], dtype=float)
        if not np.isfinite(lengths).all() or np.any(lengths <= 0):
            raise ValueError("link lengths must be positive and finite")
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"Invalid ArmSim export metadata: {exc}") from exc
    return result


def imported_layout(path: str | Path, model=None):
    """Describe a file; compiled-model details are filled when it is loaded."""
    layout = ImportedLayout(Path(path).resolve())
    if model is None:
        return layout
    metadata = export_metadata(model)
    joint_bodies = model.jnt_bodyid
    lengths = tuple(float(np.linalg.norm(model.body_pos[joint_bodies[j]]))
                    for j in (2, 3, 4))
    rotation_flat = np.empty(9)
    mujoco.mju_quat2Mat(rotation_flat, model.body_quat[joint_bodies[3]])
    axis = rotation_flat.reshape(3, 3) @ model.jnt_axis[3]
    alignment = abs(float(axis @ np.array([0., 0., 1.])))
    mount = "inline" if alignment > 1-1e-6 else "perpendicular_axis" if alignment < 1e-6 else "custom"
    drives = []
    for i in range(1, 7):
        motor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"motor_{i}")
        reducer = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"reducer_{i}")
        if reducer < 0:
            reducer = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"gearbox_{i}")
        if motor < 0 or reducer < 0 or model.body_parentid[motor] != model.body_parentid[reducer]:
            drives.append("unspecified")
            continue
        mujoco.mju_quat2Mat(rotation_flat, model.body_quat[reducer])
        shaft = rotation_flat.reshape(3, 3)[:, 2]
        delta = model.body_pos[motor] - model.body_pos[reducer]
        offset = delta - shaft * float(delta @ shaft)
        drives.append("coaxial" if np.linalg.norm(offset) < 1e-7 else "input_belt")
    if metadata is not None:
        expected = [metadata["link_lengths_m"][name] for name in ("L1", "L2", "L3")]
        if not np.allclose(lengths, expected, atol=1e-10, rtol=1e-9):
            raise ValueError("ArmSim export link metadata does not match the compiled model.")
        ratios = tuple(float(drive["ratio"]) for drive in metadata["drives"])
        drives = tuple(drive["drive"] for drive in metadata["drives"])
    else:
        ratios = layout.ratios
    return replace(layout, l1_length=lengths[0], l2_length=lengths[1],
                   l3_length=lengths[2], j4_mount=mount, drives=tuple(drives),
                   ratios=ratios, ratios_known=metadata is not None, metadata=metadata)


def discover_imported_models(model_dir: str | Path = MODEL_DIR):
    """List extra XML files without rewriting them or loading invalid files."""
    directory = Path(model_dir)
    if not directory.is_dir():
        return ()
    preset_names = {layout.filename.casefold() for layout in LAYOUTS}
    return tuple(imported_layout(path) for path in sorted(directory.glob("*.xml"),
                 key=lambda path: path.name.casefold())
                 if path.name.casefold() not in preset_names)

"""Shared simulation state, independent of the desktop controls."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from armsim.catalog import URDF_PATH, Layout, axes_for
from armsim.drivetrain_limits import (joint_torque_limit, motor_speed_limit,
                                      reducer_speed_limit, motor_torque_at_speed)


def _validate_export_servos(model, metadata):
    """Enable file-model dynamics only for the explicit, checked servo contract."""
    from .model_catalog import custom_numeric

    ratios = np.array([drive["ratio"] for drive in metadata["drives"]], dtype=float)
    rotors = np.array([drive["rotor_inertia_kg_m2"] for drive in metadata["drives"]], dtype=float)
    if model.nu != 6:
        raise ValueError("ArmSim optimized exports require six position servo actuators.")
    if (not np.all(model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT)
            or not np.array_equal(model.actuator_trnid[:, 0], np.arange(6))
            or not np.allclose(model.actuator_gear[:, 0], ratios)
            or not np.allclose(model.actuator_gear[:, 1:], 0)):
        raise ValueError("ArmSim export servo joint mappings or gear ratios do not match its metadata.")
    gain, bias = model.actuator_gainprm, model.actuator_biasprm
    if (not np.all(model.actuator_gaintype == mujoco.mjtGain.mjGAIN_FIXED)
            or not np.all(model.actuator_biastype == mujoco.mjtBias.mjBIAS_AFFINE)
            or not np.all(model.actuator_dyntype == mujoco.mjtDyn.mjDYN_NONE)
            or not np.isfinite(gain).all() or not np.isfinite(bias).all()
            or not np.all(gain[:, 0] > 0) or not np.allclose(gain[:, 1:], 0)
            or not np.allclose(bias[:, 0], 0)
            or not np.allclose(bias[:, 1], -gain[:, 0])
            or not np.all(bias[:, 2] <= 0) or not np.allclose(bias[:, 3:], 0)):
        raise ValueError("ArmSim export actuators must use the supported position-servo model.")
    instantaneous_limits = np.array([joint_torque_limit(drive)/ratio
        for drive, ratio in zip(metadata["drives"], ratios)], dtype=float)
    ranges = model.actuator_forcerange
    if (not np.all(model.actuator_forcelimited)
            or not np.isfinite(ranges).all()
            or not np.all(ranges[:, 1] > 0)
            or not np.allclose(ranges[:, 0], -ranges[:, 1])
            or np.any(ranges[:, 1] > instantaneous_limits+1e-9)):
        raise ValueError("ArmSim export servo forces must respect the selected instantaneous drivetrain limits.")
    if not np.allclose(model.dof_armature, ratios**2*rotors, atol=1e-12, rtol=1e-9):
        raise ValueError("ArmSim export reflected rotor inertia does not match the selected motors.")
    numeric_ratios = custom_numeric(model, "motor_ratios")
    speed_limits = custom_numeric(model, "joint_speed_limits")
    if (numeric_ratios is None or numeric_ratios.shape != (6,)
            or not np.allclose(numeric_ratios, ratios)):
        raise ValueError("ArmSim export needs matching motor_ratios metadata.")
    hardware_speed = np.array([min(motor_speed_limit(drive), reducer_speed_limit(drive))
                               for drive in metadata["drives"]])*2*np.pi/60/ratios
    if (speed_limits is None or speed_limits.shape != (6,)
            or not np.isfinite(speed_limits).all() or np.any(speed_limits <= 0)
            or np.any(speed_limits > hardware_speed+1e-9)):
        raise ValueError("ArmSim export needs valid hardware-limited joint_speed_limits.")
    return ratios, speed_limits


class Simulation:
    def __init__(self, layout: Layout, wrist_profile="reference"):
        self.layout = layout
        self.is_external = False
        self.supports_dynamics = True
        self.ratios_known = True
        self.export_metadata = None
        self._base_force_limits = None
        self.wrist_profile = wrist_profile
        self.axes = axes_for(wrist_profile)
        self.axis_names = tuple("XYZ"[int(np.argmax(np.abs(axis)))] for axis in self.axes)
        self.model = mujoco.MjModel.from_xml_path(str(layout.path_for(wrist_profile)))
        self.data = mujoco.MjData(self.model)
        self.ratios = np.array(layout.ratios, dtype=float)
        self.limits = self.model.jnt_range.copy()
        source = ET.parse(URDF_PATH).getroot()
        self.speed_limits = np.array([float(j.find("limit").attrib["velocity"])
                                      for j in source.findall("joint") if j.attrib["type"] == "revolute"])
        self.targets = layout.bent_pose
        self.command = layout.bent_pose
        self.dynamic = False
        self.gravity_assist = False
        self.demo = False
        self.demo_time = 0.0
        self.reset(layout.bent_pose)

    @classmethod
    def from_path(cls, path: str | Path):
        """Load a six-axis MJCF, enabling its validated ArmSim export servos.

        Generic files remain pose-only. A versioned optimized export contains
        its own checked drivetrain and controller metadata; nothing is guessed.
        """
        from .model_catalog import imported_layout

        source = Path(path).resolve()
        if not source.is_file():
            raise ValueError(f"MJCF file not found: {source}")
        try:
            model = mujoco.MjModel.from_xml_path(str(source))
        except (ValueError, OSError) as exc:
            raise ValueError(f"Cannot load {source.name}: {exc}") from exc
        if (model.njnt, model.nq, model.nv) != (6, 6, 6) or not np.all(
                model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError("Imported models must have exactly six scalar hinge joints.")
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tool_frame") < 0:
            raise ValueError("Imported arm models need a site named 'tool_frame' at the tool.")
        for joint in range(1, 6):
            parent = int(model.body_parentid[model.jnt_bodyid[joint]])
            expected = int(model.jnt_bodyid[joint-1])
            while parent > 0 and parent != expected:
                parent = int(model.body_parentid[parent])
            if parent != expected:
                raise ValueError("Imported models must describe one serial six-axis arm.")
        self = cls.__new__(cls)
        self.model, self.data = model, mujoco.MjData(model)
        self.layout = imported_layout(source, model)
        self.export_metadata = self.layout.metadata
        self.wrist_profile = "reference"
        self.is_external = True
        self.supports_dynamics = False
        self.ratios_known = False
        self.axes = model.jnt_axis.copy()
        self.axis_names = []
        for axis in self.axes:
            dominant = int(np.argmax(np.abs(axis)))
            if np.allclose(np.abs(axis), np.eye(3)[dominant], atol=1e-8):
                self.axis_names.append(("−" if axis[dominant] < 0 else "") + "XYZ"[dominant])
            else:
                self.axis_names.append("(" + ", ".join(f"{value:.2g}" for value in axis) + ")")
        self.axis_names = tuple(self.axis_names)
        self.ratios = np.ones(6)
        # Unlimited joints remain unlimited in the compiled physics model.
        # Sliders use one full turn as their display interval.
        self.limits = np.where(model.jnt_limited[:, None], model.jnt_range,
                               np.tile((-np.pi, np.pi), (6, 1)))
        self.speed_limits = np.ones(6)
        self._base_force_limits = None
        if self.export_metadata is not None:
            self.ratios, self.speed_limits = _validate_export_servos(model, self.export_metadata)
            self._base_force_limits = model.actuator_forcerange.copy()
            self.supports_dynamics = self.ratios_known = True
        self.targets = self.layout.bent_pose
        self.command = self.targets.copy()
        self.dynamic = self.gravity_assist = self.demo = False
        self.demo_time = 0.
        self.reset(self.layout.bent_pose)
        return self

    def _pose(self, pose):
        values = np.asarray(pose, dtype=float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError("A pose must contain six finite joint angles.")
        values = values.copy()
        limited = self.model.jnt_limited.astype(bool)
        values[limited] = np.clip(values[limited], self.limits[limited, 0], self.limits[limited, 1])
        return values

    def _set_controls(self, angles):
        if self.supports_dynamics:
            self.data.ctrl[:] = angles * self.ratios

    def _apply_force_limits(self):
        """Apply the same assumed motor torque-speed envelope used in scoring."""
        if self.export_metadata is None:
            return
        drives = self.export_metadata["drives"]
        rpm = np.abs(self.data.qvel)*self.ratios*60/(2*np.pi)
        limits = np.array([motor_torque_at_speed(drive, speed) for drive, speed in zip(drives, rpm)])
        limits = np.minimum(limits, self._base_force_limits[:, 1])
        self.model.actuator_forcerange[:, 0] = -limits
        self.model.actuator_forcerange[:, 1] = limits

    def reset(self, pose):
        mujoco.mj_resetData(self.model, self.data)
        self.targets = self._pose(pose)
        self.command = self.targets.copy()
        self.data.qpos[:] = self.targets
        self._apply_force_limits()
        self._set_controls(self.targets)
        self.demo_time = 0.0
        mujoco.mj_forward(self.model, self.data)

    def set_target(self, joint: int, angle: float):
        if not np.isfinite(angle):
            raise ValueError("Joint target must be finite.")
        self.targets[joint] = np.clip(angle, *self.limits[joint]) if self.model.jnt_limited[joint] else angle

    def advance(self, seconds: float, running: bool = True):
        if self.dynamic and not self.supports_dynamics:
            self.dynamic = False
        if self.demo and running:
            self.demo_time += seconds
            amplitudes = np.array([0.50, 0.20, 0.25, 0.40, 0.20, 0.45])
            phases = np.arange(6) * 0.7
            self.targets[:] = self.layout.bent_pose + amplitudes * np.sin(0.65 * self.demo_time + phases)
        if not self.dynamic:
            self.data.qpos[:] = self.targets
            self.data.qvel[:] = 0
            self.data.qfrc_applied[:] = 0
            self._apply_force_limits()
            self._set_controls(self.targets)
            self.command[:] = self.targets
            mujoco.mj_forward(self.model, self.data)
        elif running:
            steps = min(50, max(1, round(seconds / self.model.opt.timestep)))
            for _ in range(steps):
                self.command += np.clip(self.targets - self.command,
                                        -self.speed_limits * self.model.opt.timestep,
                                        self.speed_limits * self.model.opt.timestep)
                self._set_controls(self.command)
                self._apply_force_limits()
                self.data.qfrc_applied[:] = 0
                if self.gravity_assist:
                    mujoco.mj_forward(self.model, self.data)
                    self.data.qfrc_applied[:] = self.data.qfrc_bias
                mujoco.mj_step(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)

    @property
    def output_torques(self):
        if not self.supports_dynamics:
            return np.zeros(6)
        return self.data.actuator_force * self.ratios

"""One approved base-mechanism generator for both scoring and export.

Optimization changes dimensions, catalog hardware and permitted placement
coordinates. The shared ArmSim mechanics builder retains the approved edge,
saddle and output-frame construction rules. Unsupported combinations raise a
geometry error instead of acquiring a different support topology at export.
"""

from __future__ import annotations

from dataclasses import replace
import json
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from armsim.catalog import BENT_POSE, LINKS, URDF_PATH, axes_for, layout_by_number
from armsim.drivetrain_limits import joint_torque_limit, reducer_torque_limit
from armsim.mechanics import build_mechanics
from armsim.mechanism_parameters import MechanismParameters, GeometryError
from armsim.assembly_validation import validate_local_assembly
from armsim.model_primitives import add, rpy_quaternion, values
from .structural_mass import STRUCTURAL_MASS_MODEL, candidate_from_model, structural_properties


GEOMETRY_CONTRACT = "approved-base-template-v8"


def _assets(root):
    asset = add(root, "asset")
    add(asset, "texture", name="floor_texture", type="2d", builtin="checker",
        rgb1=".13 .16 .21", rgb2=".18 .22 .28", width="256", height="256")
    add(asset, "material", name="floor_material", texture="floor_texture", texrepeat="8 8")
    for name, color in (
        ("arm_blue", (.20, .43, .76, 1)), ("base_gray", (.28, .32, .39, 1)),
        ("motor_teal", (.16, .76, .62, 1)), ("gearbox_amber", (1., .65, .22, 1)),
        ("shaft_metal", (.62, .67, .74, 1)), ("belt_purple", (.72, .48, .96, 1))):
        add(asset, "material", name=name, rgba=values(color), shininess=".45", specular=".35")


def mechanism_parameters(candidate, inputs):
    motors = [inputs.motor_by_id[name] for name in candidate.motor_ids]
    reducers = [inputs.reducer_by_id[name] for name in candidate.reducer_ids]
    return MechanismParameters(
        l1_length=candidate.lengths_m[0],
        motor_masses=tuple(m.mass_kg for m in motors),
        gear_masses=tuple(r.mass_kg for r in reducers),
        motor_radii=tuple(m.radius_m for m in motors),
        gear_radii=tuple(r.radius_m for r in reducers),
        motor_lengths=tuple(m.length_m for m in motors),
        gear_lengths=tuple(r.length_m for r in reducers),
        motor_offsets=tuple(candidate.motor_offsets_m),
    )


def _inspect_pose(candidate, layout):
    pose = BENT_POSE.copy()
    if layout.perpendicular_j4:
        phase = np.arctan2(candidate.lengths_m[2], candidate.lengths_m[1])
        pose[2] += phase
        pose[4] += np.pi/2-phase
    return pose


def build_template_model(candidate, inputs, payload_kg=0., *, actuated=False, requirement=None):
    """Compile the same parameterized base model used throughout the workflow."""
    if not math.isfinite(candidate.wrist2_length_m) or candidate.wrist2_length_m <= 0:
        raise GeometryError("Wrist2 length must be positive and finite")
    motors = [inputs.motor_by_id[name] for name in candidate.motor_ids]
    reducers = [inputs.reducer_by_id[name] for name in candidate.reducer_ids]
    if requirement is not None and hasattr(requirement, "allows_motor"):
        for number, motor in enumerate(motors, 1):
            if not requirement.allows_motor(number, motor.motor_id):
                raise GeometryError(f"Study motor selection forbids {motor.motor_id} at J{number}")
    for number, reducer in enumerate(reducers, 1):
        if reducer.allowed_joints and number not in reducer.allowed_joints:
            raise GeometryError(f"Reducer {reducer.reducer_id} is not allowed at J{number}; allowed joints: {reducer.allowed_joints}")
    ratios = np.array([r.reduction_ratio for r in reducers])
    layout = replace(layout_by_number(candidate.layout_number),
                     l2_length=candidate.lengths_m[1], l3_length=candidate.lengths_m[2],
                     wrist2_length=candidate.wrist2_length_m,
                     ratios=tuple(ratios))
    axes = axes_for()
    project = getattr(requirement, "project_id", "basis-evaluation")
    root = ET.Element("mujoco", model=f"Parameterized base | {project} | layout {layout.number}")
    root.append(ET.Comment("Approved base geometry is parameterized; unsupported mounts are rejected. Structural mass, COM and inertia follow the actual stock geometry."))
    add(root, "compiler", angle="radian", autolimits="true", inertiafromgeom="auto", fusestatic="false")
    add(root, "option", gravity="0 0 -9.81", timestep=".002", integrator="implicitfast")
    default = add(root, "default")
    add(default, "geom", contype="0", conaffinity="0")
    add(default, "joint", limited="false", damping="0", frictionloss="0")
    visual = add(root, "visual")
    add(visual, "global", offwidth="1600", offheight="1200")
    add(visual, "quality", shadowsize="1024", numslices="20", numstacks="8", numquads="2")
    add(visual, "headlight", ambient=".4 .4 .4", diffuse=".6 .6 .6")
    add(root, "statistic", center="0 0 .5", extent=str(max(1.4, sum(candidate.lengths_m)*1.6)))
    _assets(root)
    world = add(root, "worldbody")
    add(world, "light", pos="1 -2 3", dir="-.3 .5 -1", directional="true", diffuse=".8 .8 .8")
    add(world, "geom", name="ground", type="plane", size="2 2 .1", mass="0",
        material="floor_material", group="5", contype="0", conaffinity="0")
    add(world, "site", name="world_frame", pos="0 0 .001", size=".003", rgba="1 1 1 0")
    base = add(world, "body", name="base_link")
    bodies = {"base_link": base}
    source = ET.parse(URDF_PATH).getroot()
    output_joints = []
    index = 0
    for original in source.findall("joint"):
        parent = original.find("parent").get("link")
        child_name = original.find("child").get("link")
        origin = original.find("origin").attrib
        position = np.fromstring(origin["xyz"], sep=" ")
        rotation = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        name = original.get("name")
        if name in {"joint_3", "joint_4", "joint_5"}:
            position = np.array([0., 0., candidate.lengths_m[int(name[-1])-3]])
        if name == "joint_6":
            position = np.array([0., 0., candidate.wrist2_length_m])
        if name == "joint_4" and layout.perpendicular_j4:
            rotation = np.array([-np.pi/2, 0., 0.])
        child = add(bodies[parent], "body", name=child_name, pos=values(position),
                    quat=values(rpy_quaternion(rotation)))
        bodies[child_name] = child
        if original.get("type") == "fixed":
            continue
        output_joints.append(add(child, "joint", name=name, type="hinge", axis=values(axes[index])))
        add(child, "site", name=f"J{index+1}", size=".004", rgba="1 1 1 0")
        index += 1
    add(bodies["tool0"], "site", name="tool_frame", size=".007", rgba="1 .5 .1 1", group="4")
    try:
        with np.errstate(invalid="raise", divide="raise", over="raise"):
            drives = build_mechanics(layout, bodies, axes,
                                     parameters=mechanism_parameters(candidate, inputs))
    except GeometryError:
        raise
    except (ValueError, FloatingPointError) as exc:
        raise GeometryError(str(exc)) from exc
    # Keep the approved builder's actual stock and small-part masses. MuJoCo
    # infers their complete 3D mass properties; no independent mass budget can
    # make a long member artificially light or a short coupling arbitrarily heavy.
    for i in range(1, 7):
        bodies_case = next(body for body in root.iter("body") if body.get("name") == f"gearbox_{i}")
        bodies_case.set("name", f"reducer_{i}")
    payload = add(bodies["tool0"], "body", name="payload")
    if payload_kg:
        add(payload, "inertial", mass=payload_kg, pos="0 0 0",
            diaginertia=values([payload_kg*4e-5]*3))
    speed_limits = []
    load_policy = getattr(requirement, "reducer_load_policy", "continuous")
    motor_load_policy = getattr(requirement, "motor_load_policy", "continuous")
    for j, (drive, motor, reducer, joint) in enumerate(zip(drives, motors, reducers, output_joints)):
        ratio = ratios[j]
        armature = ratio**2*motor.evaluation_rotor_inertia_kg_m2
        if actuated:
            joint.set("armature", str(armature))
        speed_limits.append(min(motor.evaluation_speed_limit_rpm,
                                reducer.evaluation_input_speed_limit_rpm)*2*math.pi/(60*ratio))
        drive.update({
            "motor_id": motor.motor_id, "motor_name": motor.name,
            "reducer_id": reducer.reducer_id, "reducer_name": reducer.name,
            "requested_drive": layout.drives[j], "motor_radius_m": motor.radius_m,
            "gearbox_radius_m": reducer.radius_m, "motor_length_m": motor.length_m,
            "gearbox_length_m": reducer.length_m,
            "motor_rated_torque_Nm": motor.rated_torque_Nm,
            "motor_load_policy": motor_load_policy,
            "motor_peak_torque_Nm": motor.peak_torque_Nm,
            "motor_peak_duration_s": motor.peak_duration_s,
            "motor_peak_torque_speed_curve": motor.peak_torque_speed_curve,
            "motor_peak_curve_basis": motor.peak_curve_basis,
            "motor_operating_voltage_V": motor.operating_voltage_V,
            "motor_capacity_model": motor.capacity_model,
            "motor_rated_speed_rpm": motor.rated_speed_rpm,
            "motor_evaluation_speed_limit_rpm": motor.evaluation_speed_limit_rpm,
            "stall_torque_Nm": motor.stall_torque_Nm, "no_load_speed_rpm": motor.no_load_speed_rpm,
            "continuous_output_torque_Nm": reducer.continuous_output_torque_Nm,
            "peak_output_torque_Nm": reducer.peak_output_torque_Nm,
            "reducer_load_policy": load_policy,
            "max_input_torque_Nm": reducer.max_input_torque_Nm,
            "max_motor_speed_rpm": motor.max_speed_rpm,
            "max_reducer_input_speed_rpm": reducer.max_input_speed_rpm,
            "reducer_rated_input_speed_rpm": reducer.rated_input_speed_rpm,
            "reducer_evaluation_input_speed_limit_rpm": reducer.evaluation_input_speed_limit_rpm,
            "reducer_input_torque_limit_verified": reducer.max_input_torque_Nm is not None,
            "rotor_inertia_kg_m2": motor.evaluation_rotor_inertia_kg_m2,
            "published_rotor_inertia_kg_m2": motor.rotor_inertia_kg_m2,
            "rotor_inertia_basis": motor.rotor_inertia_basis,
            "motor_rated_voltage_V": motor.rated_voltage_V,
            "motor_rated_current_A": motor.rated_current_A,
            "motor_price_usd": motor.price_usd,
            "motor_source_url": motor.source_url,
            "reducer_allowed_joints": list(reducer.allowed_joints),
            "reflected_armature_kg_m2": armature,
            "geometry_contract": GEOMETRY_CONTRACT,
        })
        drive["instantaneous_reducer_output_torque_limit_Nm"] = reducer_torque_limit(drive)
        drive["instantaneous_joint_torque_limit_Nm"] = joint_torque_limit(drive)
    pose = _inspect_pose(candidate, layout)
    if actuated:
        actuators = add(root, "actuator")
        for j, (drive, ratio) in enumerate(zip(drives, ratios)):
            force = joint_torque_limit(drive)/ratio
            add(actuators, "position", name=f"M{j+1}_servo", joint=f"joint_{j+1}", gear=ratio,
                kp=(180, 240, 180, 70, 55, 40)[j]/ratio**2,
                kv=(20, 25, 18, 5, 4, 3)[j]/ratio**2,
                forcelimited="true", forcerange=values((-force, force)))
    sensors = add(root, "sensor")
    add(sensors, "framepos", name="tool_position", objtype="site", objname="tool_frame")
    add(sensors, "framequat", name="tool_orientation", objtype="site", objname="tool_frame")
    keys = add(root, "keyframe")
    zero = {"qpos": values(np.zeros(6))}
    inspect = {"qpos": values(pose)}
    if actuated:
        zero["ctrl"] = values(np.zeros(6))
        inspect["ctrl"] = values(pose*ratios)
    add(keys, "key", name="zero", **zero)
    add(keys, "key", name="inspect", **inspect)
    metadata = {
        "title": f"Optimized base {layout.number:02d} · {project}",
        "description": "A tuned approved ArmSim base mechanism; mounting topology is preserved.",
        "geometry_contract": GEOMETRY_CONTRACT, "base_layout_number": layout.number,
        "j4_mount": layout.j4_mount, "layout_number": layout.number,
        "link_lengths_m": dict(zip(("L1", "L2", "L3"), candidate.lengths_m)),
        "wrist2_length_m": candidate.wrist2_length_m,
        "wrist_to_tcp_offset_m": candidate.wrist2_length_m+.08,
        "inspect_pose_radians": pose.tolist(), "joint_axes": axes.tolist(),
        "drives": drives, "design": candidate.to_dict(),
        "visual_mass_model": STRUCTURAL_MASS_MODEL,
        "structural_mass_model": STRUCTURAL_MASS_MODEL,
        "transmission_model": "One joint-coaxial reducer with approved local input-drive topology; reflected rotor inertia counted once.",
        "reducer_load_policy": load_policy,
        "motor_load_policy": motor_load_policy,
        "motor_torque_basis": (
            "Selected transient torque-speed envelope with separate continuous holding checks; "
            "MJCF actuator force range is only a zero-speed upper bound. ArmSim playback "
            "interpolates the embedded speed-dependent envelope."
            if motor_load_policy == "hold_and_peak_envelope" else
            "Selected continuous motor model; unverified transient motor capacity is not used."),
        "thermal_duty_model": "Not simulated; repeated-peak clipping does not certify holding torque, duty cycle or lifetime.",
        "local_assembly_checks": True,
        "collision_checks": False, "joint_stop_checks": False,
    }
    custom = add(root, "custom")
    add(custom, "text", name="armsim_geometry_contract", data=GEOMETRY_CONTRACT)
    if actuated:
        add(custom, "text", name="armsim_export_version", data="1")
        add(custom, "text", name="armsim_model_kind", data="optimized")
        add(custom, "text", name="armsim_metadata", data=json.dumps(metadata, separators=(",", ":")))
        add(custom, "numeric", name="motor_ratios", data=values(ratios))
        add(custom, "numeric", name="joint_speed_limits", data=values(speed_limits))
    ET.indent(root, space="  ")
    xml = '<?xml version="1.0" encoding="utf-8"?>\n'+ET.tostring(root, encoding="unicode")+"\n"
    try:
        model = mujoco.MjModel.from_xml_string(xml)
    except ValueError as exc:
        raise GeometryError(f"Parameterized base could not compile: {exc}") from exc
    validate_local_assembly(model, drives)
    if actuated:
        metadata["design"] = candidate_from_model(candidate, model).to_dict()
        metadata["structural_bodies"] = structural_properties(model)
        metadata["total_mass_kg"] = float(model.body_mass.sum())
        root.find("./custom/text[@name='armsim_metadata']").set(
            "data", json.dumps(metadata, separators=(",", ":")))
        ET.indent(root, space="  ")
        xml = '<?xml version="1.0" encoding="utf-8"?>\n'+ET.tostring(root, encoding="unicode")+"\n"
        model = mujoco.MjModel.from_xml_string(xml)
    return model, xml

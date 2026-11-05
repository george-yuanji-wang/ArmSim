"""Headless rigid-body mechanics for the conceptual optimization study.

These models deliberately have no contacts, joint stops or GUI dependencies.
The score is a rating-limited *basis-suite* payload capacity, not a certified
capacity for arbitrary trajectories. Optional peak-envelope policies separate
continuous holding loads from speed-dependent instantaneous motion loads.
They do not certify transient duration, cycle-average loading or lifetime.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import product
import math

import mujoco
import numpy as np

from .model_factory import build_template_model, GeometryError, GEOMETRY_CONTRACT
from .poses import resolve_joint_positions, SAMPLING_DESCRIPTION
from .reach import extended_reach_m


SHOULDER_ORIGIN_M = np.array([0., 0., .22])
TOOL_OFFSET_M = .15
_PAIRS = tuple((i, j) for i in range(6) for j in range(i+1, 6))
_ACCEL_SIGNS = np.array(tuple(product((-1., 1.), repeat=6)))


def _rx(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def _rz(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def build_model(candidate, inputs, payload_kg=0., *, requirement=None):
    """Score the approved base mechanism with the same geometry as its export.

    Catalog case placements and approved mounts come from the shared base
    builder. Unsupported dimensions/placements raise GeometryError before a
    candidate can receive a mechanical score. Rotor spin remains handled once
    in the motor-side load calculation.
    """
    return build_template_model(candidate, inputs, payload_kg, actuated=False, requirement=requirement)


def _wrist_angles(rotation):
    """Canonical Z-X-Z Euler angles, with a stable singular-case convention."""
    middle = math.acos(float(np.clip(rotation[2, 2], -1., 1.)))
    if abs(math.sin(middle)) > 1e-9:
        return (math.atan2(rotation[0, 2], -rotation[1, 2]), middle,
                math.atan2(rotation[2, 0], rotation[2, 1]))
    summed = math.atan2(rotation[1, 0], rotation[0, 0])
    return 0., middle, summed if middle < math.pi/2 else -summed


def solve_pose(model, data, position_m, quaternion_wxyz, seed=None):
    """Analytic unrestricted IK for the actual inline/right-angle six-axis chain.

    The elbow branch is canonical by default. If a seed is supplied, select the
    closest equivalent analytic branch. No geometry or joint-stop constraints
    are imposed. Returning None always means a genuine position reach failure
    or a failed forward-kinematics verification, never a silently skipped pose.
    """
    rotation_flat = np.empty(9)
    quaternion = np.asarray(quaternion_wxyz, dtype=float)
    quaternion = quaternion/np.linalg.norm(quaternion)
    mujoco.mju_quat2Mat(rotation_flat, quaternion)
    desired = rotation_flat.reshape(3, 3)
    ids = model.jnt_bodyid
    tool_offset = (float(model.body_pos[ids[5], 2]) +
                   float(model.body("tool0").pos[2]))
    wrist = np.asarray(position_m)-tool_offset*desired[:, 2]-SHOULDER_ORIGIN_M
    upper = float(model.body_pos[ids[2], 2])
    l2 = float(model.body_pos[ids[3], 2])
    l3 = float(model.body_pos[ids[4], 2])
    mount_flat = np.empty(9)
    mujoco.mju_quat2Mat(mount_flat, model.body_quat[ids[3]])
    mount = mount_flat.reshape(3, 3)
    perpendicular = not np.allclose(mount, np.eye(3))
    forearm = math.hypot(l2, l3) if perpendicular else l2+l3
    phase = math.atan2(l3, l2) if perpendicular else 0.
    radius = float(np.hypot(wrist[0], wrist[1]))
    cosine = (float(wrist@wrist)-upper*upper-forearm*forearm)/(2*upper*forearm)
    if cosine < -1-1e-9 or cosine > 1+1e-9:
        return None
    elbow_size = math.acos(float(np.clip(cosine, -1, 1)))
    yaw0 = math.atan2(-wrist[0], wrist[1]) if radius > 1e-10 else (float(seed[0]) if seed is not None else 0.)
    candidates = []
    for yaw, plane_y in ((yaw0, radius), (yaw0+math.pi, -radius)):
        for elbow in (-elbow_size, elbow_size):
            shoulder = math.atan2(-plane_y, wrist[2])-math.atan2(
                forearm*math.sin(elbow), upper+forearm*math.cos(elbow))
            q3 = elbow+phase
            prefix = _rz(yaw) @ _rx(shoulder+q3) @ mount
            roll1, bend, roll2 = _wrist_angles(prefix.T @ desired)
            candidates.append(np.array([yaw, shoulder, q3, roll1, bend, roll2]))
            candidates.append(np.array([yaw, shoulder, q3, roll1+math.pi, -bend, roll2+math.pi]))
    if seed is not None:
        seed = np.asarray(seed)
        candidates = [seed+(q-seed+math.pi)%(2*math.pi)-math.pi for q in candidates]
        candidates.sort(key=lambda q: float(np.sum((q-seed)**2)))
    q = candidates[0]
    data.qpos[:] = q
    data.qvel[:] = 0
    data.qacc[:] = 0
    mujoco.mj_forward(model, data)
    site = model.site("tool_frame").id
    if (np.linalg.norm(data.site_xpos[site]-position_m) > 2e-7 or
            np.linalg.norm(data.site_xmat[site].reshape(3, 3)-desired) > 2e-7):
        return None
    return q.copy()


def load_basis(model, data, q):
    """Return G, M, and 21 coefficient vectors spanning velocity-quadratic load."""
    data.qpos[:] = q
    data.qvel[:] = 0
    data.qacc[:] = 0
    mujoco.mj_inverse(model, data)
    gravity = data.qfrc_inverse.copy()
    matrix = np.empty((6, 6))
    mujoco.mj_fullM(model, data, matrix)
    quadratic = np.empty((21, 6))
    for i in range(6):
        data.qvel[:] = 0
        data.qvel[i] = 1.
        # q and its mass matrix are unchanged throughout this local basis.
        mujoco.mj_inverseSkip(model, data, mujoco.mjtStage.mjSTAGE_POS, 1)
        quadratic[i] = data.qfrc_inverse-gravity
    for index, (i, j) in enumerate(_PAIRS, 6):
        data.qvel[:] = 0
        data.qvel[[i, j]] = 1.
        mujoco.mj_inverseSkip(model, data, mujoco.mjtStage.mjSTAGE_POS, 1)
        quadratic[index] = data.qfrc_inverse-gravity-quadratic[i]-quadratic[j]
    data.qvel[:] = 0
    return gravity, matrix, quadratic


def _velocity_cases(speed):
    speeds = np.broadcast_to(np.asarray(speed, dtype=float), (6,))
    if not np.isfinite(speeds).all() or np.any(speeds < 0):
        raise ValueError("Joint speed limits must be six finite nonnegative values or one scalar")
    velocities = [np.zeros(6)]
    for i in range(6):
        v = np.zeros(6)
        v[i] = speeds[i]
        velocities.append(v)
    for i, j in _PAIRS:
        for sign in (-1, 1):
            v = np.zeros(6)
            v[[i, j]] = [speeds[i], sign*speeds[j]]
            velocities.append(v)
    return np.array(velocities)


def _velocity_features(velocities):
    return np.column_stack([*(velocities[:, i]**2 for i in range(6)),
                            *(velocities[:, i]*velocities[:, j] for i, j in _PAIRS)])


def motor_torque_limits(motors, speed_rpm):
    """Continuous screening envelopes; no peak or missing rating is invented.

    ``rated_continuous`` assumes the manufacturer's rated torque is available
    from rest through the published rated speed, then refuses extrapolation.
    It is a declared operating-envelope assumption, not a measured thermal map.
    The legacy linear stall/no-load model remains available for prior inputs.
    """
    motors = tuple(motors)
    speeds = np.abs(np.asarray(speed_rpm, dtype=float))
    limits = np.empty(np.broadcast_shapes(speeds.shape, (len(motors),)))
    speeds = np.broadcast_to(speeds, limits.shape)
    for index, motor in enumerate(motors):
        if motor.capacity_model == "rated_continuous":
            torque = np.full(speeds[..., index].shape, motor.rated_torque_Nm)
        elif motor.capacity_model == "legacy_linear":
            if motor.stall_torque_Nm is None or motor.no_load_speed_rpm is None:
                raise ValueError(f"Legacy motor {motor.motor_id} needs stall and no-load ratings")
            torque = np.minimum(motor.rated_torque_Nm, motor.stall_torque_Nm*np.maximum(
                0., 1-speeds[..., index]/motor.no_load_speed_rpm))
        else:
            raise ValueError(f"Unknown motor capacity model: {motor.capacity_model}")
        limits[..., index] = np.where(
            speeds[..., index] <= motor.evaluation_speed_limit_rpm+1e-9, torque, 0.)
    return limits


def motor_peak_torque_limits(motors, speed_rpm):
    """Speed-aware transient envelopes, conditional on their recorded evidence.

    A scalar advertised peak is never extended across the speed range. Curves
    are supplied at the chosen operating voltage; interpolation is linear and
    there is no extrapolation beyond either their endpoints or the supported
    motor speed. Missing duration is reported separately, not invented here.
    """
    motors = tuple(motors)
    speeds = np.abs(np.asarray(speed_rpm, dtype=float))
    limits = np.empty(np.broadcast_shapes(speeds.shape, (len(motors),)))
    speeds = np.broadcast_to(speeds, limits.shape)
    for index, motor in enumerate(motors):
        points = getattr(motor, "peak_torque_speed_curve", ())
        peak = motor.peak_torque_Nm
        if not points or peak is None or peak <= 0:
            raise ValueError(f"Motor {motor.motor_id} needs a speed-aware curve and published peak torque for hold_and_peak_envelope")
        curve = np.asarray(points, dtype=float)
        torque = np.minimum(peak, np.interp(speeds[..., index], curve[:, 0],
                                          curve[:, 1], left=0., right=0.))
        limits[..., index] = np.where(
            speeds[..., index] <= motor.evaluation_speed_limit_rpm+1e-9, torque, 0.)
    return limits


def _payload_limit(intercept, slope, limit):
    """Intersection of |intercept+p*slope|<=limit over all cases and p>=0."""
    limits = np.broadcast_to(limit, intercept.shape)
    baseline_ok = np.abs(intercept) <= limits+1e-8
    if not np.all(baseline_ok):
        excess = np.abs(intercept)-limits
        flat = int(np.argmax(excess))
        return 0., np.unravel_index(flat, intercept.shape), False
    with np.errstate(divide="ignore", invalid="ignore"):
        upper = np.where(slope > 1e-12, (limits-intercept)/slope,
                         np.where(slope < -1e-12, (-limits-intercept)/slope, np.inf))
    flat = int(np.argmin(upper))
    return max(0., float(upper.flat[flat])), np.unravel_index(flat, intercept.shape), True


@dataclass
class Evaluation:
    pmax_kg: float
    jg: float | None
    total_mass_kg: float
    reachable_count: int
    pose_count: int
    feasible: bool
    per_pose_rows: list[dict] = field(default_factory=list)
    model_xml: str = ""
    summary: dict = field(default_factory=dict)
    basis_rows: list[dict] = field(default_factory=list)

    @property
    def reachable_fraction(self):
        return self.reachable_count/self.pose_count if self.pose_count else 0.


def _pose_value(pose, name):
    return pose[name] if isinstance(pose, dict) else getattr(pose, name)


def _optional_pose_value(pose, name, default=None):
    return pose.get(name, default) if isinstance(pose, dict) else getattr(pose, name, default)


def _reach_summary(candidate, requirement):
    extension_mm = 1000*extended_reach_m(candidate)
    lower, upper = (getattr(requirement, name, None) for name in
                    ("reach_lower_mm", "reach_upper_mm"))
    bounds_pass = (lower is None and upper is None) or (
        lower is not None and upper is not None and
        float(lower)-1e-6 <= extension_mm <= float(upper)+1e-6)
    return {"extended_reach_mm": extension_mm, "reach_bounds_pass": bool(bounds_pass),
            "reach_definition": "maximum unrestricted shoulder-to-TCP extension",
            "required_extended_reach_lower_mm": lower,
            "required_extended_reach_upper_mm": upper}


def _utilization(torque, capacity):
    """Absolute demand/rating, including zero-capacity speed failures."""
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(capacity > 0, np.abs(torque)/capacity,
                        np.where(np.abs(torque) < 1e-8, 0., np.inf))


def evaluate_candidate(candidate, inputs, poses, requirement, *, collect_basis=False):
    """Evaluate one candidate on the common configuration/load basis.

    Velocities are rest, six single-axis states and both relative signs of 15
    pairs. At each velocity state all 64 corners of the joint-acceleration box
    are checked. This spans acceleration extremes exactly for those velocity
    states; it does not claim to optimize the whole continuous velocity domain.
    """
    poses = list(poses)
    reach_summary = _reach_summary(candidate, requirement)
    try:
        model0, xml = build_model(candidate, inputs, 0., requirement=requirement)
        model1, _ = build_model(candidate, inputs, 1., requirement=requirement)
    except GeometryError as exc:
        rows = [{"pose_id": _pose_value(pose, "pose_id"), "reachable": False,
                 "pmax_kg": 0., "limitation": "unsupported_base_geometry"}
                for pose in poses]
        return Evaluation(0., None, 0., 0, len(poses), False, rows, "", {
            "geometry_valid": False, "geometry_contract": GEOMETRY_CONTRACT,
            "failure_reason": "unsupported_base_geometry", "geometry_error": str(exc),
            "geometry_errors": [str(exc)],
            **reach_summary,
            "collision_checks": False, "joint_stop_checks": False,
        })
    if not reach_summary["reach_bounds_pass"]:
        rows = [{"pose_id": _pose_value(pose, "pose_id"), "reachable": False,
                 "pmax_kg": 0., "limitation": "extended_reach_out_of_bounds"}
                for pose in poses]
        return Evaluation(0., None, float(np.sum(model0.body_mass)), 0, len(poses), False,
                          rows, xml, {
                              "geometry_valid": True, "geometry_contract": GEOMETRY_CONTRACT,
                              **reach_summary, "failure_reason": "extended_reach_out_of_bounds",
                              "collision_checks": False, "joint_stop_checks": False,
                          })
    data0, data1 = mujoco.MjData(model0), mujoco.MjData(model1)
    motors = [inputs.motor_by_id[name] for name in candidate.motor_ids]
    reducers = [inputs.reducer_by_id[name] for name in candidate.reducer_ids]
    ratios = np.array([r.reduction_ratio for r in reducers])
    rotors = np.array([m.evaluation_rotor_inertia_kg_m2 for m in motors])
    velocities = _velocity_cases(getattr(requirement, "joint_speeds_rad_s", requirement.joint_speed_rad_s))
    features = _velocity_features(velocities)
    accelerations = _ACCEL_SIGNS*requirement.joint_accel_rad_s2
    speed_rpm = np.abs(velocities)*ratios*60/(2*math.pi)
    continuous_motor_limits = motor_torque_limits(motors, speed_rpm)
    motor_policy = getattr(requirement, "motor_load_policy", "continuous")
    if not isinstance(motor_policy, str) or motor_policy not in {"continuous", "hold_and_peak_envelope"}:
        raise ValueError("Unknown motor load policy: " + str(motor_policy))
    use_motor_peak = motor_policy == "hold_and_peak_envelope"
    motor_limits = (motor_peak_torque_limits(motors, speed_rpm)
                    if use_motor_peak else continuous_motor_limits)
    reducer_speed_limits = np.array([r.evaluation_input_speed_limit_rpm for r in reducers])
    reducer_speed_ok = speed_rpm <= reducer_speed_limits+1e-9
    load_policy = getattr(requirement, "reducer_load_policy", "continuous")
    if not isinstance(load_policy, str) or load_policy not in {"continuous", "hold_and_repeated_peak"}:
        raise ValueError("Unknown reducer load policy: " + str(load_policy))
    separate_hold = load_policy == "hold_and_repeated_peak"
    enforce_hold = separate_hold or use_motor_peak
    gear_hold_limits = np.array([r.output_torque_limit_Nm for r in reducers])
    gear_peak_limits = np.array([min(r.peak_output_torque_Nm,
        r.max_input_torque_Nm*r.reduction_ratio if r.max_input_torque_Nm is not None else np.inf)
        for r in reducers])
    gear_limits = gear_peak_limits if separate_hold else gear_hold_limits
    hold_limits = np.minimum(gear_hold_limits, ratios*motor_torque_limits(motors, np.zeros(6)))
    gravity_score_limits = gear_hold_limits if enforce_hold else hold_limits
    motor_speed_limits = np.array([m.evaluation_speed_limit_rpm for m in motors])
    motor_speed_ok = speed_rpm <= motor_speed_limits+1e-9
    peak_curve_speed_limits = np.array([
        min(m.evaluation_speed_limit_rpm, m.peak_torque_speed_curve[-1][0])
        for m in motors]) if use_motor_peak else motor_speed_limits
    dynamic_motor_speed_ok = speed_rpm <= peak_curve_speed_limits+1e-9
    worst = math.inf
    worst_known = math.inf
    worst_continuous = math.inf
    worst_pose, limiting_joint, failure = "", 0, ""
    reachable, jg, capacity_jg, weight_sum = 0, 0., 0., 0.
    gravity_squared = np.zeros(6)
    all_empty_ok = True
    all_known_empty_ok = True
    all_continuous_empty_ok = True
    peak_empty_utilization = np.zeros(6)
    peak_required_utilization = np.zeros(6)
    rows, basis_rows = [], []
    for pose in poses:
        pose_id = _pose_value(pose, "pose_id")
        q = resolve_joint_positions(pose, candidate)
        if q is None:
            q = solve_pose(model0, data0, _pose_value(pose, "position_m"),
                           _pose_value(pose, "quaternion_wxyz"))
        row = {"pose_id": pose_id, "reachable": q is not None,
               "pose_kind": _optional_pose_value(pose, "pose_kind", "cartesian_target"),
               "reference_name": _optional_pose_value(pose, "reference_name", "")}
        if q is None:
            row.update(pmax_kg=0., limiting_joint="", limitation="unreachable")
            rows.append(row)
            failure = "unreachable_reference_poses"
            continue
        reachable += 1
        g, matrix, quadratic = load_basis(model0, data0, q)
        g1, matrix1, quadratic1 = load_basis(model1, data1, q)
        # load_basis has already evaluated kinematics at this exact joint state.
        site = model0.site("tool_frame").id
        position = data0.site_xpos[site].copy()
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(quaternion, data0.site_xmat[site])
        for axis, value in zip("xyz", position*1000):
            row[f"tool_p{axis}_mm"] = float(value)
        for axis, value in zip("wxyz", quaternion):
            row[f"tool_q{axis}"] = float(value)
        row["shoulder_to_tcp_mm"] = float(np.linalg.norm(position-SHOULDER_ORIGIN_M)*1000)
        dg, dm, dh = g1-g, matrix1-matrix, quadratic1-quadratic
        # Arrays have [velocity case, acceleration corner, joint] dimensions.
        intercept = g[None, None, :]+(accelerations@matrix.T)[None, :, :]+(features@quadratic)[:, None, :]
        slope = dg[None, None, :]+(accelerations@dm.T)[None, :, :]+(features@dh)[:, None, :]
        motor_intercept = intercept/ratios+(accelerations*(rotors*ratios))[None, :, :]
        motor_slope = slope/ratios
        motor_cap, m_index, motor_empty = _payload_limit(
            motor_intercept, motor_slope, motor_limits[:, None, :])
        continuous_motor_cap, _, continuous_motor_empty = (_payload_limit(
            motor_intercept, motor_slope, continuous_motor_limits[:, None, :])
            if use_motor_peak else (motor_cap, m_index, motor_empty))
        gear_cap, g_index, gear_empty = _payload_limit(intercept, slope, gear_limits)
        hold_cap, h_index, hold_empty = _payload_limit(g, dg, hold_limits)
        gear_kind = "reducer_repeated_peak_torque" if separate_hold else "reducer_continuous_torque"
        motor_kind = "motor_peak_speed_envelope" if use_motor_peak else "motor_continuous_envelope"
        pose_cap, index, kind = (motor_cap, m_index, motor_kind) if motor_cap <= gear_cap else (gear_cap, g_index, gear_kind)
        known_cap = min(hold_cap, gear_cap)
        continuous_cap = min(continuous_motor_cap, gear_cap)
        if enforce_hold:
            continuous_cap = min(continuous_cap, hold_cap)
        if enforce_hold and hold_cap < pose_cap:
            pose_cap, index, kind = hold_cap, (0, 0, int(h_index[-1])), "static_holding_torque"
        empty_ok = motor_empty and gear_empty and bool(np.all(reducer_speed_ok))
        known_empty_ok = hold_empty and gear_empty and bool(np.all(reducer_speed_ok)) and bool(np.all(motor_speed_ok))
        continuous_empty_ok = continuous_motor_empty and gear_empty and bool(np.all(reducer_speed_ok))
        if enforce_hold:
            empty_ok &= hold_empty and bool(np.all(dynamic_motor_speed_ok))
            continuous_empty_ok &= hold_empty and bool(np.all(motor_speed_ok))
        if not np.all(reducer_speed_ok):
            pose_cap = 0.
            known_cap = 0.
            continuous_cap = 0.
            speed_ratio = speed_rpm/reducer_speed_limits
            velocity_index, joint_index = np.unravel_index(np.argmax(speed_ratio), speed_ratio.shape)
            index = (velocity_index, 0, joint_index)
            kind = "reducer_input_speed"
        elif enforce_hold and not np.all(dynamic_motor_speed_ok):
            pose_cap = 0.
            speed_ratio = np.divide(speed_rpm, peak_curve_speed_limits,
                out=np.zeros_like(speed_rpm), where=peak_curve_speed_limits > 0)
            speed_ratio = np.where((peak_curve_speed_limits == 0) & (speed_rpm > 0),
                                   np.inf, speed_ratio)
            velocity_index, joint_index = np.unravel_index(
                np.argmax(speed_ratio), speed_rpm.shape)
            index, kind = (velocity_index, 0, joint_index), (
                "motor_peak_curve_speed" if use_motor_peak else "motor_evaluation_speed")
        if not known_empty_ok:
            known_cap = 0.
        if not continuous_empty_ok:
            continuous_cap = 0.
        all_empty_ok &= empty_ok
        all_known_empty_ok &= known_empty_ok
        all_continuous_empty_ok &= continuous_empty_ok
        worst_known = min(worst_known, known_cap)
        worst_continuous = min(worst_continuous, continuous_cap)
        joint = int(index[-1])+1
        if pose_cap < worst:
            worst, worst_pose, limiting_joint = pose_cap, pose_id, joint
            if not empty_ok:
                failure = "empty_arm_exceeds_basis_limits"
        required_gravity = g+requirement.payload_upper_kg*dg
        weight = float(_pose_value(pose, "hold_weight"))
        jg += weight*float(np.sum((required_gravity/gravity_score_limits)**2))
        capacity_jg += weight*float(np.sum((required_gravity/hold_limits)**2))
        gravity_squared += weight*required_gravity**2
        weight_sum += weight
        row.update(pmax_kg=float(pose_cap), pmax_known_constraints_kg=float(known_cap),
                   pmax_continuous_screen_kg=float(continuous_cap),
                   limiting_joint=joint, limitation=kind,
                   empty_arm_passes=empty_ok, limiting_velocity_case=int(index[0]),
                   limiting_accel_corner=(-1 if kind == "static_holding_torque" else int(index[1])),
                   known_constraints_empty_arm_passes=known_empty_ok)
        required_load = intercept+requirement.payload_upper_kg*slope
        required_motor = motor_intercept+requirement.payload_upper_kg*motor_slope
        motor_utilization = _utilization(required_motor, motor_limits[:, None, :])
        reducer_utilization = _utilization(required_load, gear_limits)
        empty_utilization = np.maximum(
            np.max(_utilization(motor_intercept, motor_limits[:, None, :]), axis=(0, 1)),
            np.max(_utilization(intercept, gear_limits), axis=(0, 1)))
        required_hold_utilization = _utilization(required_gravity, hold_limits)
        if enforce_hold:
            empty_utilization = np.maximum(empty_utilization, _utilization(g, hold_limits))
        peak_empty_utilization = np.maximum(peak_empty_utilization, empty_utilization)
        required_utilization = np.maximum(np.max(motor_utilization, axis=(0, 1)),
                                          np.max(reducer_utilization, axis=(0, 1)))
        if enforce_hold:
            required_utilization = np.maximum(required_utilization, required_hold_utilization)
        peak_required_utilization = np.maximum(peak_required_utilization, required_utilization)
        load_peaks = np.max(np.abs(required_load), axis=(0, 1))
        motor_peaks = np.max(np.abs(required_motor), axis=(0, 1))
        utilization_peaks = np.max(motor_utilization, axis=(0, 1))
        continuous_utilization_peaks = (np.max(_utilization(
            required_motor, continuous_motor_limits[:, None, :]), axis=(0, 1))
            if use_motor_peak else utilization_peaks)
        for j in range(6):
            row[f"q{j+1}_rad"] = float(q[j])
            row[f"gravity_j{j+1}_Nm_at_required_payload"] = float(required_gravity[j])
            row[f"peak_joint_load_j{j+1}_Nm_at_required_payload"] = float(load_peaks[j])
            row[f"peak_motor_torque_j{j+1}_Nm_at_required_payload"] = float(motor_peaks[j])
            row[f"peak_motor_utilization_j{j+1}_ratio"] = float(utilization_peaks[j])
            row[f"peak_motor_continuous_utilization_j{j+1}_ratio"] = float(continuous_utilization_peaks[j])
            row[f"peak_reducer_utilization_j{j+1}_ratio"] = float(np.max(reducer_utilization[:, :, j]))
            row[f"holding_utilization_j{j+1}_ratio"] = float(required_hold_utilization[j])
            row[f"reducer_holding_utilization_j{j+1}_ratio"] = float(
                abs(required_gravity[j])/gear_hold_limits[j])
            row[f"peak_empty_arm_utilization_j{j+1}_ratio"] = float(empty_utilization[j])
            row[f"peak_motor_speed_j{j+1}_rpm"] = float(np.max(speed_rpm[:, j]))
            row[f"limiting_velocity_j{j+1}_rad_s"] = (0. if kind == "static_holding_torque"
                                                      else float(velocities[index[0], j]))
            row[f"limiting_acceleration_j{j+1}_rad_s2"] = (0. if kind == "static_holding_torque"
                                                          else float(accelerations[index[1], j]))
        rows.append(row)
        if collect_basis:
            record = {"pose_id": pose_id}
            for j in range(6):
                record[f"gravity_empty_j{j+1}_Nm"] = float(g[j])
                record[f"gravity_per_kg_j{j+1}_Nm_per_kg"] = float(dg[j])
                for k in range(6):
                    record[f"M_empty_{j+1}_{k+1}_kg_m2"] = float(matrix[j, k])
                    record[f"M_per_kg_{j+1}_{k+1}_m2"] = float(dm[j, k])
                for k in range(21):
                    record[f"velocity_empty_c{k+1}_j{j+1}_Nm_s2_per_rad2"] = float(quadratic[k, j])
                    record[f"velocity_per_kg_c{k+1}_j{j+1}_Nm_s2_per_rad2_per_kg"] = float(dh[k, j])
            basis_rows.append(record)
    all_reached = reachable == len(poses) and bool(poses)
    capacity = float(worst) if all_reached and math.isfinite(worst) else 0.
    known_capacity = float(worst_known) if all_reached and math.isfinite(worst_known) else 0.
    continuous_capacity = float(worst_continuous) if all_reached and math.isfinite(worst_continuous) else 0.
    meets = all_reached and all_empty_ok and capacity+1e-9 >= requirement.payload_upper_kg
    if not all_reached:
        failure = "unreachable_reference_poses"
    elif not meets and not failure:
        failure = "required_payload_not_supported"
    curve_evidence = [{
        "motor_id": m.motor_id,
        "curve_basis": getattr(m, "peak_curve_basis", ""),
        "operating_voltage_V": getattr(m, "operating_voltage_V", None),
        "published_rated_voltage_V": m.rated_voltage_V,
        "operating_voltage_matches_published_rating": bool(
            m.rated_voltage_V is not None and getattr(m, "operating_voltage_V", None) is not None
            and math.isclose(m.operating_voltage_V, m.rated_voltage_V)),
        "curve_points": list(getattr(m, "peak_torque_speed_curve", ())),
    } for m in motors]
    summary = {
        "geometry_valid": True, "geometry_contract": GEOMETRY_CONTRACT,
        **reach_summary,
        "pose_basis": SAMPLING_DESCRIPTION,
        "ik_policy": ("joint-state references applied directly; named extension poses resolved analytically"
                      if all(resolve_joint_positions(pose, candidate) is not None for pose in poses) else
                      "legacy Cartesian targets use a fixed canonical analytic IK branch"),
        "score_definition": (
            "conditional zero-to-P payload over the finite velocity basis and acceleration box; static holding uses continuous ratings, motion uses the recorded speed-dependent motor peak envelope; transient duration and duty unverified"
            if use_motor_peak else
            "conservative zero-to-P payload over the finite velocity basis and acceleration box; motor dynamic torque capped by continuous rating"),
        "capacity_model": (
            "continuous holding; speed-aware conditional motor transient envelope; selected reducer motion policy; ideal lossless 1:1 input belts"
            if use_motor_peak else
            "continuous-rated motors; rated reducer holding and repeated-peak motion; ideal lossless 1:1 input belts"
            if separate_hold else "continuous-rated motor/reducer screening; ideal lossless 1:1 input belts"),
        "motor_load_policy": motor_policy,
        "pmax_is_conditional_on_transient_envelope": use_motor_peak,
        "pmax_continuous_screen_kg": continuous_capacity,
        "continuous_screen_passes_required_payload": bool(all_reached and all_continuous_empty_ok and
            continuous_capacity+1e-9 >= requirement.payload_upper_kg),
        "continuous_screen_definition": "Same rigid body, reducer policy, holding limits and speed basis; motor dynamic torque restricted to the continuous envelope instead of the selected transient curve.",
        "reducer_load_policy": load_policy,
        "reducer_holding_output_limits_Nm": gear_hold_limits.tolist(),
        "reducer_repeated_peak_output_limits_Nm": gear_peak_limits.tolist(),
        "reducer_motion_output_limits_Nm": gear_limits.tolist(),
        "holding_output_limits_Nm": hold_limits.tolist(),
        "pmax_known_constraints_kg": known_capacity,
        "known_constraints_passes_required_payload": bool(all_reached and all_known_empty_ok and
            known_capacity+1e-9 >= requirement.payload_upper_kg),
        "known_constraints_definition": "Static motor continuous/reducer rated holding plus reducer motion limit and supported speeds. Omits the unverified dynamic motor transient constraint; not a complete arm payload rating.",
        "motor_transient_rating_verified": False,
        "duty_cycle_verified": False,
        "thermal_duty_verified": False,
        "hardware_validation_status": ("conditional_transient_envelope" if use_motor_peak
                                        else "conservative_basis_screen_only"),
        "motor_peak_duration_s": [m.peak_duration_s for m in motors],
        "motor_peak_durations_published": all(m.peak_duration_s is not None for m in motors),
        "duty_cycle_policy": "No timed operating cycle or manufacturer equivalent-load rule is supplied. Motor/reducer peak comparisons are instantaneous screening only; missing duration remains unknown and no RMS, cycle-average, thermal or lifetime pass is inferred. Momentary/emergency reducer ratings are not used.",
        "motor_envelope_assumption": (
            "Static holding uses the continuous motor envelope. Motion linearly interpolates the recorded peak torque-speed curve at operating voltage, capped by published peak torque; no extrapolation outside curve endpoints or supported motor rpm. Published peak duration is recorded but no timed duty verification is performed."
            if use_motor_peak else
            "Per motor: rated_continuous uses published continuous torque from rest through rated rpm with no extrapolation; legacy_linear uses min(rated torque, stall*(1-rpm/no-load)). Known mechanical maxima enforced. No measured thermal curve."),
        "motor_curve_evidence": curve_evidence,
        "motor_peak_curve_speed_limits_rpm": peak_curve_speed_limits.tolist() if use_motor_peak else None,
        "motor_dynamic_torque_limits_by_velocity_Nm": motor_limits.tolist(),
        "motor_capacity_models": [m.capacity_model for m in motors],
        "motor_evaluation_speed_limits_rpm": [m.evaluation_speed_limit_rpm for m in motors],
        "reducer_evaluation_input_speed_limits_rpm": reducer_speed_limits.tolist(),
        "unpublished_reducer_input_torque_limits": [j+1 for j, r in enumerate(reducers) if r.max_input_torque_Nm is None],
        "reducer_input_torque_policy": "Published output ratings enforced; input-shaft torque constraint applied only when separately published. Unpublished input limits remain unverified.",
        "velocity_cases_per_pose": len(velocities),
        "joint_speed_limits_rad_s": np.max(np.abs(velocities), axis=0).tolist(),
        "joint_accel_rad_s2": requirement.joint_accel_rad_s2,
        "motor_rotor_inertia_basis": [m.rotor_inertia_basis for m in motors],
        "motor_evaluation_rotor_inertia_kg_m2": rotors.tolist(),
        "motor_rotor_inertia_assumption": "Published rotor inertia when available; otherwise explicit conservative upper bound M_motor * R_envelope^2. Unpublished rotor values remain null in source inputs.",
        "velocity_coefficient_order": [*(f"qdot{j+1}^2" for j in range(6)),
                                       *(f"qdot{i+1}*qdot{j+1}" for i, j in _PAIRS)],
        "acceleration_corners_per_velocity": len(accelerations),
        "payload_inertia_model": "TCP-centered sphere, radius 10 mm, inertia per kg 4e-5 m2",
        "unknown_reducer_rotating_inertia": "omitted; complete reducer case mass on parent",
        "collision_checks": False, "joint_stop_checks": False,
        "worst_pose_id": worst_pose, "limiting_joint": limiting_joint,
        "failure_reason": failure, "empty_arm_passes": all_reached and all_empty_ok,
        "peak_empty_arm_utilization": float(np.max(peak_empty_utilization)),
        "peak_empty_arm_utilization_by_joint": peak_empty_utilization.tolist(),
        "peak_required_payload_utilization": float(np.max(peak_required_utilization)),
        "peak_required_payload_utilization_by_joint": peak_required_utilization.tolist(),
        "unreachable_pose_count": len(poses)-reachable,
        "required_payload_upper_kg": requirement.payload_upper_kg,
        "required_payload_lower_kg": requirement.payload_lower_kg if hasattr(requirement, "payload_lower_kg") else 0.,
        "required_payload_passes": meets,
        "jg_definition": ("weighted mean of squared gravity torques divided by the common reducer rated limits at required payload"
                          if enforce_hold else "sum of squared normalized gravity torques at required upper payload, fixed pose weights"),
        "gravity_capacity_score_JG": capacity_jg/weight_sum if all_reached and weight_sum else None,
        "gravity_mean_square_Nm2_by_joint": (gravity_squared/weight_sum).tolist() if all_reached and weight_sum else None,
    }
    return Evaluation(capacity, jg/weight_sum if all_reached and weight_sum else None,
                      float(np.sum(model0.body_mass)), reachable, len(poses), meets,
                      rows, xml, summary, basis_rows)

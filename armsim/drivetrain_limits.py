"""Published-rating policies shared by imported-model validation and playback."""

import math
from bisect import bisect_right


def positive(value, name):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be positive and finite")
    return result


def motor_speed_limit(drive):
    policy = drive.get("motor_capacity_model", "legacy_linear")
    mechanical = drive.get("max_motor_speed_rpm")
    if policy == "rated_continuous":
        maximum = positive(drive["motor_rated_speed_rpm"], "rated speed")
        if mechanical is not None:
            maximum = min(maximum, positive(mechanical, "mechanical speed"))
    elif policy == "legacy_linear":
        maximum = positive(mechanical, "mechanical speed")
        positive(drive["stall_torque_Nm"], "stall torque")
        positive(drive["no_load_speed_rpm"], "no-load speed")
    else:
        raise ValueError(f"Unknown motor capacity model: {policy}")
    explicit = drive.get("motor_evaluation_speed_limit_rpm", maximum)
    explicit = positive(explicit, "motor evaluation speed")
    if explicit > maximum+1e-8:
        raise ValueError("Motor evaluation speed exceeds the supported rating")
    return explicit


def reducer_speed_limit(drive):
    maximum = positive(drive["max_reducer_input_speed_rpm"], "reducer input speed")
    reference = drive.get("reducer_rated_input_speed_rpm")
    if reference is not None:
        maximum = min(maximum, positive(reference, "reducer rating reference speed"))
    explicit = drive.get("reducer_evaluation_input_speed_limit_rpm", maximum)
    explicit = positive(explicit, "reducer evaluation speed")
    if explicit > maximum+1e-8:
        raise ValueError("Reducer evaluation speed exceeds the supported rating")
    return explicit


def reducer_torque_limit(drive):
    """Instantaneous reducer ceiling under the explicitly selected policy.

    A repeated-peak rating does not establish a permissible holding load or a
    thermal duty cycle. Those checks remain separate from playback clipping.
    """
    ratio = positive(drive["ratio"], "ratio")
    continuous = positive(drive["continuous_output_torque_Nm"], "reducer continuous output torque")
    policy = drive.get("reducer_load_policy", "continuous")
    if policy == "hold_and_repeated_peak":
        maximum = positive(drive["peak_output_torque_Nm"], "reducer repeated-peak output torque")
        if maximum < continuous:
            raise ValueError("Reducer repeated-peak torque must not be below its continuous rating")
    elif policy == "continuous":
        maximum = continuous
    else:
        raise ValueError(f"Unknown reducer load policy: {policy}")
    input_limit = drive.get("max_input_torque_Nm")
    if input_limit is not None:
        maximum = min(maximum, positive(input_limit, "reducer input torque")*ratio)
    return maximum


def joint_torque_limit(drive):
    ratio = positive(drive["ratio"], "ratio")
    policy = drive.get("motor_load_policy", "continuous")
    if policy == "hold_and_peak_envelope":
        motor_maximum = _motor_peak_curve(drive)[0][1]
    elif policy == "continuous":
        motor_maximum = positive(drive["motor_rated_torque_Nm"], "motor rated torque")
    else:
        raise ValueError(f"Unknown motor load policy: {policy}")
    return min(motor_maximum*ratio, reducer_torque_limit(drive))


def _motor_peak_curve(drive):
    """Validate the explicit transient envelope retained in an optimized file."""
    maximum = positive(drive.get("motor_peak_torque_Nm"), "motor peak torque")
    try:
        curve = tuple(tuple(float(value) for value in point)
                      for point in drive["motor_peak_torque_speed_curve"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Peak motor policy requires an explicit torque-speed curve") from exc
    if (not curve or any(len(point) != 2 or any(not math.isfinite(v) or v < 0 for v in point)
                         for point in curve)
            or curve[0][0] != 0
            or any(right[0] <= left[0] or right[1] > left[1]
                   for left, right in zip(curve, curve[1:]))
            or any(torque > maximum+1e-12 for _, torque in curve)):
        raise ValueError("Invalid motor peak torque-speed curve")
    return curve


def motor_torque_at_speed(drive, rpm):
    rpm = abs(float(rpm))
    if rpm > min(motor_speed_limit(drive), reducer_speed_limit(drive))+1e-8:
        return 0.
    policy = drive.get("motor_load_policy", "continuous")
    if policy == "hold_and_peak_envelope":
        curve = _motor_peak_curve(drive)
        if rpm > curve[-1][0]:
            return 0.
        index = bisect_right([point[0] for point in curve], rpm)
        if index == len(curve):
            return curve[-1][1]
        if index == 0:
            return curve[0][1]
        low, high = curve[index-1], curve[index]
        fraction = (rpm-low[0])/(high[0]-low[0])
        return low[1]+fraction*(high[1]-low[1])
    if policy != "continuous":
        raise ValueError(f"Unknown motor load policy: {policy}")
    rated = positive(drive["motor_rated_torque_Nm"], "motor rated torque")
    if drive.get("motor_capacity_model", "legacy_linear") == "rated_continuous":
        return rated
    return min(rated, float(drive["stall_torque_Nm"])*max(0., 1-rpm/float(drive["no_load_speed_rpm"])))

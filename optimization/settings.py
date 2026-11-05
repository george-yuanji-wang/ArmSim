"""Automatic search defaults; requirement.csv contains only project goals.

These are shared study assumptions, not additional fields users must fill in.
Resolved values are recorded in every run. Cycle/seed overrides are also
available on the command line.
"""

DEFAULT_CYCLES = 200
DEFAULT_CHECKPOINT_EVERY = 10
DEFAULT_SEED = 42
# Numerical simulation noise must not decide a lexicographic score tie.
RANK_SCORE_DECIMALS = 9
ELEVATION_LOWER_DEG = -45.
ELEVATION_UPPER_DEG = 45.
JOINT_SPEED_RAD_S = 1.
JOINT_ACCEL_RAD_S2 = 2.
ALLOWED_LAYOUTS = (7, 8, 9)
PLACEMENT_DELTA_FRACTION = .15


def automatic_dimensions(reach_upper_mm):
    """Link bounds within a requested straightened shoulder-to-TCP reach."""
    reach = float(reach_upper_mm)
    return {
        "l1_lower_mm": max(30., .15*reach),
        "l1_upper_mm": max(150., .9*reach),
        "l2_lower_mm": 90.,
        "l2_upper_mm": 90.,
        "l3_lower_mm": max(10., .03*reach),
        "l3_upper_mm": max(150., .9*reach),
        "motor_offset_max_mm": max(10., .2*reach),
    }

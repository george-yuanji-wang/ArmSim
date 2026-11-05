"""Exact extension geometry and two-coordinate length parameterization.

Reach is the unrestricted shoulder-to-TCP maximum, not a workspace-shell
radius. The short J3/J4 coupling stays at its approved 90 mm length.
"""

import math

import numpy as np

from armsim.catalog import layout_by_number


TOOL_OFFSET_M = .15


def tool_offset_m(design):
    """Fixed distal chain for this study; older saved designs retain 150 mm."""
    return getattr(design, "wrist2_length_m", .07) + .08


def _lengths_and_layout(candidate_or_lengths, layout_number=None):
    if hasattr(candidate_or_lengths, "lengths_m"):
        return tuple(candidate_or_lengths.lengths_m), layout_by_number(candidate_or_lengths.layout_number)
    if layout_number is None:
        raise ValueError("A layout number is required with raw link lengths")
    return tuple(candidate_or_lengths), layout_by_number(layout_number)


def extended_reach_m(candidate_or_lengths, layout_number=None, *, wrist2_length_m=None):
    """Maximum shoulder-to-TCP radius with unrestricted joint rotations."""
    (l1, l2, l3), layout = _lengths_and_layout(candidate_or_lengths, layout_number)
    forearm = math.hypot(l2, l3) if layout.perpendicular_j4 else l2+l3
    if wrist2_length_m is not None:
        offset = wrist2_length_m + .08
    else:
        offset = tool_offset_m(candidate_or_lengths) if hasattr(candidate_or_lengths, "lengths_m") else TOOL_OFFSET_M
    return l1+forearm+offset


def maximum_reach_pose(candidate):
    """A joint pose achieving maximum extension vertically above the shoulder."""
    lengths, layout = _lengths_and_layout(candidate)
    q = np.zeros(6)
    if layout.perpendicular_j4:
        phase = math.atan2(lengths[2], lengths[1])
        q[2] = phase
        q[4] = math.pi/2-phase
    return q


def _forearm_length(l2, l3, layout):
    return math.hypot(l2, l3) if layout.perpendicular_j4 else l2+l3


def _l3_from_forearm(l2, value, layout):
    if layout.perpendicular_j4:
        return math.sqrt(max(0., value*value-l2*l2))
    return value-l2


def coordinate_bounds(requirement, layout_number):
    """Intersect the requested extension interval with component length bounds."""
    layout = layout_by_number(layout_number)
    l2 = layout.l2_length
    (a, b), _, (c, d) = requirement.length_bounds_m
    physical_low = a+_forearm_length(l2, c, layout)+tool_offset_m(requirement)
    physical_high = b+_forearm_length(l2, d, layout)+tool_offset_m(requirement)
    low = max(requirement.reach_lower_m, physical_low)
    high = min(requirement.reach_upper_m, physical_high)
    if high < low-1e-12:
        raise ValueError(f"Layout {layout_number:02d}: straightened reach bounds cannot fit the fixed L2 coupling and link bounds")
    return float(low), float(max(low, high))


def allocation_bounds(requirement, layout_number, reach_m):
    """Feasible L1 interval at one exact extension radius; L3 follows from it."""
    layout = layout_by_number(layout_number)
    (a, b), _, (c, d) = requirement.length_bounds_m
    remaining = reach_m-tool_offset_m(requirement)
    low = max(a, remaining-_forearm_length(layout.l2_length, d, layout))
    high = min(b, remaining-_forearm_length(layout.l2_length, c, layout))
    if high < low-1e-10:
        raise ValueError(f"Layout {layout_number:02d}: no link allocation fits reach {reach_m*1000:g} mm")
    return float(low), float(max(low, high))


def lengths_from_coordinates(requirement, layout_number, reach_m, allocation):
    """Substitute L3 from reach, leaving only reach and one allocation variable."""
    layout = layout_by_number(layout_number)
    lower, upper = coordinate_bounds(requirement, layout_number)
    reach = float(np.clip(reach_m, lower, upper))
    low, high = allocation_bounds(requirement, layout_number, reach)
    l1 = low+float(np.clip(allocation, 0., 1.))*(high-low)
    l3 = _l3_from_forearm(layout.l2_length, reach-tool_offset_m(requirement)-l1, layout)
    return float(l1), float(layout.l2_length), float(l3)


def length_coordinates(candidate, requirement, layout_number=None):
    """Map an existing candidate to the bounded reach/allocation coordinates."""
    number = candidate.layout_number if layout_number is None else layout_number
    lo, hi = coordinate_bounds(requirement, number)
    reach = float(np.clip(extended_reach_m(candidate), lo, hi))
    a, b = allocation_bounds(requirement, number, reach)
    allocation = .5 if b-a < 1e-12 else float(np.clip((candidate.lengths_m[0]-a)/(b-a), 0., 1.))
    return reach, allocation


def component_length_bounds(requirement, layout_number):
    """Marginal length bounds for diagnostics; reach remains a coupled constraint."""
    layout = layout_by_number(layout_number)
    lo, hi = coordinate_bounds(requirement, layout_number)
    (a, b), _, (c, d) = requirement.length_bounds_m
    l2 = layout.l2_length
    f_low, f_high = _forearm_length(l2, c, layout), _forearm_length(l2, d, layout)
    offset = tool_offset_m(requirement)
    a, b = max(a, lo-offset-f_high), min(b, hi-offset-f_low)
    f_low = max(f_low, lo-offset-b)
    f_high = min(f_high, hi-offset-a)
    return ((float(a), float(b)), (float(l2), float(l2)),
            (_l3_from_forearm(l2, f_low, layout), _l3_from_forearm(l2, f_high, layout)))

"""Independent design variables and bounded, reproducible search proposals."""

from collections import deque
from dataclasses import asdict, dataclass, replace
from functools import lru_cache
from itertools import product
import re

import numpy as np

from armsim.catalog import layout_by_number
from . import settings
from .reach import (component_length_bounds, coordinate_bounds, length_coordinates,
                    lengths_from_coordinates)


@dataclass(frozen=True)
class Candidate:
    layout_number: int
    lengths_m: tuple[float, float, float]
    link_masses_kg: tuple[float, ...]
    com_fractions: tuple[float, ...]
    spread_fractions: tuple[float, ...]
    motor_offsets_m: tuple[float, float, float, float]
    motor_ids: tuple[str, ...]
    reducer_ids: tuple[str, ...]
    wrist2_length_m: float = .07

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, values):
        values = dict(values)
        for key in ("lengths_m", "link_masses_kg", "com_fractions",
                    "spread_fractions", "motor_offsets_m", "motor_ids", "reducer_ids"):
            values[key] = tuple(values[key])
        return cls(**values)


def active_offsets(layout_number, offsets):
    """J2 has two planar coordinates; J3/J5 have one line coordinate each."""
    drives = layout_by_number(layout_number).drives
    u2, v2, s3, s5 = offsets
    return (u2 if drives[1] == "input_belt" else 0.,
            v2 if drives[1] == "input_belt" else 0.,
            s3 if drives[2] == "input_belt" else 0.,
            s5 if drives[4] == "input_belt" else 0.)


def _pair_allowed(pair, joint):
    """Reducer eligibility is one-based in input data, zero-based internally."""
    allowed = getattr(pair[1], "allowed_joints", ())
    return not allowed or joint+1 in allowed


def _joint_pairs(pairs, joint, requirement=None):
    """Apply reducer eligibility and the study's motor scope before geometry."""
    eligible = [pair for pair in pairs if _pair_allowed(pair, joint)]
    if requirement is not None and hasattr(requirement, "allows_motor"):
        eligible = [pair for pair in eligible if requirement.allows_motor(joint+1, pair[0].motor_id)]
    if not eligible:
        raise ValueError(f"No eligible motor/reducer pair for J{joint+1}; check allowed_joints and motor selection")
    if requirement is None:
        return eligible
    speeds = getattr(requirement, "joint_speeds_rad_s", (requirement.joint_speed_rad_s,)*6)
    qualified = [pair for pair in eligible if
                 speeds[joint]*pair[1].reduction_ratio*60/(2*np.pi)
                 <= min(pair[0].evaluation_speed_limit_rpm,
                        pair[1].evaluation_input_speed_limit_rpm)]
    return qualified or eligible


def hardware_pairs(inputs, requirement, joint=None):
    """Prefer speed-capable pairs without borrowing another joint's reducer."""
    pairs = list(product(inputs.motors, inputs.reducers))
    if not pairs:
        raise ValueError("Fill motor.csv and reducer.csv before optimizing.")
    if joint is not None:
        return _joint_pairs(pairs, joint, requirement)
    # Preserve each joint's eligible fallback when its speed cannot be met.
    # Filtering globally first could otherwise discard its only reducer just
    # because a different joint has some speed-capable hardware. The evaluator
    # still rejects an underspeed design; the seed remains useful diagnostics.
    preferred = {pair for index in range(6)
                 for pair in _joint_pairs(pairs, index, requirement)}
    return [pair for pair in pairs if pair in preferred]


def _center_bounds(link):
    """Intersect COM bounds with existence of the minimum requested variance."""
    root = np.sqrt(max(0., 1-4*link.spread_fraction_lower))
    return (max(link.com_fraction_lower, (1-root)/2),
            min(link.com_fraction_upper, (1+root)/2))


def template_length_bounds(requirement, layout_number):
    """Diagnostic marginals; feasible lengths use the coupled reach coordinates."""
    return component_length_bounds(requirement, layout_number)


def placement_delta_bounds(lengths, requirement):
    """Offsets tune the base mounting locations, not unrestricted new mounts."""
    maximum = requirement.motor_offset_max_m
    fractions = np.array([.12, .12, lengths[0], lengths[2]])*settings.PLACEMENT_DELTA_FRACTION
    return np.minimum(maximum, fractions)


def _fit_initial_placement(candidate, inputs, requirement):
    """Try small permitted placement changes before replacing selected hardware.

    The zero-offset catalog template is only a seed, not a mounting constraint.
    All trials retain the same hardware, lengths, topology and geometry gates.
    """
    active = np.flatnonzero(active_offsets(candidate.layout_number, np.ones(4)))
    if not len(active):
        return None
    bounds = placement_delta_bounds(candidate.lengths_m, requirement)
    preferred = np.zeros(4)
    preferred[[2, 3]] = -.003
    deltas = [preferred]
    for values in product((0., -.003, .003), repeat=len(active)):
        delta = np.zeros(4)
        delta[active] = values
        deltas.append(delta)
    seen = {_physical_key(candidate)}
    for delta in deltas:
        offsets = np.clip(np.asarray(candidate.motor_offsets_m)+delta, -bounds, bounds)
        trial = replace(candidate, motor_offsets_m=active_offsets(candidate.layout_number, offsets))
        key = _physical_key(trial)
        if key in seen:
            continue
        seen.add(key)
        resolved = _compile_candidate(trial, inputs)
        if resolved is not None:
            return resolved
    return None


def _fit_initial_lengths(candidate, inputs, requirement):
    """Find a bounded feasible length allocation without changing hardware.

    A compact coaxial stack can fit between the quarter-grid samples while
    both neighbouring samples fail. Probe the remaining dyadic eighth points
    before concluding that this fixed hardware needs replacement. These are
    only geometry seeds; the normal continuous proposals still optimize them.
    """
    low,high=coordinate_bounds(requirement,candidate.layout_number)
    seen={_physical_key(candidate)}
    for reach in dict.fromkeys(((low+high)/2,low,high)):
        for allocation in (0.,.25,.75,1.,.125,.375,.625,.875):
            trial=replace(candidate,lengths_m=lengths_from_coordinates(
                requirement,candidate.layout_number,reach,allocation))
            key=_physical_key(trial)
            if key in seen:continue
            seen.add(key)
            resolved=_compile_candidate(trial,inputs)
            if resolved is None:
                resolved=_fit_initial_placement(trial,inputs,requirement)
            if resolved is not None:return resolved
    return None


def _capacity(pair):
    """A holding-capacity seed heuristic; the load suite still ranks designs."""
    motor, reducer = pair
    return min(motor.rated_torque_Nm*reducer.reduction_ratio,
               reducer.output_torque_limit_Nm)


def _with_hardware(candidate, joint, pair):
    if not _pair_allowed(pair, joint):
        raise ValueError(f"Reducer {pair[1].reducer_id} is not eligible for J{joint+1}")
    motors, reducers = list(candidate.motor_ids), list(candidate.reducer_ids)
    motors[joint], reducers[joint] = pair[0].motor_id, pair[1].reducer_id
    return replace(candidate, motor_ids=tuple(motors), reducer_ids=tuple(reducers))


def _physical_key(candidate):
    return (candidate.layout_number, candidate.lengths_m,
            candidate.motor_offsets_m, candidate.motor_ids, candidate.reducer_ids,
            candidate.wrist2_length_m)


@lru_cache(maxsize=512)
def _compile_geometry(physical_key, inputs):
    """Resolve a proposal only after its actual base geometry passes validation."""
    from .model_factory import build_template_model, GeometryError
    from .structural_mass import candidate_from_model

    number, lengths, offsets, motors, reducers, wrist2_length = physical_key
    if any(not _pair_allowed((inputs.motor_by_id[mid], inputs.reducer_by_id[rid]), joint)
           for joint, (mid, rid) in enumerate(zip(motors, reducers))):
        return None
    candidate = Candidate(number, lengths, (0.,)*6, (0.,)*6, (0.,)*6,
                          offsets, motors, reducers, wrist2_length)
    try:
        model, _ = build_template_model(candidate, inputs)
    except GeometryError:
        return None
    return candidate_from_model(candidate, model)


def _compile_candidate(candidate, inputs):
    # The bounded cache stores immutable resolved candidates, never MuJoCo
    # model/data objects. Independent mass/COM serialization fields do not
    # cause duplicate geometry probes.
    return _compile_geometry(_physical_key(candidate), inputs)


@lru_cache(maxsize=32)
def _repair_options(pair_tuple, joint):
    """Cover strength and envelope-size extremes within 96 catalog options.

    A capacity-sorted prefix can contain only large units in a real catalog.
    Interleave it with short and small-radius envelopes so compact fitting
    options are considered even when their torque ranks far down the catalog.
    """
    capacity = sorted(pair_tuple, key=lambda pair: (
        -_capacity(pair), pair[0].mass_kg+pair[1].mass_kg,
        pair[0].motor_id, pair[1].reducer_id))
    axial = sorted(pair_tuple, key=lambda pair: (
        pair[0].length_m+pair[1].length_m,
        max(pair[0].radius_m, pair[1].radius_m), -_capacity(pair),
        pair[0].motor_id, pair[1].reducer_id))
    radial = sorted(pair_tuple, key=lambda pair: (
        max(pair[0].radius_m, pair[1].radius_m),
        pair[0].length_m+pair[1].length_m, -_capacity(pair),
        pair[0].motor_id, pair[1].reducer_id))
    orders = (capacity, axial, radial) if joint < 3 else (axial, radial, capacity)
    iterators = [iter(order) for order in orders]
    seen, result = set(), []
    while iterators and len(result) < 96:
        remaining = []
        for items in iterators:
            for pair in items:
                motor, reducer = pair
                envelope = (motor.radius_m, motor.length_m, reducer.radius_m, reducer.length_m)
                if envelope not in seen:
                    seen.add(envelope)
                    result.append(pair)
                    remaining.append(items)
                    break
            if len(result) >= 96:
                break
        iterators = remaining
    return tuple(result)


def _envelope_distance(pair, reference):
    """Scale-independent change from a currently fitting local envelope."""
    motor, reducer = pair
    old_motor, old_reducer = reference
    values = np.array((motor.radius_m, motor.length_m, reducer.radius_m, reducer.length_m))
    old = np.array((old_motor.radius_m, old_motor.length_m,
                    old_reducer.radius_m, old_reducer.length_m))
    delta = np.log(values/old)
    return float(delta@delta)


@lru_cache(maxsize=64)
def _nearby_hardware_options(pair_tuple, reference):
    """Keep ratios and windings available around an already fitting assembly."""
    return tuple(sorted(pair_tuple, key=lambda pair: (
        _envelope_distance(pair, reference),
        pair[0].motor_id != reference[0].motor_id,
        -_capacity(pair), pair[0].mass_kg+pair[1].mass_kg,
        pair[0].motor_id, pair[1].reducer_id)))


@lru_cache(maxsize=64)
def _challenge_options(pair_tuple, joint, reference):
    """Reserve half a joint's probes for local stronger hardware alternatives.

    A one-millimeter package change can double a reducer's rating. Global
    strength/size extremes alone miss such alternatives in a broad catalog.
    Nearby stronger options therefore precede the independent global coverage.
    Only the challenge deduplicates geometry; ordinary mutation retains every
    ratio and winding because inertia and rated speed can change its objective.
    """
    selected_capacity = _capacity(reference)
    representatives = {}
    for pair in sorted(pair_tuple, key=lambda pair: (
            -_capacity(pair), pair[0].mass_kg+pair[1].mass_kg,
            pair[0].motor_id, pair[1].reducer_id)):
        if _capacity(pair) <= selected_capacity+1e-12:
            continue
        motor, reducer = pair
        envelope = (motor.radius_m, motor.length_m, reducer.radius_m, reducer.length_m)
        representatives.setdefault(envelope, pair)
    available = tuple(representatives.values())
    nearest = sorted(available, key=lambda pair: (
        _envelope_distance(pair, reference), -_capacity(pair),
        pair[0].mass_kg+pair[1].mass_kg, pair[0].motor_id, pair[1].reducer_id))
    same_motor = sorted((pair for pair in available if pair[0].motor_id == reference[0].motor_id),
                        key=lambda pair: (_envelope_distance(pair, reference), -_capacity(pair)))
    options, seen = [], set()
    # Interleave same-motor reducer upgrades and nearby combined changes.
    for index in range(max(len(same_motor), len(nearest))):
        for group in (same_motor, nearest):
            if index >= len(group):
                continue
            pair = group[index]
            key = (pair[0].radius_m, pair[0].length_m, pair[1].radius_m, pair[1].length_m)
            if key not in seen:
                options.append(pair)
                seen.add(key)
        if len(options) >= 8:
            break
    local = options[:8]
    # Preserve a second, global pool rather than spending every probe locally.
    global_options = _repair_options(pair_tuple, joint)
    global_unique = []
    for pair in global_options:
        key = (pair[0].radius_m, pair[0].length_m, pair[1].radius_m, pair[1].length_m)
        if key not in seen and _capacity(pair) > selected_capacity+1e-12:
            global_unique.append(pair)
            seen.add(key)
    return tuple(local+global_unique)


def _strongest_fitting_hardware(candidate, inputs, pairs, *, reverse=False, diagnostics=None,
                               requirement=None):
    """Challenge all six selections with stronger fitting catalog hardware.

    This is a bounded construction of a capacity benchmark, not the objective
    function. It can restore several previously downsized drives together when
    a greedy, mass-breaking search would otherwise keep each one too small.
    Identical catalog envelopes need only one geometry trial here; all catalog
    alternatives remain available to the ordinary mutation proposals.
    """
    if requirement is not None and hasattr(requirement, "allows_motor") and any(
            not requirement.allows_motor(joint, motor_id)
            for joint, motor_id in enumerate(candidate.motor_ids, 1)):
        candidate = _fit_initial_hardware(candidate, inputs, pairs, requirement=requirement)
    current = _compile_candidate(candidate, inputs)
    if current is None:
        return candidate
    order = reversed(range(6)) if reverse else range(6)
    probes = 0
    for joint in order:
        selected = (inputs.motor_by_id[current.motor_ids[joint]],
                    inputs.reducer_by_id[current.reducer_ids[joint]])
        joint_probes = 0
        # Reserve nearby fitting-envelope upgrades before global strength/size
        # extremes. Each joint still receives at most16 of96 total probes.
        options = _joint_pairs(pairs, joint, requirement)
        for pair in _challenge_options(tuple(options), joint, selected):
            if _capacity(pair) <= _capacity(selected)+1e-12:
                continue
            if joint_probes >= 16:
                if diagnostics is not None:
                    diagnostics["capacity_challenge_probe_limit_reached"] = True
                break
            probes += 1
            joint_probes += 1
            trial = _compile_candidate(_with_hardware(current, joint, pair), inputs)
            if trial is not None:
                current = trial
                selected = pair
            elif diagnostics is not None:
                diagnostics["geometry_rejected_trials"] = diagnostics.get("geometry_rejected_trials", 0)+1
    if diagnostics is not None:
        diagnostics["capacity_challenge_probes"] = probes
    return current


def _fit_initial_hardware(candidate, inputs, pairs, *, requirement=None):
    """Repair catalog choices against the actual base mounts, with bounded work.

    A small motor is not automatically a fitting motor: its four approved
    stays must still make a spatial mount. Probe the same compiled template
    used for scoring, then try catalog alternatives at the failing joint.
    This chooses a usable starting point; optimization still scores hardware.
    """
    from .model_factory import build_template_model, GeometryError

    # A previous study's saved choice may be geometrically valid but outside
    # this study's shortlist. Bring it into scope before any geometry probe.
    if requirement is not None and hasattr(requirement, "allows_motor"):
        for joint, mid in enumerate(candidate.motor_ids):
            if requirement.allows_motor(joint+1, mid):
                continue
            options = _joint_pairs(pairs, joint, requirement)
            reference = (inputs.motor_by_id[mid], inputs.reducer_by_id[candidate.reducer_ids[joint]])
            pair = min(options, key=lambda item: (
                _envelope_distance(item, reference), -_capacity(item), item[0].mass_kg))
            candidate = _with_hardware(candidate, joint, pair)

    def affected_joints(message):
        match = re.search(r"\bJ([1-6])\b", message)
        if match:
            return (int(match.group(1))-1,)
        # Parent frame dimensions depend on both its incoming and outgoing
        # catalog envelopes. Try the outgoing component first.
        names = ("base_link", "shoulder_link", "upper_arm_link", "forearm_link",
                 "wrist_1_link", "wrist_2_link", "tool_flange_link")
        for index, name in enumerate(names):
            if name in message:
                return tuple(dict.fromkeys((min(index, 5), max(0, index-1))))
        return tuple(range(6))

    pending = deque([(candidate, None, None)])
    seen = {(candidate.motor_ids, candidate.reducer_ids)}
    # This is a bounded local repair, not an exponential hardware search.
    for _ in range(96):
        if not pending:
            break
        current, changed_joint, previous_failure = pending.popleft()
        try:
            for joint, (mid, rid) in enumerate(zip(current.motor_ids, current.reducer_ids)):
                if not _pair_allowed((inputs.motor_by_id[mid], inputs.reducer_by_id[rid]), joint):
                    raise GeometryError(f"J{joint+1} reducer is not eligible for this joint")
                if requirement is not None and hasattr(requirement, "allows_motor") and not requirement.allows_motor(joint+1, mid):
                    raise GeometryError(f"J{joint+1} motor is not eligible for this study")
            build_template_model(current, inputs)
            return current
        except GeometryError as exc:
            failure_joints = affected_joints(str(exc))
            joints = failure_joints
        if joints == previous_failure:
            # The parent already queued other pairs for this same joint. Do
            # not recursively enumerate that identical one-joint choice again.
            # Multi-joint frame errors may still require the other neighbor.
            joints = tuple(joint for joint in joints if joint != changed_joint)
        alternatives = []
        for joint in joints:
            options = _joint_pairs(pairs, joint, requirement)
            for motor, reducer in _repair_options(tuple(options), joint):
                motors, reducers = list(current.motor_ids), list(current.reducer_ids)
                motors[joint], reducers[joint] = motor.motor_id, reducer.reducer_id
                key = (tuple(motors), tuple(reducers))
                if key not in seen:
                    seen.add(key)
                    alternatives.append((replace(current, motor_ids=key[0], reducer_ids=key[1]),
                                         joint, failure_joints))
                if len(alternatives) >= 96:
                    break
            if len(alternatives) >= 96:
                break
        # Repair a newly exposed joint before exploring unrelated alternatives.
        pending.extendleft(reversed(alternatives))
        while len(pending) > 96:
            pending.pop()
    # The evaluator will report the actual geometric error if these catalogs
    # cannot supply a supported seed within the bounded repair search.
    return candidate


def _uniform_hardware_seed(candidate, inputs, pairs, requirement):
    """Find a complete compact assembly before trying coordinated upgrades.

    Starting all six axes with oversized high-capacity units can exhaust a
    bounded one-joint repair before any complete model fits. A uniform trial
    changes all six units together, using the approved wrist envelopes as a
    reference because those are the tightest fixed-length mounting templates.
    This is only a feasible starting point; the subsequent capacity challenge
    and load search still select each joint independently.
    """
    from armsim.mechanism_parameters import MechanismParameters

    nominal = MechanismParameters.nominal(layout_by_number(candidate.layout_number))
    reference = np.array([np.median(values[3:]) for values in (
        nominal.motor_radii, nominal.motor_lengths, nominal.gear_radii, nominal.gear_lengths)])
    representatives = {}
    for pair in sorted(pairs, key=lambda pair: (
            -_capacity(pair), pair[0].mass_kg+pair[1].mass_kg,
            pair[0].motor_id, pair[1].reducer_id)):
        motor, reducer = pair
        envelope = (motor.radius_m, motor.length_m, reducer.radius_m, reducer.length_m)
        representatives.setdefault(envelope, pair)

    def proximity(item):
        envelope, pair = item
        relative = (np.array(envelope)-reference)/reference
        return (float(relative@relative), -_capacity(pair),
                pair[0].mass_kg+pair[1].mass_kg, pair[0].motor_id, pair[1].reducer_id)

    options = sorted(representatives.items(), key=proximity)[:96]
    pools = [_joint_pairs(pairs, joint, requirement) for joint in range(6)]
    seen = set()
    for _, (motor, reducer) in options:
        # With dedicated reducers a coherent seed can share a motor but must
        # retain each joint's assigned reducer. Unrestricted catalogs preserve
        # the original truly uniform trial.
        reference_pair = (motor, reducer)
        chosen = [(reference_pair if reference_pair in pool else min(pool, key=lambda pair: (
                    pair[0].motor_id != motor.motor_id,
                    _envelope_distance(pair, reference_pair), -_capacity(pair),
                    pair[0].motor_id, pair[1].reducer_id))) for pool in pools]
        trial = replace(candidate, motor_ids=tuple(pair[0].motor_id for pair in chosen),
                        reducer_ids=tuple(pair[1].reducer_id for pair in chosen))
        key = _physical_key(trial)
        if key in seen:
            continue
        seen.add(key)
        resolved = _compile_candidate(trial, inputs)
        if resolved is not None:
            return resolved
    return None


def initial_candidate(inputs, requirement):
    """Start from one approved layout within the straightened reach interval."""
    number = requirement.allowed_layouts[0]
    lo, hi = coordinate_bounds(requirement, number)
    lengths = lengths_from_coordinates(requirement, number, (lo+hi)/2, .5)
    # Retained serialization fields are placeholders until the chosen geometry
    # supplies its actual mass properties; none is an optimization coordinate.
    masses, coms, spreads = [], [], []
    for link in inputs.links:
        masses.append(link.mass_lower_kg)
        low, high = _center_bounds(link)
        center = (low + high) / 2
        coms.append(center)
        spreads.append(min((link.spread_fraction_lower + link.spread_fraction_upper)/2,
                           center*(1-center)))
    pairs = hardware_pairs(inputs, requirement)
    pools = [hardware_pairs(inputs, requirement, joint) for joint in range(6)]
    chosen = [(max(pool, key=_capacity) if joint < 3 else min(pool, key=lambda pair: (
        pair[0].length_m+pair[1].length_m, pair[0].radius_m+pair[1].radius_m,
        pair[0].mass_kg+pair[1].mass_kg))) for joint, pool in enumerate(pools)]
    candidate = Candidate(number, lengths, tuple(masses), tuple(coms), tuple(spreads),
                          (0., 0., 0., 0.),
                          tuple(m.motor_id for m, _ in chosen), tuple(r.reducer_id for _, r in chosen),
                          getattr(requirement, "wrist2_length_m", .07))
    resolved = _compile_candidate(candidate, inputs)
    if resolved is None:
        resolved = _fit_initial_placement(candidate, inputs, requirement)
    if resolved is None:
        resolved = _fit_initial_lengths(candidate, inputs, requirement)
    # Broad supplier catalogs need a complete compact alternative rather than
    # only a coordinate repair of an invalid all-oversized assembly.
    if resolved is None and len(pairs) > 96:
        resolved = _uniform_hardware_seed(candidate, inputs, pairs, requirement)
    if resolved is None:
        repaired = _fit_initial_hardware(candidate, inputs, pairs, requirement=requirement)
        resolved = _compile_candidate(repaired, inputs)
    if resolved is None:
        resolved = _uniform_hardware_seed(candidate, inputs, pairs, requirement)
    if resolved is not None:
        candidate = resolved
    return _strongest_fitting_hardware(candidate, inputs, pairs, requirement=requirement)


def propose_candidate(best, inputs, requirement, rng, cycle, *, diagnostics=None):
    """Prevalidate separate coordinate blocks before spending a load evaluation.

    Ordinary trials change lengths, one permitted placement coordinate, or one
    joint's catalog pair. Periodic capacity challenges change multiple drives
    together. Geometry-only rejection trials do not consume a simulation cycle.
    Up to 24 local trials are attempted; if none fits, retain the valid incumbent.
    """
    if hasattr(requirement, "allows_motor") and any(
            not requirement.allows_motor(joint, mid)
            for joint, mid in enumerate(best.motor_ids, 1)):
        raise ValueError("Incumbent motor choices are outside this study's selection; initialize a new candidate")
    pairs = hardware_pairs(inputs, requirement)
    incumbent_key = _physical_key(best)
    info = diagnostics if diagnostics is not None else {}
    info.update({"block_trials": 0, "geometry_rejected_trials": 0,
                 "duplicate_trials": 0, "capacity_challenge_probes": 0,
                 "capacity_challenge_probe_limit_reached": False,
                 "fallback_to_incumbent": False, "fallback_reason": "",
                 "load_evaluation_ready": False})
    if cycle % 16 == 0:
        challenge = _strongest_fitting_hardware(
            best, inputs, pairs, reverse=bool((cycle//16) % 2), diagnostics=info,
            requirement=requirement)
        if _physical_key(challenge) != incumbent_key:
            info["selected_block"] = "coordinated_capacity_challenge"
            info["load_evaluation_ready"] = True
            return challenge

    restart = cycle % 10 == 0

    def propose(value, low, high):
        if low == high:
            return float(low)
        if restart:
            return float(rng.uniform(low, high))
        scale = .12 if cycle < requirement.cycles*.75 else .04
        return float(np.clip(value + rng.normal(0, scale*(high-low)), low, high))

    seen = {incumbent_key}
    for attempt in range(24):
        info["block_trials"] += 1
        number = (int(rng.choice(requirement.allowed_layouts))
                  if len(requirement.allowed_layouts) > 1 and rng.random() < .15
                  else best.layout_number)
        candidate = replace(best, layout_number=number,
                            motor_offsets_m=active_offsets(number, best.motor_offsets_m))
        block = "lengths" if restart and attempt == 0 else rng.choice(("lengths", "placement", "hardware"))
        if block == "lengths":
            reach, allocation = length_coordinates(best, requirement, number)
            lengths = lengths_from_coordinates(requirement, number,
                        propose(reach, *coordinate_bounds(requirement, number)),
                        propose(allocation, 0., 1.))
            bounds = placement_delta_bounds(lengths, requirement)
            offsets = tuple(float(np.clip(value, -bound, bound))
                            for value, bound in zip(candidate.motor_offsets_m, bounds))
            candidate = replace(candidate, lengths_m=lengths,
                                motor_offsets_m=active_offsets(number, offsets))
        elif block == "placement":
            bounds = placement_delta_bounds(candidate.lengths_m, requirement)
            active = np.flatnonzero(active_offsets(number, np.ones(4)))
            if not len(active):
                continue
            coordinate = int(rng.choice(active))
            offsets = list(candidate.motor_offsets_m)
            offsets[coordinate] = propose(offsets[coordinate], -bounds[coordinate], bounds[coordinate])
            candidate = replace(candidate, motor_offsets_m=tuple(offsets))
        else:
            joint = int(rng.integers(6))
            options = _joint_pairs(pairs, joint, requirement)
            if rng.random() < .75:
                reference = (inputs.motor_by_id[candidate.motor_ids[joint]],
                             inputs.reducer_by_id[candidate.reducer_ids[joint]])
                neighbors = _nearby_hardware_options(tuple(options), reference)
                pool = neighbors[:min(48, len(neighbors))]
                pair = pool[int(rng.integers(len(pool)))]
            else:
                pair = options[int(rng.integers(len(options)))]
            candidate = _with_hardware(candidate, joint, pair)
        key = _physical_key(candidate)
        if key in seen:
            info["duplicate_trials"] += 1
            continue
        seen.add(key)
        fitted = _compile_candidate(candidate, inputs)
        if fitted is not None:
            info["selected_block"] = str(block)
            info["load_evaluation_ready"] = True
            return fitted
        info["geometry_rejected_trials"] += 1
    # This fallback is intentionally explicit: a small/saturated catalog can
    # have no distinct feasible neighbor within the bounded proposal work.
    info["fallback_to_incumbent"] = True
    info["fallback_reason"] = "No distinct supported neighbor found in 24 bounded geometry trials"
    incumbent = _compile_candidate(best, inputs)
    info["load_evaluation_ready"] = incumbent is not None
    return incumbent if incumbent is not None else best

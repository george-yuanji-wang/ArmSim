"""Human-editable CSV inputs and their unit-aware validation.

CSV column names carry units.  Input objects retain those names; convenience
properties convert lengths and payloads to SI for the simulation code.
"""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass, is_dataclass, replace
import json
import math
from pathlib import Path
import re
from typing import Any

from . import settings


DATA_DIR = Path(__file__).resolve().parent / "data"
STRUCTURAL_BODIES = (
    "shoulder_link", "upper_arm_link", "forearm_link",
    "wrist_1_link", "wrist_2_link", "tool_flange_link",
)
DEFAULT_CYCLES = settings.DEFAULT_CYCLES
DEFAULT_CHECKPOINT_EVERY = settings.DEFAULT_CHECKPOINT_EVERY


class InputError(ValueError):
    """A CSV input cannot be interpreted as a valid design project."""


@dataclass(frozen=True)
class Motor:
    motor_id: str
    name: str
    rated_torque_Nm: float
    rated_speed_rpm: float
    stall_torque_Nm: float | None
    no_load_speed_rpm: float | None
    max_speed_rpm: float | None
    mass_kg: float
    radius_mm: float
    length_mm: float
    rotor_inertia_kg_m2: float | None
    rated_power_W: float | None = None
    peak_torque_Nm: float | None = None
    peak_duration_s: float | None = None
    capacity_model: str = "legacy_linear"
    rotor_inertia_upper_bound_kg_m2: float | None = None
    rotor_inertia_basis: str = "published"
    rated_voltage_V: float | None = None
    rated_current_A: float | None = None
    price_usd: float | None = None
    source_url: str = ""
    operating_voltage_V: float | None = None
    peak_torque_speed_curve: tuple[tuple[float, float], ...] = ()
    peak_curve_basis: str = ""

    def __post_init__(self):
        points = tuple(tuple(float(value) for value in point)
                       for point in self.peak_torque_speed_curve)
        if any(len(point) != 2 or any(not math.isfinite(v) or v < 0 for v in point)
               for point in points):
            raise ValueError("peak_torque_speed_curve needs finite nonnegative rpm/torque pairs")
        if points and (points[0][0] != 0 or any(
                right[0] <= left[0] or right[1] > left[1]
                for left, right in zip(points, points[1:]))):
            raise ValueError("peak_torque_speed_curve must start at zero rpm and have increasing speed/nonincreasing torque")
        if points and self.peak_torque_Nm is not None and any(
                torque > self.peak_torque_Nm + 1e-12 for _, torque in points):
            raise ValueError("peak_torque_speed_curve exceeds the published peak torque")
        object.__setattr__(self, "peak_torque_speed_curve", points)

    @property
    def evaluation_rotor_inertia_kg_m2(self) -> float:
        """Use a published value or an explicitly recorded physical upper bound."""
        value = (self.rotor_inertia_kg_m2 if self.rotor_inertia_kg_m2 is not None
                 else self.rotor_inertia_upper_bound_kg_m2)
        if value is None:
            raise ValueError(f"Motor {self.motor_id} has no supported rotor inertia")
        return value

    @property
    def id(self) -> str:
        return self.motor_id

    @property
    def radius_m(self) -> float:
        return self.radius_mm / 1000

    @property
    def length_m(self) -> float:
        return self.length_mm / 1000

    @property
    def rated_torque_nm(self) -> float:
        return self.rated_torque_Nm

    @property
    def stall_torque_nm(self) -> float | None:
        return self.stall_torque_Nm

    @property
    def peak_torque_nm(self) -> float:
        return self.peak_torque_Nm or self.rated_torque_Nm

    @property
    def evaluation_speed_limit_rpm(self) -> float:
        """Speed range supported by the chosen continuous-rating model.

        A rated-continuous model never extrapolates above the published rated
        operating speed. This is an evaluation restriction, not an invented
        mechanical maximum speed.
        """
        if self.capacity_model == "rated_continuous":
            return min(self.rated_speed_rpm, self.max_speed_rpm
                       if self.max_speed_rpm is not None else self.rated_speed_rpm)
        if self.max_speed_rpm is None:
            raise ValueError(f"Legacy motor {self.motor_id} needs a mechanical speed limit")
        return self.max_speed_rpm


@dataclass(frozen=True)
class Reducer:
    reducer_id: str
    name: str
    reduction_ratio: float
    mass_kg: float
    radius_mm: float
    length_mm: float
    continuous_output_torque_Nm: float
    peak_output_torque_Nm: float
    max_input_torque_Nm: float | None
    max_input_speed_rpm: float
    rated_input_speed_rpm: float | None = None
    allowed_joints: tuple[int, ...] = ()

    def __post_init__(self):
        joints = tuple(self.allowed_joints)
        if len(set(joints)) != len(joints) or any(
                isinstance(j, bool) or not isinstance(j, int) or not 1 <= j <= 6 for j in joints):
            raise ValueError("allowed_joints must contain distinct joint numbers from 1 through 6")
        object.__setattr__(self, "allowed_joints", joints)

    @property
    def id(self) -> str:
        return self.reducer_id

    @property
    def radius_m(self) -> float:
        return self.radius_mm / 1000

    @property
    def length_m(self) -> float:
        return self.length_mm / 1000

    @property
    def continuous_output_torque_nm(self) -> float:
        return self.continuous_output_torque_Nm

    @property
    def peak_output_torque_nm(self) -> float:
        return self.peak_output_torque_Nm

    @property
    def max_input_torque_nm(self) -> float | None:
        return self.max_input_torque_Nm

    @property
    def output_torque_limit_Nm(self) -> float:
        """Published output limit, additionally capped by any known input limit."""
        if self.max_input_torque_Nm is None:
            return self.continuous_output_torque_Nm
        return min(self.continuous_output_torque_Nm,
                   self.max_input_torque_Nm*self.reduction_ratio)

    @property
    def evaluation_input_speed_limit_rpm(self) -> float:
        """Avoid extending a speed-specific rated torque beyond its reference."""
        return min(self.max_input_speed_rpm, self.rated_input_speed_rpm
                   if self.rated_input_speed_rpm is not None else self.max_input_speed_rpm)


@dataclass(frozen=True)
class Requirement:
    project_id: str
    payload_lower_g: float
    payload_upper_g: float
    reach_lower_mm: float
    reach_upper_mm: float
    elevation_lower_deg: float = settings.ELEVATION_LOWER_DEG
    elevation_upper_deg: float = settings.ELEVATION_UPPER_DEG
    joint_speed_rad_s: float = settings.JOINT_SPEED_RAD_S
    joint_accel_rad_s2: float = settings.JOINT_ACCEL_RAD_S2
    l1_lower_mm: float | None = None
    l1_upper_mm: float | None = None
    l2_lower_mm: float | None = None
    l2_upper_mm: float | None = None
    l3_lower_mm: float | None = None
    l3_upper_mm: float | None = None
    motor_offset_max_mm: float | None = None
    seed: int = settings.DEFAULT_SEED
    cycles: int = DEFAULT_CYCLES
    checkpoint_every: int = DEFAULT_CHECKPOINT_EVERY
    allowed_layouts: tuple[int, ...] = settings.ALLOWED_LAYOUTS
    joint_speed_limits_rad_s: tuple[float, ...] | None = None
    wrist2_length_m: float = .07
    motor_candidates_by_joint: tuple[tuple[str, ...], ...] | None = None
    shared_wrist_motor_id: str | None = None
    reducer_load_policy: str = "continuous"
    motor_load_policy: str = "continuous"

    def __post_init__(self):
        if self.motor_load_policy not in {"continuous", "hold_and_peak_envelope"}:
            raise ValueError("motor_load_policy must be continuous or hold_and_peak_envelope")
        if not isinstance(self.reducer_load_policy, str) or self.reducer_load_policy not in {"continuous", "hold_and_repeated_peak"}:
            raise ValueError("reducer_load_policy must be continuous or hold_and_repeated_peak")
        if self.motor_candidates_by_joint is not None:
            groups = self.motor_candidates_by_joint
            if not isinstance(groups, (tuple, list)) or len(groups) != 3:
                raise ValueError("motor_candidates_by_joint needs three candidate lists for J1, J2 and J3")
            normalized = []
            for joint, candidates in enumerate(groups, 1):
                if not isinstance(candidates, (tuple, list)) or not candidates or any(
                        not isinstance(value, str) or not value.strip() for value in candidates):
                    raise ValueError(f"motor_candidates_by_joint J{joint} needs a nonempty list of motor IDs")
                candidates = tuple(value.strip() for value in candidates)
                if len(candidates) != len(set(candidates)):
                    raise ValueError(f"motor_candidates_by_joint J{joint} contains duplicate motor IDs")
                normalized.append(candidates)
            object.__setattr__(self, "motor_candidates_by_joint", tuple(normalized))
        if self.shared_wrist_motor_id is not None:
            if not isinstance(self.shared_wrist_motor_id, str) or not self.shared_wrist_motor_id.strip():
                raise ValueError("shared_wrist_motor_id must be a nonempty motor ID")
            object.__setattr__(self, "shared_wrist_motor_id", self.shared_wrist_motor_id.strip())
        if not math.isfinite(self.wrist2_length_m) or self.wrist2_length_m <= 0:
            raise ValueError("wrist2_length_m must be positive and finite")
        if self.joint_speed_limits_rad_s is not None:
            speeds = tuple(float(value) for value in self.joint_speed_limits_rad_s)
            if len(speeds) != 6 or any(not math.isfinite(value) or value <= 0 for value in speeds):
                raise ValueError("joint_speed_limits_rad_s needs six positive finite speeds")
            object.__setattr__(self, "joint_speed_limits_rad_s", speeds)
        for name, value in settings.automatic_dimensions(self.reach_upper_mm).items():
            if getattr(self, name) is None:
                object.__setattr__(self, name, value)

    @property
    def joint_speeds_rad_s(self) -> tuple[float, ...]:
        return self.joint_speed_limits_rad_s or (self.joint_speed_rad_s,) * 6

    def allows_motor(self, joint_number: int, motor_id: str) -> bool:
        """One-based motor scope; unconfigured legacy studies stay unrestricted."""
        if isinstance(joint_number, bool) or not isinstance(joint_number, int) or not 1 <= joint_number <= 6:
            raise ValueError("joint_number must be an integer from 1 through 6")
        if joint_number <= 3 and self.motor_candidates_by_joint is not None:
            return motor_id in self.motor_candidates_by_joint[joint_number-1]
        if joint_number >= 4 and self.shared_wrist_motor_id is not None:
            return motor_id == self.shared_wrist_motor_id
        return True

    @property
    def payload_lower_kg(self) -> float:
        return self.payload_lower_g / 1000

    @property
    def payload_upper_kg(self) -> float:
        return self.payload_upper_g / 1000

    @property
    def payload_bounds_kg(self) -> tuple[float, float]:
        return self.payload_lower_kg, self.payload_upper_kg

    @property
    def reach_lower_m(self) -> float:
        return self.reach_lower_mm / 1000

    @property
    def reach_upper_m(self) -> float:
        return self.reach_upper_mm / 1000

    @property
    def reach_bounds_m(self) -> tuple[float, float]:
        return self.reach_lower_m, self.reach_upper_m

    @property
    def length_bounds_m(self) -> tuple[tuple[float, float], ...]:
        return tuple(
            (getattr(self, f"l{i}_lower_mm") / 1000,
             getattr(self, f"l{i}_upper_mm") / 1000)
            for i in (1, 2, 3)
        )

    @property
    def motor_offset_max_m(self) -> float:
        return self.motor_offset_max_mm / 1000


@dataclass(frozen=True)
class LinkBounds:
    body_name: str
    mass_lower_kg: float
    mass_upper_kg: float
    com_fraction_lower: float
    com_fraction_upper: float
    spread_fraction_lower: float
    spread_fraction_upper: float
    radius_mm: float

    @property
    def radius_m(self) -> float:
        return self.radius_mm / 1000


@dataclass(frozen=True)
class Inputs:
    motors: tuple[Motor, ...]
    reducers: tuple[Reducer, ...]
    requirements: tuple[Requirement, ...]
    links: tuple[LinkBounds, ...]

    @property
    def link_bounds(self) -> tuple[LinkBounds, ...]:
        return self.links

    @property
    def motor_by_id(self) -> dict[str, Motor]:
        return {entry.motor_id: entry for entry in self.motors}

    @property
    def reducer_by_id(self) -> dict[str, Reducer]:
        return {entry.reducer_id: entry for entry in self.reducers}

MOTOR_COLUMNS = (
    "motor_id", "name", "rated_torque_Nm", "rated_speed_rpm", "rated_power_W",
    "stall_torque_Nm", "no_load_speed_rpm", "max_speed_rpm", "mass_kg",
    "radius_mm", "length_mm", "rotor_inertia_kg_m2", "peak_torque_Nm", "peak_duration_s",
)
REDUCER_COLUMNS = (
    "reducer_id", "name", "reduction_ratio", "mass_kg", "radius_mm", "length_mm",
    "continuous_output_torque_Nm", "peak_output_torque_Nm", "max_input_torque_Nm",
    "max_input_speed_rpm",
)
REQUIREMENT_COLUMNS = (
    "project_id", "payload_lower_g", "payload_upper_g", "reach_lower_mm", "reach_upper_mm",
)
LINK_COLUMNS = tuple(LinkBounds.__dataclass_fields__)


def _rows(path: Path, required: tuple[str, ...]) -> list[tuple[int, dict[str, str]]]:
    if not path.is_file():
        raise InputError(f"Missing input file: {path}")
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            headers = reader.fieldnames
            if not headers:
                raise InputError(f"{path.name}: a CSV header is required")
            headers = [header.strip() for header in headers]
            if len(headers) != len(set(headers)):
                raise InputError(f"{path.name}: duplicate column names")
            missing = sorted(set(required) - set(headers))
            if missing:
                raise InputError(f"{path.name}: missing columns: {', '.join(missing)}")
            reader.fieldnames = headers
            result = []
            for line, row in enumerate(reader, start=2):
                if None in row:
                    raise InputError(f"{path.name}:{line}: too many cells; quote text containing commas")
                clean = {key: (value or "").strip() for key, value in row.items()}
                if any(clean.values()):
                    result.append((line, clean))
            return result
    except (OSError, UnicodeError, csv.Error) as exc:
        raise InputError(f"Cannot read {path}: {exc}") from exc


def _fail(path: Path, line: int, message: str) -> None:
    raise InputError(f"{path.name}:{line}: {message}")


def _number(row: dict[str, str], key: str, path: Path, line: int, *,
            minimum: float | None = None, positive: bool = False,
            optional: bool = False) -> float | None:
    text = row.get(key, "")
    if not text and optional:
        return None
    try:
        value = float(text)
    except ValueError:
        _fail(path, line, f"{key} must contain a number")
    if not math.isfinite(value):
        _fail(path, line, f"{key} must be finite")
    if positive and value <= 0:
        _fail(path, line, f"{key} must be greater than zero")
    if minimum is not None and value < minimum:
        _fail(path, line, f"{key} must be at least {minimum:g}")
    return value


def _identifier(row: dict[str, str], key: str, path: Path, line: int,
                seen: set[str], *, safe_path: bool = False) -> str:
    value = row.get(key, "")
    if not value:
        _fail(path, line, f"{key} cannot be blank")
    if safe_path and not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", value):
        _fail(path, line, f"{key} must use letters, numbers, underscores, hyphens or dots")
    if value in seen:
        _fail(path, line, f"duplicate {key}: {value}")
    seen.add(value)
    return value


def _bounds(row: dict[str, str], lower: str, upper: str, path: Path, line: int,
            *, minimum: float = 0, positive: bool = False,
            maximum: float | None = None) -> tuple[float, float]:
    lo = _number(row, lower, path, line, minimum=minimum, positive=positive)
    hi = _number(row, upper, path, line, minimum=minimum, positive=positive)
    if lo > hi:
        _fail(path, line, f"{lower} exceeds {upper}; equal bounds are allowed")
    if maximum is not None and hi > maximum:
        _fail(path, line, f"{upper} must not exceed {maximum:g}")
    return lo, hi


def load_motors(path: str | Path) -> tuple[Motor, ...]:
    path = Path(path)
    optional = {"rated_power_W", "peak_torque_Nm", "peak_duration_s"}
    rating_fields = {"stall_torque_Nm", "no_load_speed_rpm", "max_speed_rpm"}
    required = tuple(key for key in MOTOR_COLUMNS if key not in optional | rating_fields)
    result, seen = [], set()
    for line, row in _rows(path, required):
        motor_id = _identifier(row, "motor_id", path, line, seen)
        if not row.get("name"):
            _fail(path, line, "name cannot be blank")
        capacity_model = row.get("capacity_model") or "legacy_linear"
        if capacity_model not in {"legacy_linear", "rated_continuous"}:
            _fail(path, line, "capacity_model must be legacy_linear or rated_continuous")
        numbers = {
            key: _number(row, key, path, line, positive=True)
            for key in required if key not in {"motor_id", "name", "rotor_inertia_kg_m2"}
        }
        numbers["rotor_inertia_kg_m2"] = _number(
            row, "rotor_inertia_kg_m2", path, line, minimum=0, optional=True)
        numbers["rotor_inertia_upper_bound_kg_m2"] = _number(
            row, "rotor_inertia_upper_bound_kg_m2", path, line, positive=True, optional=True)
        inertia_basis = row.get("rotor_inertia_basis") or "published"
        if inertia_basis not in {"published", "mass_radius_upper_bound"}:
            _fail(path, line, "rotor_inertia_basis must be published or mass_radius_upper_bound")
        if numbers["rotor_inertia_kg_m2"] is None:
            bound = numbers["rotor_inertia_upper_bound_kg_m2"]
            if inertia_basis != "mass_radius_upper_bound" or bound is None:
                _fail(path, line, "rotor_inertia_kg_m2 needs a published value or explicit mass_radius_upper_bound")
            physical_bound = numbers["mass_kg"] * (numbers["radius_mm"] / 1000) ** 2
            if bound < physical_bound * (1 - 1e-9):
                _fail(path, line, "rotor inertia upper bound is below mass_kg * radius_m^2")
        elif inertia_basis != "published":
            _fail(path, line, "a published rotor_inertia_kg_m2 must use published inertia basis")
        for key in ("rated_voltage_V", "rated_current_A", "price_usd", "operating_voltage_V"):
            numbers[key] = _number(row, key, path, line, positive=True, optional=True)
        numbers.update({key: _number(row, key, path, line, positive=True, optional=True)
                        for key in optional})
        numbers.update({key: _number(row, key, path, line, positive=True,
                                    optional=capacity_model == "rated_continuous")
                        for key in rating_fields})
        if numbers["max_speed_rpm"] is not None and numbers["rated_speed_rpm"] > numbers["max_speed_rpm"]:
            _fail(path, line, "rated_speed_rpm exceeds max_speed_rpm")
        if numbers["no_load_speed_rpm"] is not None and numbers["rated_speed_rpm"] > numbers["no_load_speed_rpm"]:
            _fail(path, line, "rated_speed_rpm exceeds no_load_speed_rpm")
        if numbers["stall_torque_Nm"] is not None and numbers["stall_torque_Nm"] < numbers["rated_torque_Nm"]:
            _fail(path, line, "stall_torque_Nm is below rated_torque_Nm")
        peak = numbers["peak_torque_Nm"]
        if peak is not None and peak < numbers["rated_torque_Nm"]:
            _fail(path, line, "peak_torque_Nm is below rated_torque_Nm")
        if (capacity_model == "legacy_linear" and peak is not None
                and peak > numbers["rated_torque_Nm"] and numbers["peak_duration_s"] is None):
            _fail(path, line, "peak_duration_s is required when peak torque exceeds rated torque")
        result.append(Motor(motor_id=motor_id, name=row["name"],
                            capacity_model=capacity_model, rotor_inertia_basis=inertia_basis,
                            source_url=row.get("source_url", ""), **numbers))
    return tuple(result)


def load_reducers(path: str | Path) -> tuple[Reducer, ...]:
    path = Path(path)
    result, seen = [], set()
    for line, row in _rows(path, REDUCER_COLUMNS):
        reducer_id = _identifier(row, "reducer_id", path, line, seen)
        if not row.get("name"):
            _fail(path, line, "name cannot be blank")
        numbers = {key: _number(row, key, path, line, positive=True,
                               optional=key == "max_input_torque_Nm")
                   for key in REDUCER_COLUMNS if key not in {"reducer_id", "name"}}
        numbers["rated_input_speed_rpm"] = _number(row, "rated_input_speed_rpm", path,
                                                  line, positive=True, optional=True)
        if numbers["reduction_ratio"] < 1:
            _fail(path, line, "reduction_ratio must be at least 1")
        if numbers["peak_output_torque_Nm"] < numbers["continuous_output_torque_Nm"]:
            _fail(path, line, "peak_output_torque_Nm is below continuous_output_torque_Nm")
        text = row.get("allowed_joints", "")
        try:
            joints = tuple(int(value.strip()) for value in text.split(";") if value.strip())
            reducer = Reducer(reducer_id=reducer_id, name=row["name"], allowed_joints=joints, **numbers)
        except ValueError as exc:
            _fail(path, line, f"invalid allowed_joints: {exc}")
        result.append(reducer)
    return tuple(result)


def load_requirements(path: str | Path = DATA_DIR / "requirement.csv") -> tuple[Requirement, ...]:
    path = Path(path)
    result, seen = [], set()
    for line, row in _rows(path, REQUIREMENT_COLUMNS):
        project_id = _identifier(row, "project_id", path, line, seen, safe_path=True)
        values: dict[str, Any] = {"project_id": project_id}
        for prefix, minimum, positive in (
                ("payload", 0, False), ("reach", 0, True)):
            unit = "g" if prefix == "payload" else "mm"
            lower, upper = f"{prefix}_lower_{unit}", f"{prefix}_upper_{unit}"
            values[lower], values[upper] = _bounds(
                row, lower, upper, path, line, minimum=minimum, positive=positive)
        result.append(Requirement(**values))
    return tuple(result)


def load_link_bounds(path: str | Path = DATA_DIR / "link_bounds.csv") -> tuple[LinkBounds, ...]:
    path = Path(path)
    result, seen = {}, set()
    for line, row in _rows(path, LINK_COLUMNS):
        body_name = _identifier(row, "body_name", path, line, seen)
        if body_name not in STRUCTURAL_BODIES:
            _fail(path, line, f"unknown body_name: {body_name}")
        values: dict[str, Any] = {"body_name": body_name}
        values["mass_lower_kg"], values["mass_upper_kg"] = _bounds(
            row, "mass_lower_kg", "mass_upper_kg", path, line, positive=True)
        for prefix, maximum in (("com", 1), ("spread", .25)):
            lower, upper = f"{prefix}_fraction_lower", f"{prefix}_fraction_upper"
            values[lower], values[upper] = _bounds(row, lower, upper, path, line, maximum=maximum)
        center = min(max(.5, values["com_fraction_lower"]), values["com_fraction_upper"])
        if values["spread_fraction_lower"] > center * (1 - center):
            _fail(path, line, "spread lower bound cannot satisfy kappa <= lambda*(1-lambda) anywhere in the COM range")
        values["radius_mm"] = _number(row, "radius_mm", path, line, positive=True)
        result[body_name] = LinkBounds(**values)
    missing = set(STRUCTURAL_BODIES) - set(result)
    if missing:
        raise InputError(f"{path.name}: missing structural bodies: {', '.join(sorted(missing))}")
    return tuple(result[name] for name in STRUCTURAL_BODIES)


def load_inputs(data_dir: str | Path = DATA_DIR, *, require_catalogs: bool = True) -> Inputs:
    data_dir = Path(data_dir)
    inputs = Inputs(
        motors=load_motors(data_dir / "motor.csv"),
        reducers=load_reducers(data_dir / "reducer.csv"),
        requirements=load_requirements(data_dir / "requirement.csv"),
        links=load_link_bounds(data_dir / "link_bounds.csv"),
    )
    curve_path = data_dir / "motor_curve.csv"
    if curve_path.is_file():
        groups: dict[str, list[tuple[float, float]]] = {}
        bases: dict[str, str] = {}
        for line, row in _rows(curve_path, ("motor_id", "speed_rpm", "available_torque_Nm", "basis")):
            motor_id = row["motor_id"]
            if motor_id not in inputs.motor_by_id:
                _fail(curve_path, line, f"unknown motor_id: {motor_id}")
            groups.setdefault(motor_id, []).append((
                _number(row, "speed_rpm", curve_path, line, minimum=0),
                _number(row, "available_torque_Nm", curve_path, line, minimum=0)))
            if not row["basis"] or (motor_id in bases and bases[motor_id] != row["basis"]):
                _fail(curve_path, line, "each motor curve needs one consistent nonempty basis")
            bases[motor_id] = row["basis"]
        try:
            inputs = replace(inputs, motors=tuple(replace(motor,
                peak_torque_speed_curve=tuple(groups[motor.motor_id]),
                peak_curve_basis=bases[motor.motor_id]) if motor.motor_id in groups else motor
                for motor in inputs.motors))
        except ValueError as exc:
            raise InputError(f"motor_curve.csv: {exc}") from exc
    study_path = data_dir / "study.json"
    if study_path.is_file():
        try:
            study = json.loads(study_path.read_text(encoding="utf-8-sig"))
            supported = {"joint_speed_limits_deg_s", "joint_accel_rad_s2", "wrist2_length_mm",
                         "motor_candidates_by_joint", "shared_wrist_motor_id", "reducer_load_policy",
                         "motor_load_policy"}
            if not isinstance(study, dict) or set(study) - supported:
                raise ValueError("supported fields are " + ", ".join(sorted(supported)))
            updates = {}
            if "motor_load_policy" in study:
                updates["motor_load_policy"] = study["motor_load_policy"]
            if "reducer_load_policy" in study:
                updates["reducer_load_policy"] = study["reducer_load_policy"]
            if "joint_speed_limits_deg_s" in study:
                updates["joint_speed_limits_rad_s"] = tuple(
                    math.radians(float(value)) for value in study["joint_speed_limits_deg_s"])
            if "joint_accel_rad_s2" in study:
                acceleration = float(study["joint_accel_rad_s2"])
                if not math.isfinite(acceleration) or acceleration <= 0:
                    raise ValueError("joint_accel_rad_s2 must be positive and finite")
                updates["joint_accel_rad_s2"] = acceleration
            if "wrist2_length_mm" in study:
                updates["wrist2_length_m"] = float(study["wrist2_length_mm"]) / 1000
            if "motor_candidates_by_joint" in study:
                groups = study["motor_candidates_by_joint"]
                if not isinstance(groups, dict) or set(groups) != {"J1", "J2", "J3"}:
                    raise ValueError("motor_candidates_by_joint must map exactly J1, J2 and J3 to candidate lists")
                updates["motor_candidates_by_joint"] = tuple(groups[f"J{joint}"] for joint in range(1, 4))
            if "shared_wrist_motor_id" in study:
                motor_id = study["shared_wrist_motor_id"]
                if not isinstance(motor_id, str) or not motor_id.strip():
                    raise ValueError("shared_wrist_motor_id must be a nonempty motor ID")
                updates["shared_wrist_motor_id"] = motor_id
            inputs = replace(inputs, requirements=tuple(replace(row, **updates) for row in inputs.requirements))
            known = set(inputs.motor_by_id)
            for requirement in inputs.requirements:
                configured = {motor_id for group in requirement.motor_candidates_by_joint or ()
                              for motor_id in group}
                if requirement.shared_wrist_motor_id is not None:
                    configured.add(requirement.shared_wrist_motor_id)
                unknown = configured - known
                if unknown:
                    raise ValueError("motor selection references unknown motor IDs: " + ", ".join(sorted(unknown)))
        except (TypeError, ValueError, OSError) as exc:
            raise InputError(f"study.json: {exc}") from exc
    if require_catalogs:
        missing = [name for name, entries in (("motor.csv", inputs.motors), ("reducer.csv", inputs.reducers)) if not entries]
        if missing:
            raise InputError("Fill at least one real catalog row in " + " and ".join(missing) + " before optimization; templates contain no invented hardware.")
    return inputs


def to_jsonable(value: Any) -> Any:
    """Convert dataclasses, paths and array-like values for provenance files."""
    if is_dataclass(value) and not isinstance(value, type):
        return to_jsonable(asdict(value))
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [to_jsonable(item) for item in value]
    if hasattr(value, "tolist"):
        return to_jsonable(value.tolist())
    return value

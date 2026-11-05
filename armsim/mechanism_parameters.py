"""Dimensions which may tune an existing ArmSim mounting template.

Offsets are local adjustments to the approved motor placement, not arbitrary
component positions.  Reducer length is its complete housing/bearing envelope.
"""

from dataclasses import dataclass
import numpy as np


class GeometryError(ValueError):
    """The chosen values do not fit the existing mounting geometry."""


@dataclass(frozen=True)
class MechanismParameters:
    l1_length: float
    motor_masses: tuple[float, ...]
    gear_masses: tuple[float, ...]
    motor_radii: tuple[float, ...]
    gear_radii: tuple[float, ...]
    motor_lengths: tuple[float, ...]
    gear_lengths: tuple[float, ...]
    motor_offsets: tuple[float, float, float, float] = (0., 0., 0., 0.)

    def __post_init__(self):
        if not np.isfinite(self.l1_length) or self.l1_length <= 0:
            raise GeometryError("L1 must be positive and finite")
        for name in ("motor_masses", "gear_masses", "motor_radii", "gear_radii",
                     "motor_lengths", "gear_lengths"):
            values = np.asarray(getattr(self, name), dtype=float)
            if values.shape != (6,) or not np.isfinite(values).all() or np.any(values <= 0):
                raise GeometryError(f"{name} needs six positive finite values")
        offsets = np.asarray(self.motor_offsets, dtype=float)
        if offsets.shape != (4,) or not np.isfinite(offsets).all():
            raise GeometryError("motor_offsets needs four finite local adjustments")

    @classmethod
    def nominal(cls, layout=None):
        radii = (.037, .035, .033, .026, .024, .022)
        return cls(.32, (.90, .90, .65, .22, .18, .15),
                   (.30, .35, .28, .12, .10, .08), radii,
                   tuple(radius+.002 for radius in radii),
                   (.020, .020, .020, .018, .018, .018), (.030,)*6)

"""Tool mobility at the actual pose, including singular configurations."""

import mujoco
import numpy as np


def tool_jacobian(model, data):
    """World-frame [linear velocity; angular velocity] at tool_frame."""
    jacobian = np.zeros((6, model.nv))
    mujoco.mj_jacSite(model, data, jacobian[:3], jacobian[3:], model.site('tool_frame').id)
    return jacobian


def tool_mobility(model, data):
    # Normalize translation by L1=0.32 m for conditioning, rather than mixing
    # meters and radians. Row scaling preserves exact rank.
    jacobian = tool_jacobian(model, data)
    jacobian[:3] /= 0.32
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    tolerance = max(singular_values[0], 1.)*1e-8
    rank = int(np.count_nonzero(singular_values > tolerance))
    ratio = float(singular_values[-1]/singular_values[0]) if singular_values[0] else 0.
    return {'rank': rank, 'near_singular': rank < 6 or ratio < 1e-3,
            'reciprocal_condition': ratio, 'scaled_singular_values': singular_values.tolist()}

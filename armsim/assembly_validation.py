"""Exact local moving/fixed fit checks for parameterized base assemblies.

This checks each joint's own mounting at assembly zero. It does not test global
self-collision, swept travel, or mechanical joint stops.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import mujoco
import numpy as np

from armsim.mechanism_parameters import GeometryError


def _world_half_extents(model, data):
    """Conservative AABBs prune pairs before the exact MuJoCo distance query."""
    half = np.repeat(model.geom_rbound[:, None], 3, axis=1)
    rotations = data.geom_xmat.reshape(-1, 3, 3)
    for kind in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER,
                 mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_SPHERE):
        chosen = model.geom_type == kind
        size = model.geom_size[chosen]
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            half[chosen] = np.einsum('nij,nj->ni', np.abs(rotations[chosen]), size)
        elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            half[chosen] = size[:, :1]
        else:
            axis = np.abs(rotations[chosen, :, 2])
            radial = (np.sqrt(np.maximum(0., 1-axis**2))
                      if kind == mujoco.mjtGeom.mjGEOM_CYLINDER else np.ones_like(axis))
            half[chosen] = radial*size[:, :1]+axis*size[:, 1:2]
    return half


def validate_local_assembly(model, drives: Sequence[Mapping], *, tolerance=1e-7):
    """Reject overlap between each joint's child supports and fixed mounting.

    Grey flanges, their own output supports and the child link's twelve cage
    edges must clear that joint's motor/reducer cases and parent structural
    stock. The bearing/output interface may touch. A small negative tolerance
    absorbs numerical noise, without allowing measurable stock penetration.
    The model's simulation state is not changed.
    """
    if not np.isfinite(tolerance) or tolerance < 0:
        raise ValueError('Assembly tolerance must be finite and nonnegative')
    data = mujoco.MjData(model)
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    half = _world_half_extents(model, data)
    centers = data.geom_xpos
    for drive in drives:
        j = int(drive['joint'])
        try:
            parent = model.body(drive['motor_mount']).id
            child = model.body(drive['output_mount']).id
            fixed = np.flatnonzero((model.geom_bodyid == parent) &
                                   np.isin(model.geom_group, (0, 4))).tolist()
            fixed += [model.geom(f'{prefix}{j}_{part}').id for prefix, part in (
                ('G', 'housing'), ('G', 'bearing_housing'),
                ('M', 'stator'), ('M', 'end_cap'))]
            moving = [model.geom(f'O{j}_moving_flange').id]
            moving += [model.geom(name).id for name in drive['moving_support_geoms']]
            moving += [i for i in np.flatnonzero(model.geom_bodyid == child)
                       if model.geom(i).name.startswith(drive['output_mount']+'_frame_')]
        except KeyError as exc:
            raise GeometryError(f'J{j} local assembly metadata is incomplete: {exc}') from exc
        moving, fixed = np.unique(moving), np.unique(fixed)
        separation = np.abs(centers[moving, None]-centers[fixed])
        bounds = half[moving, None]+half[fixed]
        # Any actual overlap must overlap on all three world axes. Keep a small
        # buffer so a near-touching bearing interface reaches the exact query.
        possible = np.argwhere(np.all(separation <= bounds+tolerance, axis=2))
        for left, right in possible:
            first, second = int(moving[left]), int(fixed[right])
            distance = float(mujoco.mj_geomDistance(model, data, first, second, .001, None))
            if distance < -tolerance:
                raise GeometryError(
                    f'J{j} local assembly interference: {model.geom(first).name} '
                    f'intersects {model.geom(second).name} by {-distance*1000:.6g} mm')

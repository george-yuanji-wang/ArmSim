"""Report the actual mass properties of each approved structural assembly.

The mechanics builder assigns aluminium stock volume to the frame edges,
mounts and output flanges. Small input-drive parts retain its explicit mass
estimates. These values are inferred by MuJoCo, rather than optimized as
independent mass, COM or inertia targets.
"""

from dataclasses import replace

import mujoco
import numpy as np

from armsim.catalog import LINKS
from armsim.mechanism_parameters import GeometryError
from .inputs import STRUCTURAL_BODIES


STRUCTURAL_MASS_MODEL = (
    "Approved aluminium edge/frame/output stock at 2700 kg/m3, plus the base "
    "builder's explicit small transmission-part masses. Catalog motor/reducer "
    "masses are separate; structural mass, 3D COM and full inertia follow geometry."
)


def structural_properties(model):
    """Return base and link rigid-assembly properties, excluding catalog cases.

    Each output flange belongs to its moving link. Fixed noncatalog attachment
    bodies are combined into that link using the parallel-axis theorem. COM and
    inertia are in the named link's local XYZ frame; inertia is about that COM.
    The supplied model is not modified.
    """
    data = mujoco.MjData(model)
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    roots = {model.body(name).id: name for name in LINKS}
    members = {name: [] for name in LINKS}
    for body in range(1, model.nbody):
        name = model.body(body).name
        if name.startswith(("motor_", "reducer_", "gearbox_", "payload")):
            continue
        parent = body
        excluded = False
        while parent not in roots and parent:
            if model.body(parent).name.startswith(("motor_", "reducer_", "gearbox_", "payload")):
                excluded = True
                break
            parent = int(model.body_parentid[parent])
        if not excluded and parent in roots and model.body_mass[body] > 0:
            members[roots[parent]].append(body)

    rows = []
    for body, name in roots.items():
        ids = members[name]
        masses = model.body_mass[ids]
        mass = float(masses.sum())
        if not mass > 0:
            raise GeometryError(f"No structural stock mass was generated for {name}")
        positions = data.xipos[ids]
        center = (masses[:, None]*positions).sum(axis=0)/mass
        inertia_world = np.zeros((3, 3))
        for index, part in enumerate(ids):
            principal_rotation = data.ximat[part].reshape(3, 3)
            inertia_world += (principal_rotation*model.body_inertia[part])@principal_rotation.T
            delta = positions[index]-center
            inertia_world += masses[index]*(np.eye(3)*float(delta@delta)-np.outer(delta, delta))
        rotation = data.xmat[body].reshape(3, 3)
        center_local = rotation.T@(center-data.xpos[body])
        inertia = rotation.T@inertia_world@rotation
        covariance = (np.eye(3)*np.trace(inertia)/2-inertia)/mass
        rows.append({
            "body_name": name, "mass_kg": mass,
            "com_local_xyz_m": center_local.tolist(),
            "inertia_com_local_kg_m2": inertia.tolist(),
            "longitudinal_variance_m2": float(max(0., covariance[2, 2])),
            "fixed_component_bodies": [model.body(part).name for part in ids],
        })
    return rows


def candidate_from_model(candidate, model):
    """Fill retained serialization fields with measured geometry properties."""
    rows = {row["body_name"]: row for row in structural_properties(model)}
    lengths = (.12, *candidate.lengths_m, candidate.wrist2_length_m, .08)
    masses, coms, spreads = [], [], []
    for name, length in zip(STRUCTURAL_BODIES, lengths):
        row = rows[name]
        masses.append(row["mass_kg"])
        coms.append(row["com_local_xyz_m"][2]/length)
        spreads.append(row["longitudinal_variance_m2"]/length**2)
    return replace(candidate, link_masses_kg=tuple(masses), com_fractions=tuple(coms),
                   spread_fractions=tuple(spreads))


def canonicalize_candidate(candidate, inputs):
    """Resolve valid candidates; leave invalid geometry for evaluator rejection."""
    from .model_factory import build_template_model

    try:
        model, _ = build_template_model(candidate, inputs)
    except GeometryError:
        return candidate
    return candidate_from_model(candidate, model)


resolved_candidate = canonicalize_candidate

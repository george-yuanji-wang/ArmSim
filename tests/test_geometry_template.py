"""The optimizer must tune the approved base mechanism, never invent a mount."""

from dataclasses import replace
import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import numpy as np
import mujoco

from armsim.catalog import LAYOUTS, LINKS, MODEL_DIR, axes_for
from armsim.assembly_validation import validate_local_assembly
from armsim.mechanics import build_mechanics
from armsim.mechanism_parameters import GeometryError, MechanismParameters
from armsim.model_primitives import values


def geometry_tree(layout, parameters=None):
    root = ET.Element("mujoco")
    world = ET.SubElement(root, "worldbody")
    parent = ET.SubElement(world, "body", name=LINKS[0])
    bodies = {LINKS[0]: parent}
    offsets = layout.joint_offsets.copy()
    if parameters is not None:
        offsets[2] = parameters.l1_length
    for index, name in enumerate(LINKS[1:]):
        attrs = {"name": name, "pos": values((0, 0, offsets[index]))}
        if index == 3 and layout.perpendicular_j4:
            attrs["euler"] = values((-np.pi/2, 0, 0))
        child = ET.SubElement(parent, "body", **attrs)
        bodies[name] = child
        parent = child
    manifest = build_mechanics(layout, bodies, axes_for(), parameters=parameters)
    return root, manifest


def named_geoms(root):
    return {geom.get("name"): geom for geom in root.iter("geom") if geom.get("name")}


def geometry_signature(root):
    return {name: canonical_attributes(geom) for name, geom in named_geoms(root).items()}


def canonical_attributes(element):
    numeric = {"fromto", "size", "pos", "quat", "zaxis", "rgba", "mass", "density"}
    return {key: tuple(np.round(np.fromstring(value, sep=" "), 12)) if key in numeric else value
            for key, value in element.attrib.items()}


def connected_to_frame(geoms, starts, frame_names):
    """Geometric edge connectivity includes contacts in the middle of a rail."""
    names = [name for name, geom in geoms.items()
             if geom.get("group") in {"0", "4"} and "fromto" in geom.attrib]
    segments = np.array([np.fromstring(geoms[name].get("fromto"), sep=" ").reshape(2, 3)
                         for name in names])
    direction = segments[:, 1]-segments[:, 0]
    relative = segments[:, :, None, :]-segments[None, None, :, 0, :]
    parameter = np.sum(relative*direction[None, None, :, :], axis=3)/np.sum(direction**2, axis=1)
    residual = relative-parameter[..., None]*direction[None, None, :, :]
    touching = np.any((parameter >= -1e-10) & (parameter <= 1+1e-10)
                      & (np.linalg.norm(residual, axis=3) < 2e-10), axis=1)
    touching |= touching.T
    reached = np.array([name in frame_names for name in names])
    while True:
        expanded = reached | np.any(touching[:, reached], axis=1)
        if np.array_equal(expanded, reached):
            return all(reached[names.index(name)] for name in starts)
        reached = expanded


class GeometryTemplateTests(unittest.TestCase):
    def test_all_nine_approved_base_assemblies_pass_without_geometry_changes(self):
        metadata = json.loads((MODEL_DIR / "layouts.json").read_text(encoding="utf-8"))
        drives = {entry["number"]: entry["drives"] for entry in metadata
                  if entry["wrist_profile"] == "reference"}
        for layout in LAYOUTS:
            with self.subTest(layout=layout.number):
                model = mujoco.MjModel.from_xml_path(str(layout.path))
                arrays = {name: getattr(model, name).copy() for name in (
                    "body_pos", "body_quat", "body_mass", "body_inertia", "geom_pos", "geom_quat", "geom_size")}
                self.assertIsNone(validate_local_assembly(model, drives[layout.number]))
                for name, before in arrays.items():
                    np.testing.assert_array_equal(getattr(model, name), before)

    def test_local_validator_rejects_a_case_intruding_into_its_child_cage(self):
        layout = LAYOUTS[0]
        metadata = json.loads((MODEL_DIR / "layouts.json").read_text(encoding="utf-8"))
        drives = next(entry["drives"] for entry in metadata
                      if entry["number"] == layout.number and entry["wrist_profile"] == "reference")
        model = mujoco.MjModel.from_xml_path(str(layout.path))
        # Keep the approved frame and mounting unchanged, but make the J5 case
        # too large for its own wrist cage. A fallback bracket cannot fix this.
        model.geom_size[model.geom("G5_housing").id, 0] = .08
        with self.assertRaisesRegex(GeometryError, "J5.*G5_housing"):
            validate_local_assembly(model, drives)

    def test_nominal_parameters_reproduce_the_saved_nine_base_geometries(self):
        for layout in LAYOUTS:
            with self.subTest(layout=layout.number):
                original, _ = geometry_tree(layout)
                nominal, _ = geometry_tree(layout, MechanismParameters.nominal(layout))
                self.assertEqual(geometry_signature(nominal), geometry_signature(original))
                saved = named_geoms(ET.parse(layout.path).getroot())
                for name, geom in named_geoms(nominal).items():
                    self.assertIn(name, saved)
                    self.assertEqual(canonical_attributes(geom), canonical_attributes(saved[name]))

    def assert_ports_and_frames(self, root, manifest):
        geoms = named_geoms(root)
        owners = {geom.get("name"): body.get("name") for body in root.iter("body")
                  for geom in body.findall("geom")}
        body_parents = {child.get("name"): body.get("name") for body in root.iter("body")
                        for child in body.findall("body")}
        for name in LINKS:
            frame_names = [f"{name}_frame_{i}" for i in range(12)]
            endpoints = np.array([np.fromstring(geoms[edge].get("fromto"), sep=" ").reshape(2, 3)
                                  for edge in frame_names])
            points = np.unique(np.round(endpoints.reshape(-1, 3), 11), axis=0)
            self.assertEqual(points.shape, (8, 3))
            self.assertEqual(np.linalg.matrix_rank(points-points.mean(axis=0), tol=1e-10), 3)
            for edge in endpoints:
                self.assertEqual(np.count_nonzero(np.abs(edge[1]-edge[0]) > 1e-10), 1)
            for point in points:
                self.assertEqual(np.count_nonzero(np.linalg.norm(endpoints-point, axis=2) < 1e-10), 3)
        for drive in manifest:
            joint = drive["joint"]
            parent, child = LINKS[joint-1:joint+1]
            self.assertEqual(body_parents[f"motor_{joint}"], parent)
            self.assertEqual(body_parents[f"gearbox_{joint}"], parent)
            self.assertEqual(body_parents[f"output_coupling_{joint}"], child)
            axis = np.asarray(drive["shaft_axis"])
            for prefix, center_key, contact_key, mount_key in (
                    (f"J{joint}_fixed_mount", "gearbox_center_m", "gearbox_contact_points_m", "fixed_support_geoms"),
                    (f"M{joint}_case_mount", "motor_center_m", "motor_contact_points_m", "motor_mount_geoms")):
                contacts = np.asarray(drive[contact_key])
                center = np.asarray(drive[center_key])
                self.assertEqual(len(np.unique(np.round(contacts, 11), axis=0)), 4)
                np.testing.assert_allclose((contacts-center)@axis, 0, atol=1e-11)
                stays = [np.fromstring(geoms[f"{prefix}_case_contact_{i}"].get("fromto"), sep=" ").reshape(2, 3)
                         for i in range(4)]
                lengths = np.linalg.norm(np.array(stays)[:, 1]-np.array(stays)[:, 0], axis=1)
                np.testing.assert_allclose(lengths, np.full(4, lengths[0]), atol=1e-10)
                self.assertTrue(all(owners[name] == parent for name in drive[mount_key]))
                for i, contact in enumerate(contacts):
                    self.assertLess(np.linalg.norm(stays[i]-contact, axis=1).min(), 1e-10)
                frame = [f"{parent}_frame_{i}" for i in range(12)]
                local_geoms = {name: geom for name, geom in geoms.items() if owners[name] == parent}
                self.assertTrue(connected_to_frame(local_geoms, drive[mount_key], frame))
            self.assertTrue(all(owners[name] == child for name in drive["moving_support_geoms"]))
            frame = [f"{child}_frame_{i}" for i in range(12)]
            local_geoms = {name: geom for name, geom in geoms.items() if owners[name] == child}
            self.assertTrue(connected_to_frame(local_geoms, drive["moving_support_geoms"], frame))

    def test_base_ports_have_real_edge_paths_to_their_own_link_cages(self):
        for layout in LAYOUTS:
            with self.subTest(layout=layout.number):
                root, manifest = geometry_tree(layout, MechanismParameters.nominal(layout))
                self.assert_ports_and_frames(root, manifest)

    def test_length_tuning_preserves_the_base_named_connection_graph(self):
        layout = LAYOUTS[2]
        nominal = MechanismParameters.nominal(layout)
        reference, _ = geometry_tree(layout, nominal)
        tuned_layout = replace(layout, l2_length=layout.l2_length+.025,
                               l3_length=layout.l3_length+.020)
        tuned = replace(nominal, l1_length=nominal.l1_length+.040)
        actual, manifest = geometry_tree(tuned_layout, tuned)
        self.assertEqual(set(named_geoms(actual)), set(named_geoms(reference)))
        self.assert_ports_and_frames(actual, manifest)

    def test_small_wrist_motor_saddles_fit_the_selected_square_flange_envelope(self):
        # GL40/GL30 net case dimensions with HD14/CD17/CS11-1U-E.
        # CS11's 40 mm flange is square: retain its circumscribed radius,
        # rather than incorrectly interpreting 40 mm as a circular diameter.
        wrist=.11
        for number in (7,8,9):
            with self.subTest(layout=number):
                forearm=.072 if number==7 else .16
                upper=.55-wrist-.08-np.hypot(.09,forearm)
                parameters=MechanismParameters(
                    upper, (.107,)*3+(.041,)*3, (.312,.421,.421,.2,.2,.2),
                    (.02325,)*3+(.01725,)*3,
                    (.035,.031,.031)+(.020*np.sqrt(2),)*3,
                    (.0215,)*3+(.0157,)*3, (.0175,.0265,.0265)+(.0565,)*3)
                layout=replace(LAYOUTS[number-1],l2_length=.09,
                               l3_length=forearm,wrist2_length=wrist)
                root,manifest=geometry_tree(layout,parameters)
                self.assert_ports_and_frames(root,manifest)
                # Every original case connection is still a mid-rim port,
                # and all four stays have equal length (checked above).
                motor=manifest[4]
                contacts=np.asarray(motor['motor_contact_points_m'])
                center=np.asarray(motor['motor_center_m'])
                np.testing.assert_allclose(np.linalg.norm(contacts-center,axis=1),.01725)

    def test_short_roll_stack_and_unsupported_motor_offsets_are_rejected(self):
        layout = LAYOUTS[4]
        nominal = MechanismParameters.nominal(layout)
        with self.assertRaises((GeometryError, ValueError)):
            geometry_tree(replace(layout, l2_length=.005), nominal)
        with self.assertRaises((GeometryError, ValueError)):
            geometry_tree(layout, replace(nominal, motor_offsets=(0., 1., 0., 0.)))

    def test_fixed_lightweight_hardware_seed_finds_the_narrow_coaxial_length_region(self):
        from optimization.design import initial_candidate
        from optimization.inputs import Inputs,Motor,Reducer,Requirement,load_inputs
        from optimization.model_factory import build_template_model
        from optimization.reach import extended_reach_m
        source=Path(__file__).resolve().parents[1]/"optimization/examples/synthetic"
        links=load_inputs(source).links
        gl40=Motor("GL40","GL40 fixture",.25,2100.,None,None,None,.107,
                   23.25,21.5,7.4e-6,capacity_model="rated_continuous")
        gl30=Motor("GL30","GL30 fixture",.08,1990.,None,None,None,.041,
                   17.25,15.7,2.42e-6,capacity_model="rated_continuous")
        gears=(Reducer("HD","HD14 fixture",50.,.312,35.,17.5,4.5,13.,None,8500.,allowed_joints=(1,)),
               Reducer("CD","CD17 fixture",50.,.421,31.,26.5,11.,23.,None,7300.,allowed_joints=(2,3)),
               Reducer("CS","CS11 square envelope fixture",30.,.2,20*np.sqrt(2),56.5,
                       2.2,4.5,None,8500.,allowed_joints=(4,5,6)))
        inputs=Inputs((gl40,gl30),gears,(),links)
        requirement=Requirement("LOCKED_GEOMETRY_FIXTURE",500,500,550,550,
            allowed_layouts=(7,),wrist2_length_m=.11,
            motor_candidates_by_joint=(("GL40",),("GL40",),("GL40",)),
            shared_wrist_motor_id="GL30")
        candidate=initial_candidate(inputs,requirement)
        model,_=build_template_model(candidate,inputs,requirement=requirement)
        self.assertEqual(model.nv,6)
        self.assertEqual(candidate.motor_ids,("GL40",)*3+("GL30",)*3)
        self.assertEqual(candidate.reducer_ids,("HD","CD","CD","CS","CS","CS"))
        self.assertAlmostEqual(extended_reach_m(candidate),.55,places=12)

    def test_invalid_geometry_never_reaches_the_payload_load_solver(self):
        from optimization.design import Candidate
        from optimization.inputs import load_inputs
        from optimization.physics import evaluate_candidate
        from optimization.poses import generate_poses
        source = Path(__file__).resolve().parents[1] / "optimization/examples/synthetic"
        inputs = load_inputs(source)
        requirement = inputs.requirements[0]
        candidate = Candidate(
            1, (.32, .005, .27), tuple(link.mass_lower_kg for link in inputs.links),
            (.5,)*6, (1/12,)*6, (0.,)*4,
            (inputs.motors[0].motor_id,)*6, (inputs.reducers[0].reducer_id,)*6,
        )
        with patch("optimization.physics.mujoco.mj_inverse", side_effect=AssertionError("invalid candidate was simulated")):
            result = evaluate_candidate(candidate, inputs, generate_poses(requirement, limit=1), requirement)
        self.assertFalse(result.feasible)
        self.assertEqual(result.pmax_kg, 0.)
        self.assertFalse(result.summary["geometry_valid"])
        self.assertEqual(result.summary["failure_reason"], "unsupported_base_geometry")
        self.assertTrue(result.summary["geometry_errors"])


if __name__ == "__main__":
    unittest.main()

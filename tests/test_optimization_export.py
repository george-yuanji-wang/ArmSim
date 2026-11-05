"""A full export must retain the optimized mechanics and work as an ArmSim model."""

from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from armsim.catalog import AXES
from armsim.brackets import EDGE_RADIUS
from armsim.simulation import Simulation
from optimization.design import initial_candidate
from optimization.export import export_model
from optimization.inputs import load_inputs
from optimization.physics import build_model
from optimization.structural_mass import candidate_from_model, structural_properties
from armsim.mechanism_parameters import GeometryError


DEMO = Path(__file__).resolve().parents[1] / "optimization/examples/synthetic"


def inertia_tensor(model, body):
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.body_iquat[body])
    rotation = rotation.reshape(3, 3)
    return rotation @ np.diag(model.body_inertia[body]) @ rotation.T


def metadata(xml):
    root = ET.fromstring(xml)
    return root, json.loads(root.find("./custom/text[@name='armsim_metadata']").get("data"))


class OptimizationExportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.inputs = load_inputs(DEMO)
        cls.requirement = cls.inputs.requirements[0]
        cls.candidate = initial_candidate(cls.inputs, cls.requirement)

    def candidate_for(self, number):
        requirement = replace(self.requirement, allowed_layouts=(number,))
        return initial_candidate(self.inputs, requirement)

    def test_independent_mass_com_and_spread_fields_cannot_change_physics(self):
        candidate = self.candidate_for(8)
        reference, reference_xml = build_model(candidate, self.inputs)
        poisoned = replace(candidate, link_masses_kg=(999.,)*6,
                           com_fractions=(-5.,)*6, spread_fractions=(999.,)*6)
        actual, actual_xml = build_model(poisoned, self.inputs)
        self.assertEqual(actual_xml, reference_xml)
        for array in ("body_mass", "body_ipos", "body_inertia", "body_iquat"):
            np.testing.assert_array_equal(getattr(actual, array), getattr(reference, array))
        states = ([.2, .7, -.8, .4, .6, -.3], [.3, -.2, .1, -.4, .2, .5],
                  [.4, -.3, .6, -.2, .1, -.5])
        loads = []
        for model in (reference, actual):
            data = mujoco.MjData(model)
            data.qpos[:], data.qvel[:], data.qacc[:] = states
            mujoco.mj_inverse(model, data)
            loads.append(data.qfrc_inverse.copy())
        np.testing.assert_array_equal(loads[0], loads[1])
        _, info = metadata(export_model(poisoned, self.inputs, self.requirement))
        expected = json.loads(json.dumps(candidate_from_model(candidate, reference).to_dict()))
        self.assertEqual(info["design"], expected)

    def test_length_increases_actual_rail_stock_mass_and_exports_full_com(self):
        candidate = self.candidate_for(7)
        original, _ = build_model(candidate, self.inputs)
        extension = .05
        longer = replace(candidate, lengths_m=(candidate.lengths_m[0]+extension,
                                               *candidate.lengths_m[1:]))
        extended, _ = build_model(longer, self.inputs)
        before = {row["body_name"]: row for row in structural_properties(original)}
        after = {row["body_name"]: row for row in structural_properties(extended)}
        # Only the four longitudinal L1 cage rails gain stock length; its
        # unchanged end-loop, mounts and output interfaces translate with it.
        expected_gain = 4*2700*np.pi*EDGE_RADIUS**2*extension
        self.assertAlmostEqual(after["upper_arm_link"]["mass_kg"]-
                               before["upper_arm_link"]["mass_kg"], expected_gain, places=12)
        self.assertGreater(after["upper_arm_link"]["com_local_xyz_m"][2],
                           before["upper_arm_link"]["com_local_xyz_m"][2])
        self.assertAlmostEqual(after["forearm_link"]["mass_kg"],
                               before["forearm_link"]["mass_kg"], places=12)
        xml = export_model(longer, self.inputs, self.requirement)
        _, info = metadata(xml)
        compiled = mujoco.MjModel.from_xml_string(xml)
        self.assertEqual(info["structural_bodies"], structural_properties(compiled))
        forearm = next(row for row in info["structural_bodies"]
                       if row["body_name"] == "forearm_link")
        self.assertGreater(abs(forearm["com_local_xyz_m"][0]), .0001)
        self.assertGreater(abs(forearm["com_local_xyz_m"][1]), .0001)
        inertia = np.array(forearm["inertia_com_local_kg_m2"])
        self.assertGreater(np.max(np.abs(inertia-np.diag(np.diag(inertia)))), 1e-7)

    def test_structural_and_catalog_mass_partition_includes_the_base(self):
        for number in (7, 8, 9):
            with self.subTest(layout=number):
                candidate = self.candidate_for(number)
                model, _ = build_model(candidate, self.inputs)
                rows = structural_properties(model)
                self.assertEqual(len(rows), 7)
                base = next(row for row in rows if row["body_name"] == "base_link")
                self.assertGreater(base["mass_kg"], 0)
                hardware = sum(self.inputs.motor_by_id[name].mass_kg for name in candidate.motor_ids)
                hardware += sum(self.inputs.reducer_by_id[name].mass_kg for name in candidate.reducer_ids)
                structural = sum(row["mass_kg"] for row in rows)
                self.assertAlmostEqual(structural+hardware, model.body_mass.sum(), places=12)
                # The six retained candidate fields exclude the fixed base
                # fixture, whereas the complete assembly mass includes it.
                resolved = candidate_from_model(candidate, model)
                self.assertAlmostEqual(sum(resolved.link_masses_kg)+base["mass_kg"],
                                       structural, places=12)

    def test_child_cage_cannot_pass_through_its_own_reducer(self):
        candidate = self.candidate_for(7)
        reducers = list(candidate.reducer_ids)
        reducers[4] = "SYNTHETIC_R_50"
        # A 35 mm J5 case intersects the fixed 70 mm wrist's own child stock.
        # Connections alone are insufficient: this candidate must not score/export.
        candidate = replace(candidate, reducer_ids=tuple(reducers))
        with self.assertRaisesRegex(GeometryError, "J5"):
            build_model(candidate, self.inputs)
        with self.assertRaisesRegex(GeometryError, "J5"):
            export_model(candidate, self.inputs, self.requirement)

    def test_export_preserves_mass_distribution_fk_gravity_and_inertial_loads(self):
        rng = np.random.default_rng(126)
        for number in (7, 8, 9):
            with self.subTest(layout=number):
                candidate = self.candidate_for(number)
                reference, _ = build_model(candidate, self.inputs)
                exported = mujoco.MjModel.from_xml_string(
                    export_model(candidate, self.inputs, self.requirement))
                self.assertAlmostEqual(float(reference.body_mass.sum()),
                                       float(exported.body_mass.sum()), places=11)
                for old in range(1, reference.nbody):
                    if reference.body_mass[old] == 0:
                        continue
                    name = reference.body(old).name
                    new = exported.body(name).id
                    self.assertEqual(exported.body(int(exported.body_parentid[new])).name,
                                     reference.body(int(reference.body_parentid[old])).name)
                    np.testing.assert_allclose(exported.body_pos[new], reference.body_pos[old], atol=1e-12)
                    rotation0, rotation1 = np.empty(9), np.empty(9)
                    mujoco.mju_quat2Mat(rotation0, reference.body_quat[old])
                    mujoco.mju_quat2Mat(rotation1, exported.body_quat[new])
                    np.testing.assert_allclose(rotation1, rotation0, atol=1e-12)
                    np.testing.assert_allclose(exported.body_mass[new], reference.body_mass[old], atol=1e-12)
                    np.testing.assert_allclose(exported.body_ipos[new], reference.body_ipos[old], atol=1e-12)
                    np.testing.assert_allclose(inertia_tensor(exported, new), inertia_tensor(reference, old),
                                               rtol=1e-10, atol=1e-12)
                data0, data1 = mujoco.MjData(reference), mujoco.MjData(exported)
                matrix0, matrix1 = np.empty((6, 6)), np.empty((6, 6))
                armature = np.array([
                    self.inputs.reducer_by_id[rid].reduction_ratio**2
                    * self.inputs.motor_by_id[mid].rotor_inertia_kg_m2
                    for mid, rid in zip(candidate.motor_ids, candidate.reducer_ids)
                ])
                for _ in range(64):
                    q = rng.uniform(-np.pi, np.pi, 6)
                    for data in (data0, data1):
                        data.qpos[:] = q
                        data.qvel[:] = 0
                        data.qacc[:] = 0
                    mujoco.mj_inverse(reference, data0)
                    mujoco.mj_inverse(exported, data1)
                    np.testing.assert_allclose(data1.site("tool_frame").xpos,
                                               data0.site("tool_frame").xpos, atol=2e-12)
                    np.testing.assert_allclose(data1.site("tool_frame").xmat,
                                               data0.site("tool_frame").xmat, atol=2e-12)
                    np.testing.assert_allclose(data1.qfrc_inverse, data0.qfrc_inverse, atol=2e-10)
                    mujoco.mj_fullM(reference, data0, matrix0)
                    mujoco.mj_fullM(exported, data1, matrix1)
                    np.testing.assert_allclose(matrix1-matrix0, np.diag(armature), atol=2e-11)
                # Random mixed motion tests rotor inertia once, not merely static equivalence.
                velocity, acceleration = rng.normal(size=(2, 6))
                for data in (data0, data1):
                    data.qvel[:] = velocity
                    data.qacc[:] = acceleration
                mujoco.mj_inverse(reference, data0)
                mujoco.mj_inverse(exported, data1)
                np.testing.assert_allclose(data1.qfrc_inverse-data0.qfrc_inverse,
                                           armature*acceleration, atol=2e-10)

    def test_complete_visual_structure_has_balanced_case_ports_and_correct_ownership(self):
        for number in (7, 8, 9):
            with self.subTest(layout=number):
                candidate = self.candidate_for(number)
                xml = export_model(candidate, self.inputs, self.requirement)
                tree, info = metadata(xml)
                model = mujoco.MjModel.from_xml_string(xml)
                geoms = {geom.get("name"): geom for geom in tree.iter("geom") if geom.get("name")}
                structure = [geom for geom in geoms.values() if geom.get("group") in {"0", "4"}]
                self.assertGreater(len(structure), 100)
                for geom in structure:
                    self.assertEqual(geom.get("type"), "cylinder")
                    endpoints = np.fromstring(geom.get("fromto", ""), sep=" ")
                    self.assertEqual(len(endpoints), 6, geom.get("name"))
                    self.assertGreater(np.linalg.norm(endpoints[:3]-endpoints[3:]), 1e-8)
                    radius = float(geom.get("size"))
                    stock_mass = 2700*np.pi*radius**2*np.linalg.norm(endpoints[:3]-endpoints[3:])
                    self.assertAlmostEqual(float(geom.get("mass")), stock_mass, places=12)
                for drive in info["drives"]:
                    joint = drive["joint"]
                    motor = self.inputs.motor_by_id[candidate.motor_ids[joint-1]]
                    reducer = self.inputs.reducer_by_id[candidate.reducer_ids[joint-1]]
                    np.testing.assert_allclose(model.geom(f"M{joint}_stator").size[:2],
                                               [motor.radius_m, motor.length_m/2], atol=1e-12)
                    self.assertAlmostEqual(model.geom(f"G{joint}_housing").size[0], reducer.radius_m)
                    reducer_body = model.body(f"reducer_{joint}").id
                    case_geoms = np.flatnonzero(model.geom_bodyid == reducer_body)
                    lower = np.min(model.geom_pos[case_geoms, 2]-model.geom_size[case_geoms, 1])
                    upper = np.max(model.geom_pos[case_geoms, 2]+model.geom_size[case_geoms, 1])
                    self.assertAlmostEqual(upper-lower, reducer.length_m)
                    parent = model.body(f"motor_{joint}").parentid[0]
                    child = int(model.jnt_bodyid[joint-1])
                    self.assertEqual(model.body(f"reducer_{joint}").parentid[0], parent)
                    self.assertEqual(model.body(f"output_coupling_{joint}").parentid[0], child)
                    for names in (drive["fixed_support_geoms"], drive["motor_mount_geoms"],
                                  drive["moving_support_geoms"]):
                        points = np.vstack([np.fromstring(geoms[name].get("fromto"), sep=" ").reshape(2, 3)
                                            for name in names])
                        self.assertEqual(np.linalg.matrix_rank(points-points.mean(axis=0), tol=1e-10), 3)
                    axis = np.array(drive["shaft_axis"])
                    for contacts_key, center_key, mount_names in (
                            ("gearbox_contact_points_m", "gearbox_center_m", drive["fixed_support_geoms"]),
                            ("motor_contact_points_m", "motor_center_m", drive["motor_mount_geoms"])):
                        contacts = np.asarray(drive[contacts_key])
                        self.assertEqual(contacts.shape, (4, 3))
                        self.assertEqual(len(np.unique(np.round(contacts, 11), axis=0)), 4)
                        delta = contacts-np.asarray(drive[center_key])
                        np.testing.assert_allclose(delta @ axis, 0, atol=1e-11)
                        lengths = np.linalg.norm(delta, axis=1)
                        np.testing.assert_allclose(lengths, np.full(4, lengths[0]), atol=1e-11)
                        segments = [np.fromstring(geoms[name].get("fromto"), sep=" ").reshape(2, 3)
                                    for name in mount_names]
                        stays = []
                        for contact in contacts:
                            attached = [segment for segment in segments
                                        if np.min(np.linalg.norm(segment-contact, axis=1)) < 1e-10]
                            self.assertEqual(len(attached), 1)
                            stays.append(np.linalg.norm(attached[0][1]-attached[0][0]))
                        np.testing.assert_allclose(stays, np.full(4, stays[0]), atol=1e-10)
                    for geom_name, expected in (
                            (f"M{joint}_stator", (.16, .76, .62, 1)),
                            (f"G{joint}_housing", (1, .65, .22, 1)),
                            (f"O{joint}_moving_flange", (.62, .67, .74, 1))):
                        gid = model.geom(geom_name).id
                        material = int(model.geom_matid[gid])
                        color = model.mat_rgba[material] if material >= 0 else model.geom_rgba[gid]
                        np.testing.assert_allclose(color, expected, atol=1e-6)
                self.assertIsNotNone(tree.find("./worldbody/light"))
                self.assertIsNotNone(tree.find("./asset/texture"))
                self.assertFalse(any(geom.get("type") == "capsule" for geom in tree.iter("geom")))

    def test_catalog_actuators_markers_sensors_and_metadata_are_self_contained(self):
        candidate = self.candidate_for(8)
        xml = export_model(candidate, self.inputs, self.requirement)
        tree, info = metadata(xml)
        model = mujoco.MjModel.from_xml_string(xml)
        self.assertEqual(model.nu, 6)
        self.assertEqual(tree.find("./custom/text[@name='armsim_export_version']").get("data"), "1")
        self.assertEqual(tree.find("./custom/text[@name='armsim_model_kind']").get("data"), "optimized")
        self.assertFalse(np.any(model.jnt_limited))
        np.testing.assert_allclose(model.jnt_axis, AXES)
        for i, (mid, rid) in enumerate(zip(candidate.motor_ids, candidate.reducer_ids)):
            motor, reducer = self.inputs.motor_by_id[mid], self.inputs.reducer_by_id[rid]
            ratio = reducer.reduction_ratio
            self.assertEqual(model.actuator_trnid[i, 0], i)
            self.assertAlmostEqual(model.actuator_gear[i, 0], ratio)
            expected_force = min(motor.rated_torque_Nm, reducer.continuous_output_torque_Nm/ratio,
                                 reducer.max_input_torque_Nm)
            np.testing.assert_allclose(model.actuator_forcerange[i], [-expected_force, expected_force])
            self.assertAlmostEqual(model.dof_armature[i], ratio**2*motor.rotor_inertia_kg_m2)
            self.assertEqual(info["drives"][i]["motor_id"], mid)
            self.assertEqual(info["drives"][i]["reducer_id"], rid)
            self.assertEqual(info["drives"][i]["ratio"], ratio)
            for prefix in ("J", "M", "G", "O"):
                self.assertGreaterEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{prefix}{i+1}"), 0)
        for site in ("world_frame", "tool_frame"):
            self.assertGreaterEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site), 0)
        self.assertGreaterEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "tool_position"), 0)
        self.assertGreaterEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "tool_orientation"), 0)
        inspect = model.key("inspect")
        np.testing.assert_allclose(inspect.ctrl, inspect.qpos*model.actuator_gear[:, 0])

    def test_certified_export_runs_rated_servo_dynamics_but_raw_model_stays_passive(self):
        for number in (7, 8, 9):
            with self.subTest(layout=number), tempfile.TemporaryDirectory() as directory:
                candidate = self.candidate_for(number)
                path = Path(directory) / "best_model.xml"
                path.write_text(export_model(candidate, self.inputs, self.requirement), encoding="utf-8")
                sim = Simulation.from_path(path)
                self.assertTrue(sim.supports_dynamics)
                self.assertTrue(sim.ratios_known)
                ratios = np.array([self.inputs.reducer_by_id[rid].reduction_ratio for rid in candidate.reducer_ids])
                np.testing.assert_allclose(sim.ratios, ratios)
                sim.dynamic = True
                sim.targets[:] += [.02, -.02, .03, .05, -.02, .01]
                for _ in range(100):
                    sim.advance(.01)
                    self.assertTrue(np.isfinite(sim.data.qpos).all())
                    self.assertTrue(np.isfinite(sim.data.qvel).all())
                    self.assertFalse(any(warning.number for warning in sim.data.warning))
                    np.testing.assert_array_less(
                        np.abs(sim.data.actuator_force), sim.model.actuator_forcerange[:, 1]+1e-8)
                    np.testing.assert_allclose(sim.output_torques, sim.data.actuator_force*ratios)
                self.assertGreater(sim.data.time, 0)
                _, raw = build_model(candidate, self.inputs)
                path = Path(directory) / "raw.xml"
                path.write_text(raw, encoding="utf-8")
                passive = Simulation.from_path(path)
                self.assertFalse(passive.supports_dynamics)


if __name__ == "__main__":
    unittest.main()

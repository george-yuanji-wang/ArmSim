"""The exported/runtime envelope must match the selected reducer load policy."""

from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from armsim.model_catalog import export_metadata
from armsim.simulation import Simulation
from optimization.design import Candidate
from optimization.export import export_model
from optimization.inputs import Inputs, LinkBounds, Motor, Reducer, Requirement, STRUCTURAL_BODIES


def fixture():
    """Small numerical hardware, with a deliberately reducer-limited motor."""
    mass, radius = .2, .020
    motor = Motor(
        "fixture_motor", "Compact export-policy test motor", 1., 6000.,
        None, None, None, mass, radius*1000, 18., None,
        capacity_model="rated_continuous",
        rotor_inertia_upper_bound_kg_m2=mass*radius**2,
        rotor_inertia_basis="mass_radius_upper_bound")
    reducer = Reducer("fixture_gear", "Export-policy test reducer", 30., .1,
                      25., 20., 3., 8.7, None, 6000., allowed_joints=(1, 2, 3, 4, 5, 6))
    links = tuple(LinkBounds(name, .01, 1., .1, .9, .01, .2, 10.)
                  for name in STRUCTURAL_BODIES)
    inputs = Inputs((motor,), (reducer,), (), links)
    candidate = Candidate(
        8, (.55-.15-np.hypot(.09, .16), .09, .16),
        (.1,)*6, (.5,)*6, (.1,)*6, (0.,)*4,
        (motor.motor_id,)*6, (reducer.reducer_id,)*6)
    requirement = Requirement("EXPORT_POLICY_FIXTURE", 500., 500., 550., 550.,
                              allowed_layouts=(8,))
    return candidate, inputs, requirement


def change_metadata(xml, change):
    root = ET.fromstring(xml)
    node = root.find("./custom/text[@name='armsim_metadata']")
    info = json.loads(node.get("data"))
    change(info)
    node.set("data", json.dumps(info))
    return ET.tostring(root, encoding="unicode")


class PeakExportConsistencyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.candidate, cls.inputs, cls.requirement = fixture()
        cls.peak_requirement = replace(cls.requirement, reducer_load_policy="hold_and_repeated_peak")
        cls.legacy_xml = export_model(cls.candidate, cls.inputs, cls.requirement)
        cls.peak_xml = export_model(cls.candidate, cls.inputs, cls.peak_requirement)

    def test_new_policy_exports_repeated_peak_and_legacy_keeps_continuous_cap(self):
        for xml, output_limit, policy in (
                (self.legacy_xml, 3., "continuous"),
                (self.peak_xml, 8.7, "hold_and_repeated_peak")):
            with self.subTest(policy=policy):
                model = mujoco.MjModel.from_xml_string(xml)
                info = export_metadata(model)
                self.assertEqual(info["geometry_contract"], "approved-base-template-v8")
                self.assertEqual(info["reducer_load_policy"], policy)
                self.assertIn("Not simulated", info["thermal_duty_model"])
                self.assertIn("unverified transient", info["motor_torque_basis"])
                np.testing.assert_allclose(model.actuator_forcerange[:, 1]*30., output_limit)
                for drive in info["drives"]:
                    self.assertEqual(drive["reducer_load_policy"], policy)
                    self.assertEqual(drive["continuous_output_torque_Nm"], 3.)
                    self.assertEqual(drive["peak_output_torque_Nm"], 8.7)
                    self.assertEqual(drive["instantaneous_reducer_output_torque_limit_Nm"], output_limit)
                    self.assertEqual(drive["instantaneous_joint_torque_limit_Nm"], output_limit)

    def test_repeated_peak_never_overrides_motor_continuous_or_known_input_limit(self):
        cases = (
            (replace(self.inputs, motors=(replace(self.inputs.motors[0], rated_torque_Nm=.2),)), 6., 8.7),
            (replace(self.inputs, reducers=(replace(self.inputs.reducers[0], max_input_torque_Nm=.15),)), 4.5, 4.5),
        )
        for inputs, joint_limit, reducer_limit in cases:
            with self.subTest(joint_limit=joint_limit):
                xml = export_model(self.candidate, inputs, self.peak_requirement)
                model = mujoco.MjModel.from_xml_string(xml)
                info = export_metadata(model)
                np.testing.assert_allclose(model.actuator_forcerange[:, 1]*30., joint_limit)
                for drive in info["drives"]:
                    self.assertEqual(drive["instantaneous_joint_torque_limit_Nm"], joint_limit)
                    self.assertEqual(drive["instantaneous_reducer_output_torque_limit_Nm"], reducer_limit)

    def test_previous_exports_without_new_policy_fields_still_load(self):
        def legacy(info):
            info.pop("reducer_load_policy")
            for drive in info["drives"]:
                for key in ("reducer_load_policy", "peak_output_torque_Nm",
                            "instantaneous_reducer_output_torque_limit_Nm",
                            "instantaneous_joint_torque_limit_Nm"):
                    drive.pop(key)
        xml = change_metadata(self.legacy_xml, legacy)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"legacy.xml"
            path.write_text(xml, encoding="utf-8")
            sim = Simulation.from_path(path)
            self.assertTrue(sim.supports_dynamics)
            np.testing.assert_allclose(sim._base_force_limits[:, 1]*30., 3.)

    def test_inconsistent_policy_or_instantaneous_metadata_is_rejected(self):
        def wrong_limit(info):
            info["drives"][0]["instantaneous_joint_torque_limit_Nm"] = 30.
        def missing_limit(info):
            info["drives"][0].pop("instantaneous_reducer_output_torque_limit_Nm")
        def mismatched_policy(info):
            info["reducer_load_policy"] = "continuous"
        for change in (wrong_limit, missing_limit, mismatched_policy):
            with self.subTest(change=change.__name__):
                model = mujoco.MjModel.from_xml_string(change_metadata(self.peak_xml, change))
                with self.assertRaisesRegex(ValueError, "Invalid ArmSim export metadata"):
                    export_metadata(model)

    def test_runtime_uses_repeated_peak_cap_and_remains_finite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"peak.xml"
            path.write_text(self.peak_xml, encoding="utf-8")
            sim = Simulation.from_path(path)
            self.assertTrue(sim.supports_dynamics)
            sim.data.ctrl[:] = (sim.data.qpos+1.)*sim.ratios
            mujoco.mj_forward(sim.model, sim.data)
            np.testing.assert_allclose(sim.data.actuator_force*sim.ratios, 8.7)
            # At supported speeds playback retains the exported peak cap; it
            # does not silently restore the old 3 Nm continuous reducer cap.
            sim._apply_force_limits()
            np.testing.assert_allclose(sim.model.actuator_forcerange[:, 1]*30., 8.7)
            sim.dynamic = True
            for _ in range(5):
                sim.advance(.02)
            self.assertTrue(np.isfinite(sim.data.qpos).all())
            self.assertTrue(np.isfinite(sim.data.qvel).all())
            self.assertTrue(np.isfinite(sim.output_torques).all())
            self.assertTrue(np.all(np.abs(sim.output_torques) <= 8.7+1e-8))
            # The policy adds no unsupported motor-speed capability.
            sim.data.qvel[:] = 6001.*2*np.pi/(60*sim.ratios)
            sim._apply_force_limits()
            np.testing.assert_array_equal(sim.model.actuator_forcerange, np.zeros((6, 2)))

    def test_runtime_rejects_actuators_exceeding_the_selected_peak_cap(self):
        root = ET.fromstring(self.peak_xml)
        root.find("./actuator/position").set("forcerange", "-1 1")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"invalid.xml"
            path.write_text(ET.tostring(root, encoding="unicode"), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "instantaneous drivetrain limits"):
                Simulation.from_path(path)


if __name__ == "__main__":
    unittest.main()

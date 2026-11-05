"""Transient optimization envelopes must survive export and playback."""

from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import mujoco
import numpy as np

from armsim.drivetrain_limits import joint_torque_limit, motor_torque_at_speed
from armsim.model_catalog import export_metadata
from armsim.simulation import Simulation
from optimization.export import export_model
from tests.test_peak_export_consistency import fixture, change_metadata


class MotorPeakExportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.candidate, inputs, requirement = fixture()
        motor = replace(inputs.motors[0], rated_torque_Nm=.08,
                        rated_speed_rpm=2000., peak_torque_Nm=.28,
                        peak_torque_speed_curve=((0., .28), (1000., .18),
                                                 (1500., .12), (2000., .05)),
                        operating_voltage_V=16., peak_curve_basis="synthetic test envelope")
        reducer = replace(inputs.reducers[0], continuous_output_torque_Nm=2.2,
                          peak_output_torque_Nm=4.5)
        cls.inputs = replace(inputs, motors=(motor,), reducers=(reducer,))
        cls.requirement = replace(requirement, motor_load_policy="hold_and_peak_envelope",
                                  reducer_load_policy="hold_and_repeated_peak")
        cls.xml = export_model(cls.candidate, cls.inputs, cls.requirement)

    def test_export_preserves_original_rated_torque_and_explicit_transient_curve(self):
        model = mujoco.MjModel.from_xml_string(self.xml)
        info = export_metadata(model)
        self.assertEqual(info["motor_load_policy"], "hold_and_peak_envelope")
        self.assertIn("zero-speed upper bound", info["motor_torque_basis"])
        self.assertIn("Not simulated", info["thermal_duty_model"])
        np.testing.assert_allclose(model.actuator_forcerange[:, 1]*30., 4.5)
        for drive in info["drives"]:
            self.assertEqual(drive["motor_rated_torque_Nm"], .08)
            self.assertEqual(drive["motor_peak_torque_Nm"], .28)
            self.assertEqual(drive["motor_operating_voltage_V"], 16.)
            self.assertIsNone(drive["motor_peak_duration_s"])
            self.assertAlmostEqual(joint_torque_limit(drive), 4.5)
            self.assertAlmostEqual(motor_torque_at_speed(drive, 1250.), .15)
            self.assertAlmostEqual(motor_torque_at_speed(drive, -1500.), .12)
            self.assertAlmostEqual(motor_torque_at_speed(drive, 2000.), .05)
            self.assertEqual(motor_torque_at_speed(drive, 2001.), 0.)

    def test_playback_enforces_speed_curve_instead_of_rated_constant_or_peak_constant(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"peak.xml"
            path.write_text(self.xml, encoding="utf-8")
            sim = Simulation.from_path(path)
            for rpm, expected in ((0., .15), (1500., .12), (2000., .05), (2001., 0.)):
                sim.data.qvel[:] = rpm*2*np.pi/(60*sim.ratios)
                sim._apply_force_limits()
                np.testing.assert_allclose(sim.model.actuator_forcerange[:, 1], expected)
                np.testing.assert_allclose(sim.model.actuator_forcerange[:, 0], -expected)

    def test_import_rejects_missing_curve_or_mismatched_motor_policy(self):
        def missing_curve(info):
            info["drives"][0].pop("motor_peak_torque_speed_curve")
        def wrong_policy(info):
            info["drives"][0]["motor_load_policy"] = "continuous"
        for change in (missing_curve, wrong_policy):
            with self.subTest(change=change.__name__):
                model = mujoco.MjModel.from_xml_string(change_metadata(self.xml, change))
                with self.assertRaisesRegex(ValueError, "Invalid ArmSim export metadata"):
                    export_metadata(model)

    def test_continuous_policy_retains_existing_rated_limit(self):
        requirement = replace(self.requirement, motor_load_policy="continuous")
        model = mujoco.MjModel.from_xml_string(export_model(self.candidate, self.inputs, requirement))
        info = export_metadata(model)
        np.testing.assert_allclose(model.actuator_forcerange[:, 1], .08)
        self.assertAlmostEqual(motor_torque_at_speed(info["drives"][0], 1500.), .08)


if __name__ == "__main__":
    unittest.main()

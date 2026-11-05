"""Holding and speed-dependent transient limits are different constraints."""

from dataclasses import replace
import unittest

import numpy as np

from optimization.physics import motor_peak_torque_limits
from tests.test_reducer_peak_screen import fixture, evaluate_prescribed


def peak_fixture():
    candidate, inputs, requirement = fixture(motor_torque=.1)
    motor = replace(inputs.motors[0], peak_torque_Nm=.3,
                    peak_duration_s=None, rated_voltage_V=12., operating_voltage_V=16.,
                    peak_torque_speed_curve=((0., .3), (1000., .3), (2000., 0.)),
                    peak_curve_basis="voltage_shift_estimate_from_official_12V_curve")
    return candidate, replace(inputs, motors=(motor,)), replace(
        requirement, motor_load_policy="hold_and_peak_envelope")


class MotorPeakScreenTests(unittest.TestCase):
    def test_curve_interpolation_uses_absolute_speed_and_no_extrapolation(self):
        _, inputs, _ = peak_fixture()
        motor = replace(inputs.motors[0], rated_speed_rpm=1800.,
                        peak_torque_speed_curve=((0., .3), (1000., .2), (2000., 0.)))
        rpm = np.array([[0.], [500.], [-1500.], [1800.], [1801.], [2001.]])
        np.testing.assert_allclose(motor_peak_torque_limits((motor,), rpm)[:, 0],
                                   [.3, .25, .1, .04, 0., 0.])

    def test_peak_policy_can_pass_when_continuous_dynamic_screen_fails(self):
        candidate, inputs, requirement = peak_fixture()
        result = evaluate_prescribed(candidate, inputs, requirement)
        self.assertTrue(result.feasible)
        self.assertAlmostEqual(result.pmax_kg, 1.)
        self.assertEqual(result.summary["pmax_continuous_screen_kg"], 0.)
        self.assertFalse(result.summary["continuous_screen_passes_required_payload"])
        self.assertEqual(result.summary["hardware_validation_status"], "conditional_transient_envelope")
        self.assertEqual(result.summary["motor_load_policy"], "hold_and_peak_envelope")
        self.assertTrue(result.summary["pmax_is_conditional_on_transient_envelope"])
        self.assertGreater(result.per_pose_rows[0]["peak_motor_continuous_utilization_j2_ratio"], 1.)
        self.assertLess(result.per_pose_rows[0]["peak_motor_utilization_j2_ratio"], 1.)

    def test_unknown_duration_and_shifted_voltage_are_recorded_without_invented_duty(self):
        candidate, inputs, requirement = peak_fixture()
        result = evaluate_prescribed(candidate, inputs, requirement)
        self.assertGreater(result.pmax_kg, 0.)
        for key in ("motor_transient_rating_verified", "duty_cycle_verified", "thermal_duty_verified",
                    "motor_peak_durations_published"):
            self.assertFalse(result.summary[key])
        self.assertEqual(result.summary["motor_peak_duration_s"], [None]*6)
        evidence = result.summary["motor_curve_evidence"][0]
        self.assertEqual(evidence["operating_voltage_V"], 16.)
        self.assertEqual(evidence["published_rated_voltage_V"], 12.)
        self.assertFalse(evidence["operating_voltage_matches_published_rating"])
        self.assertIn("voltage_shift_estimate", evidence["curve_basis"])

    def test_continuous_diagnostic_matches_an_independent_continuous_policy_evaluation(self):
        candidate, inputs, requirement = peak_fixture()
        inputs = replace(inputs,
            motors=(replace(inputs.motors[0], rated_torque_Nm=.2),),
            reducers=(replace(inputs.reducers[0], continuous_output_torque_Nm=5.,
                              peak_output_torque_Nm=9.),))
        peak = evaluate_prescribed(candidate, inputs, requirement)
        continuous = evaluate_prescribed(candidate, inputs,
                                         replace(requirement, motor_load_policy="continuous"))
        self.assertAlmostEqual(peak.pmax_kg, 4.)
        self.assertAlmostEqual(continuous.pmax_kg, 2.-30.*1e-8*30.*6./1., places=6)
        self.assertAlmostEqual(peak.summary["pmax_continuous_screen_kg"], continuous.pmax_kg,
                               places=12)

    def test_holding_cannot_use_motor_peak_torque(self):
        candidate, inputs, requirement = peak_fixture()
        inputs = replace(inputs, reducers=(replace(inputs.reducers[0],
            continuous_output_torque_Nm=5., peak_output_torque_Nm=15.),))
        result = evaluate_prescribed(candidate, inputs, requirement, gravity=4., inertia=0.)
        self.assertFalse(result.feasible)
        self.assertEqual(result.pmax_kg, 0.)
        self.assertEqual(result.per_pose_rows[0]["limitation"], "static_holding_torque")
        self.assertEqual(result.per_pose_rows[0]["limiting_accel_corner"], -1)
        self.assertTrue(all(result.per_pose_rows[0][f"limiting_acceleration_j{j}_rad_s2"] == 0
                            for j in range(1, 7)))

    def test_dynamic_load_uses_curve_at_tested_rpm_instead_of_scalar_peak(self):
        candidate, inputs, requirement = peak_fixture()
        rpm = .2*30*60/(2*np.pi)
        motor = replace(inputs.motors[0], peak_torque_speed_curve=((0., .3), (rpm, .12), (100., .1)))
        result = evaluate_prescribed(candidate, replace(inputs, motors=(motor,)), requirement)
        self.assertEqual(result.pmax_kg, 0.)
        self.assertFalse(result.feasible)
        self.assertEqual(result.per_pose_rows[0]["limitation"], "motor_peak_speed_envelope")
        self.assertGreater(result.summary["pmax_known_constraints_kg"], 0.)

    def test_curve_endpoint_cannot_be_extrapolated_even_with_higher_rated_speed(self):
        candidate, inputs, requirement = peak_fixture()
        motor = replace(inputs.motors[0], peak_torque_speed_curve=((0., .3), (50., .2)))
        result = evaluate_prescribed(candidate, replace(inputs, motors=(motor,)), requirement)
        self.assertEqual(result.pmax_kg, 0.)
        self.assertEqual(result.per_pose_rows[0]["limitation"], "motor_peak_curve_speed")
        self.assertGreater(result.summary["pmax_known_constraints_kg"], 0.)

    def test_signed_rotor_acceleration_adds_to_negative_joint_load(self):
        candidate, inputs, requirement = peak_fixture()
        motor = replace(inputs.motors[0], rotor_inertia_kg_m2=.0001)
        result = evaluate_prescribed(candidate, replace(inputs, motors=(motor,)), requirement,
                                     gravity=-2., inertia=.5)
        # At 500g, G=-1.5 and qddot2=-6: tau_motor=(-1.5-3)/30-.0001*30*6.
        self.assertAlmostEqual(result.per_pose_rows[0]["peak_motor_torque_j2_Nm_at_required_payload"],
                               .168, places=12)

    def test_default_continuous_policy_ignores_optional_peak_curve(self):
        candidate, inputs, requirement = peak_fixture()
        result = evaluate_prescribed(candidate, inputs, replace(requirement, motor_load_policy="continuous"))
        self.assertEqual(result.pmax_kg, 0.)
        self.assertEqual(result.summary["pmax_continuous_screen_kg"], result.pmax_kg)
        self.assertEqual(result.summary["hardware_validation_status"], "conservative_basis_screen_only")
        self.assertFalse(result.summary["pmax_is_conditional_on_transient_envelope"])

    def test_peak_policy_refuses_a_scalar_peak_without_speed_evidence(self):
        _, inputs, _ = peak_fixture()
        motor = replace(inputs.motors[0], peak_torque_speed_curve=())
        with self.assertRaisesRegex(ValueError, "speed-aware curve"):
            motor_peak_torque_limits((motor,), [[1000.]])


if __name__ == "__main__":
    unittest.main()

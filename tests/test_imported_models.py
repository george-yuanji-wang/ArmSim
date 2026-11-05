"""Optimizer MJCF imports retain their physics and work without servo metadata."""

from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import mujoco
import numpy as np

from armsim.catalog import LAYOUTS
from armsim.model_catalog import discover_imported_models
from armsim.simulation import Simulation
from optimization.design import initial_candidate
from optimization.inputs import load_inputs
from optimization.physics import build_model
from optimization.export import export_model


DEMO = Path(__file__).resolve().parents[1] / "optimization/examples/synthetic"


class ImportedModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.inputs = load_inputs(DEMO)
        cls.candidate = initial_candidate(cls.inputs, cls.inputs.requirements[0])

    def test_exported_arms_load_in_pose_mode_without_actuators(self):
        for number in (7, 8, 9):
            requirement = replace(self.inputs.requirements[0], allowed_layouts=(number,))
            candidate = initial_candidate(self.inputs, requirement)
            expected, xml = build_model(candidate, self.inputs)
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "best_model.xml"
                path.write_text(xml, encoding="utf-8")
                sim = Simulation.from_path(path)
                self.assertEqual(path.read_text(encoding="utf-8"), xml)
            self.assertTrue(sim.is_external)
            self.assertFalse(sim.supports_dynamics)
            self.assertFalse(sim.ratios_known)
            self.assertEqual(sim.model.nu, 0)
            self.assertFalse(np.any(sim.model.jnt_limited))
            np.testing.assert_allclose(sim.model.body_mass, expected.body_mass)
            np.testing.assert_allclose(sim.model.body_inertia, expected.body_inertia)
            np.testing.assert_allclose(sim.axes, expected.jnt_axis)
            self.assertAlmostEqual(sim.layout.l1_length, candidate.lengths_m[0])
            self.assertAlmostEqual(sim.layout.l2_length, candidate.lengths_m[1])
            self.assertAlmostEqual(sim.layout.l3_length, candidate.lengths_m[2])
            self.assertTrue(sim.layout.perpendicular_j4)
            np.testing.assert_allclose(sim.output_torques, np.zeros(6))
            sim.set_target(2, .6)
            sim.advance(.01)
            self.assertAlmostEqual(sim.data.qpos[2], .6)
            sim.dynamic = True
            sim.advance(.01)
            self.assertFalse(sim.dynamic)
            self.assertTrue(np.isfinite(sim.data.qpos).all())

    def test_unlimited_import_targets_are_not_zeroed_or_physically_clamped(self):
        _, xml = build_model(self.candidate, self.inputs)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "best_model.xml"
            path.write_text(xml)
            sim = Simulation.from_path(path)
        np.testing.assert_allclose(sim.limits, np.tile((-np.pi, np.pi), (6, 1)))
        desired = np.array([2*np.pi, .5, -.5, 0, .2, -.2])
        sim.reset(desired)
        np.testing.assert_allclose(sim.targets, desired)
        sim.set_target(0, 3*np.pi)
        self.assertEqual(sim.targets[0], 3*np.pi)
        with self.assertRaisesRegex(ValueError, "six finite"):
            sim.reset([0, 0])

    def test_published_continuous_export_preserves_unknown_ratings_and_limits_playback(self):
        inputs = replace(self.inputs,
                         motors=tuple(replace(m, capacity_model="rated_continuous",
                                              stall_torque_Nm=None, no_load_speed_rpm=None,
                                              max_speed_rpm=None) for m in self.inputs.motors),
                         reducers=tuple(replace(r, max_input_torque_Nm=None,
                                                rated_input_speed_rpm=2000.) for r in self.inputs.reducers))
        xml = export_model(self.candidate, inputs, inputs.requirements[0])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"real-policy.xml"
            path.write_text(xml, encoding="utf-8")
            sim = Simulation.from_path(path)
        self.assertTrue(sim.supports_dynamics)
        self.assertTrue(all(drive["max_input_torque_Nm"] is None
                            and drive["stall_torque_Nm"] is None for drive in sim.export_metadata["drives"]))
        sim.data.qvel[:] = .5*sim.speed_limits
        sim._apply_force_limits()
        self.assertTrue(np.all(sim.model.actuator_forcerange[:, 1] > 0))
        sim.data.qvel[:] = 1.01*sim.speed_limits
        sim._apply_force_limits()
        np.testing.assert_allclose(sim.model.actuator_forcerange, 0)

    def test_discovery_excludes_presets_and_keeps_extra_files(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            for name in ("best_model.xml", "My arm.xml", "ignore.urdf", LAYOUTS[0].filename):
                (folder / name).write_text("<mujoco/>")
            models = discover_imported_models(folder)
            self.assertEqual([model.filename for model in models], ["best_model.xml", "My arm.xml"])
            self.assertTrue(all(model.path.is_absolute() for model in models))
            self.assertTrue(all(model.number == 0 for model in models))

    def test_incompatible_models_raise_actionable_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            path = folder / "empty.xml"
            path.write_text("<mujoco/>")
            with self.assertRaisesRegex(ValueError, "six scalar hinge"):
                Simulation.from_path(path)
            _, xml = build_model(self.candidate, self.inputs)
            path.write_text(xml.replace('name="tool_frame"', 'name="other_tool"'))
            with self.assertRaisesRegex(ValueError, "tool_frame"):
                Simulation.from_path(path)
            with self.assertRaisesRegex(ValueError, "not found"):
                Simulation.from_path(folder / "missing.xml")

    def test_imported_scene_has_component_labels_frames_and_visibility_groups(self):
        from armsim.scene import SceneGeometry
        _, xml = build_model(self.candidate, self.inputs)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "best_model.xml"
            path.write_text(xml)
            sim = Simulation.from_path(path)
        scene = SceneGeometry(sim)
        scene.labels = "All components"
        scene.frames = "Selected drivetrain"
        scene.selected = 3
        labels = scene.update()
        self.assertTrue(any("motor" in item[1] for item in labels))
        self.assertTrue(any("fixed case" in item[1] for item in labels))
        self.assertGreaterEqual(np.count_nonzero(sim.model.geom_group == 1), 6)
        self.assertGreaterEqual(np.count_nonzero(sim.model.geom_group == 2), 6)
        self.assertGreater(scene.camera.distance, 0)
        self.assertTrue(np.isfinite(scene.camera.lookat).all())


if __name__ == "__main__":
    unittest.main()

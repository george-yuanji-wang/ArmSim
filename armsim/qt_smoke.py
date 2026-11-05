"""Integration checks against a real Qt window and its native OpenGL context."""

import json
from pathlib import Path
import time
import traceback

import numpy as np
from PySide6.QtCore import QPoint, QRectF, Qt, QEventLoop, QTimer
from PySide6.QtGui import QImage, QPainter
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from armsim.catalog import LAYOUTS, ROOT, WRIST_PROFILES, axes_for


class SmokeTest:
    def __init__(self, window, screenshot=None):
        self.window = window
        self.output = Path(screenshot) if screenshot else ROOT / ".artifacts/qt-viewport.png"

    @staticmethod
    def wait(milliseconds=150):
        QTest.qWait(milliseconds)

    def capture(self, path):
        view = self.window.viewport
        previous = view.capture_image
        view.capture(path)
        for _ in range(20):
            self.wait(50)
            if view.capture_image is not previous:
                break
        assert view.capture_image is not None and view.capture_image is not previous
        image = view.capture_image.convertToFormat(QImage.Format.Format_RGBA8888)
        pixels = np.ndarray((image.height(), image.width(), 4), dtype=np.uint8,
                            buffer=image.bits(), strides=(image.bytesPerLine(), 4, 1))
        # Require visible structural blue, not merely a non-black floor.
        blue = ((pixels[:, :, 2].astype(int) - pixels[:, :, 0].astype(int) > 60) &
                (pixels[:, :, 2] > 130))
        assert np.count_nonzero(blue) > 100, "Robot geometry is absent from the native viewport"
        assert (image.width(),image.height()) == view.capture_dimensions, (image.size(),view.capture_dimensions,view.size(),view.devicePixelRatio())

    def start(self):
        window, view = self.window, self.window.viewport
        try:
            assert view.isValid() and view.mjr is not None and not view.failed
            if self.window.sim.is_external:
                self.check_imported_model()
            context = view.context()
            for width, height in ((1000, 720), (1600, 1000), (2200, 1300)):
                window.resize(width, height)
                self.wait(180)
                assert view.width() == window.viewport_container.width()
                assert view.height() == window.viewport_container.height()
                assert view.context() is context, "Resizing recreated the OpenGL context"
            window.zoom_combo.setCurrentText("130%")
            self.wait(100)
            assert abs(QApplication.font().pointSizeF() - 13) < 0.01
            window.zoom_combo.setCurrentText("115%")
            window.resize(1600, 1000)
            self.wait(180)
            print("PASS: resizable Qt splitter, three window sizes, interface zoom, device-pixel viewport and persistent GL context.", flush=True)

            for layout in LAYOUTS:
                window.layout_combo.setCurrentIndex(layout.number - 1)
                self.wait(100)
                assert window.sim.layout.number == layout.number
                np.testing.assert_allclose(window.sim.targets,layout.bent_pose,atol=1e-12)
                l2=window.sim.data.xanchor[3]-window.sim.data.xanchor[2]
                cosine=abs(float(l2@window.sim.data.xaxis[3]))/np.linalg.norm(l2)
                assert abs(cosine-(0 if layout.perpendicular_j4 else 1))<1e-10
                assert view.gl_model is window.sim.model
                window.clear_selection()
                self.capture(ROOT / f".artifacts/qt-model-{layout.number}.png")
                shoulder=view.geometry.project(window.sim.data.xanchor[1])
                tool=view.geometry.project(window.sim.data.site('tool_frame').xpos)
                apex=max((view.geometry.project(window.sim.data.xanchor[2]),view.geometry.project(window.sim.data.xanchor[3])),key=lambda point:point[0])
                assert apex[0]>max(shoulder[0],tool[0])+10, 'Default posture is not a > silhouette'
                assert tool[1]<apex[1]<shoulder[1]
                control = window.joint_controls[2]
                control.spin.setValue(42.0)
                self.wait(40)
                np.testing.assert_allclose(window.sim.data.qpos[2], np.radians(42))
                assert control.slider.value() == 420
                window.frame_combo.setCurrentText("Selected drivetrain")
                window.label_combo.setCurrentText("Selected drive")
                window.reset(layout.bent_pose)
            print(f"PASS: all {len(LAYOUTS)} MJCF switches, angle fields/sliders, shaft frames, labels and real rendered robot pixels.", flush=True)
            for key in WRIST_PROFILES:
                window.wrist_combo.setCurrentIndex(list(WRIST_PROFILES).index(key))
                self.wait(180)
                np.testing.assert_allclose(window.sim.model.jnt_axis,axes_for(key))
                window.reset(np.zeros(6))
                window.update_readouts()
                assert 'Singular pose' in window.mobility_info.text()
                assert ('5/6' if window.sim.layout.perpendicular_j4 else '3/6') in window.mobility_info.text()
                window.reset(window.sim.layout.bent_pose)
                window.update_readouts()
                assert '6/6' in window.mobility_info.text()
                assert 'Singular pose' not in window.mobility_info.text()
            window.wrist_combo.setCurrentIndex(0)
            self.wait(180)

            control.spin.setFocus()
            control.spin.selectAll()
            QTest.keyClicks(control.spin, "-25.5")
            QTest.keyClick(control.spin, Qt.Key.Key_Return)
            np.testing.assert_allclose(window.sim.targets[2], np.radians(-25.5))
            window.reset(window.sim.layout.bent_pose)
            old_frames = window.frame_combo.currentIndex()
            QTest.keyClick(view, Qt.Key.Key_F)
            assert window.frame_combo.currentIndex() == (old_frames + 1) % window.frame_combo.count()
            window.frame_combo.setCurrentText("Selected drivetrain")

            # Exercise real Qt events, rather than only calling camera helpers.
            before = view.geometry.camera.azimuth
            QTest.mousePress(view, Qt.MouseButton.LeftButton, pos=QPoint(240, 240))
            QTest.mouseMove(view, QPoint(300, 270), 30)
            QTest.mouseRelease(view, Qt.MouseButton.LeftButton, pos=QPoint(300, 270))
            self.wait(100)
            assert view.geometry.camera.azimuth != before
            view.camera_preset("Home")
            self.wait(100)
            picked = 0
            last_pick = None
            for j in range(6):
                projected = view.geometry.project(window.sim.data.site(f"M{j + 1}").xpos)
                if projected is None:
                    continue
                window.select_joint((j + 1) % 6)
                QTest.mouseClick(view, Qt.MouseButton.LeftButton,
                                 pos=QPoint(round(projected[0]), round(projected[1])))
                self.wait(20)
                if view.geometry.selected == j:
                    picked += 1
                    last_pick=(j,QPoint(round(projected[0]),round(projected[1])))
            assert picked > 0, "Clicking visible motors did not select their joints"
            # Same component toggles off; empty space and Escape also clear it.
            window.select_joint(last_pick[0])
            self.wait(60)
            QTest.mouseClick(view,Qt.MouseButton.LeftButton,pos=last_pick[1])
            assert view.geometry.selected == -1
            window.select_joint(3)
            QTest.mouseClick(view,Qt.MouseButton.LeftButton,pos=QPoint(5,5))
            assert view.geometry.selected == -1
            window.select_joint(4)
            QTest.keyClick(view,Qt.Key.Key_Escape)
            assert view.geometry.selected == -1
            assert not any(bool(control.property('selected')) for control in window.joint_controls)
            print('PASS: deselection by reselect, background click and Escape; wrist comparison preserves J3/J4 orthogonality.',flush=True)
            window.visibility["motors"].setChecked(False)
            assert not view.geometry.visible["motors"]
            window.visibility["motors"].setChecked(True)
            window.dynamics_button.click()
            window.demo_button.click()
            self.wait(500)
            assert window.animation_timer.isActive()
            assert window.sim.data.time > 0
            assert np.isfinite(window.sim.data.qpos).all()
            assert not any(w.number for w in window.sim.data.warning)
            window.pause_button.click()
            assert not window.animation_timer.isActive()
            window.pose_button.click()
            window.reset(window.sim.layout.bent_pose)
            if not window.running:
                window.pause_button.click()
            self.wait(150)
            idle_frame = view.frame_number
            self.wait(400)
            assert view.frame_number - idle_frame <= 1, "Idle poses keep redrawing"
            print("PASS: native camera drag/picking, visibility, dynamics/demo, pause and no idle redraw loop.", flush=True)

            # Measure the complete native repaint cadence during animation.
            window.reset(window.sim.layout.bent_pose)
            window.clear_selection()
            window.demo_button.click()
            self.wait(300)
            view.paint_times.clear()
            initial = view.frame_number
            started = time.perf_counter()
            # Use the native event loop for cadence measurement. QTest.qWait
            # pumps events in polling batches, which can distort frame timing.
            benchmark_loop=QEventLoop()
            QTimer.singleShot(1400,benchmark_loop.quit)
            benchmark_loop.exec()
            elapsed = time.perf_counter() - started
            performance = {
                "renderer": view.renderer_name,
                "logical_viewport": [view.width(), view.height()],
                "device_pixel_ratio": view.devicePixelRatio(),
                "physical_viewport": [int(view.width()*view.devicePixelRatio()+0.5), int(view.height()*view.devicePixelRatio()+0.5)],
                "quality": view.quality,
                "animation_fps": round((view.frame_number - initial) / elapsed, 1),
                "median_cpu_paint_ms": round(float(np.median(view.paint_times)), 2),
                "p95_cpu_paint_ms": round(float(np.percentile(view.paint_times, 95)), 2),
                "steady_state_pixel_readbacks": 0,
            }
            (ROOT / ".artifacts/qt-performance.json").write_text(json.dumps(performance, indent=2) + "\n")
            print("PERFORMANCE " + json.dumps(performance), flush=True)
            window.demo_button.click()
            window.reset(window.sim.layout.bent_pose)
            window.select_joint(3)
            window.frame_combo.setCurrentText("Selected joint")
            self.wait(100)
            self.capture(self.output)
            window.clear_selection()
            self.wait(100)
            self.capture(self.output.with_name('no-selection.png'))

            # Export the actual Qt widget layout plus its native child surface
            # as a review artifact, without desktop screenshot automation.
            pixmap = window.grab()
            painter = QPainter(pixmap)
            try:
                origin = window.viewport_container.mapTo(window, QPoint(0, 0))
                painter.drawImage(QRectF(origin.x(), origin.y(), view.width(), view.height()), view.capture_image)
            finally:
                painter.end()
            pixmap.save(str(self.output.with_name("qt-interface.png")))
            assert not view.failed
            print("PASS: native Qt/OpenGL integration and PNG export.", flush=True)
        except Exception:
            window.exit_code = 1
            traceback.print_exc()
        finally:
            window.close()
            QApplication.instance().exit(window.exit_code)

    def check_imported_model(self):
        """Exercise a passive optimization export before checking the presets."""
        window, view = self.window, self.window.viewport
        source = window.sim.layout.path.resolve()
        assert window.layout_combo.findData(str(source)) >= len(LAYOUTS)
        actuated = window.sim.export_metadata is not None
        assert window.sim.supports_dynamics == actuated
        assert window.dynamics_button.isEnabled() == actuated
        assert window.sim.model.nu == (6 if actuated else 0)
        window.select_joint(3)
        window.frame_combo.setCurrentText("Selected drivetrain")
        window.label_combo.setCurrentText("All components")
        self.wait(100)
        annotations = view.geometry.update()
        assert any("motor" in text for _, text, _ in annotations)
        assert any("fixed case" in text for _, text, _ in annotations)
        before = window.sim.data.qpos.copy()
        window.joint_controls[1].spin.setValue(35.)
        self.wait(100)
        assert not np.array_equal(before, window.sim.data.qpos)
        np.testing.assert_allclose(window.sim.data.qpos[1], np.radians(35.))
        window.frame_combo.setCurrentText("All joints")
        self.wait(100)
        self.capture(ROOT / ".artifacts/imported-model.png")
        if actuated:
            window.dynamics_button.click()
            self.wait(350)
            assert window.sim.dynamic and window.sim.data.time > 0
            assert np.isfinite(window.sim.data.qpos).all()
            assert not any(warning.number for warning in window.sim.data.warning)
            np.testing.assert_array_less(np.abs(window.sim.data.actuator_force),
                                         window.sim.model.actuator_forcerange[:, 1]+1e-8)
            window.pose_button.click()
            window.reset(window.sim.layout.bent_pose)
            window.frame_combo.setCurrentText("Selected drivetrain")
            window.label_combo.setCurrentText("Selected drive")
            self.wait(100)
            self.capture(ROOT / ".artifacts/imported-model.png")
        pixmap = window.grab()
        painter = QPainter(pixmap)
        try:
            origin = window.viewport_container.mapTo(window, QPoint(0, 0))
            painter.drawImage(QRectF(origin.x(), origin.y(), view.width(), view.height()), view.capture_image)
        finally:
            painter.end()
        pixmap.save(str(ROOT / ".artifacts/imported-interface.png"))
        window.load_layout(1)
        self.wait(100)
        assert not window.sim.is_external and window.dynamics_button.isEnabled()
        window.layout_combo.setCurrentIndex(window.layout_combo.findData(str(source)))
        self.wait(100)
        assert window.sim.is_external and window.sim.layout.path.resolve() == source
        assert window.dynamics_button.isEnabled() == actuated
        window.refresh_model_list()
        self.wait(100)
        assert window.sim.is_external and window.layout_combo.findData(str(source)) >= len(LAYOUTS)
        window.frame_combo.setCurrentText("Selected joint")
        window.label_combo.setCurrentText("Selected drive")
        window.clear_selection()
        print("PASS: imported MJCF discovery, pose/dynamics capabilities, frames/labels, render, and preset/file switching.", flush=True)

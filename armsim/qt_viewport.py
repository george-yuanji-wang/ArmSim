"""Direct MuJoCo/OpenGL viewport. No per-frame image readback or raster copies."""

from collections import deque
from pathlib import Path
import math
import time
import traceback
import re

import mujoco
import numpy as np
from OpenGL.GL import (
    GL_ALL_ATTRIB_BITS, GL_CLIENT_ALL_ATTRIB_BITS, GL_ARRAY_BUFFER, GL_ELEMENT_ARRAY_BUFFER,
    GL_PIXEL_UNPACK_BUFFER, GL_RENDERER, GL_SCISSOR_TEST, GL_STENCIL_TEST,
    GL_TEXTURE0, GL_UNPACK_ALIGNMENT, GL_UNPACK_ROW_LENGTH,
    glActiveTexture, glBindBuffer, glDisable, glGetString, glPixelStorei, glUseProgram,
    glPushAttrib, glPopAttrib, glPushClientAttrib, glPopClientAttrib,
)
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QFontMetricsF, QPainter, QPen
from PySide6.QtOpenGL import QOpenGLWindow

from armsim.catalog import LINKS
from armsim.scene import SceneGeometry


class ArmViewport(QOpenGLWindow):
    jointPicked = Signal(int)
    renderFailed = Signal(str)
    captureSaved = Signal(str)

    def __init__(self, simulation):
        super().__init__(QOpenGLWindow.UpdateBehavior.NoPartialUpdate)
        self.sim = simulation
        self.geometry = SceneGeometry(simulation)
        self.mjr = None
        self.gl_model = None
        self.failed = False
        self.renderer_name = "Initializing OpenGL"
        self.quality = "Balanced"
        self.ui_scale = 1.15
        self.paint_times = deque(maxlen=120)
        self.paint_timestamps = deque(maxlen=120)
        self.frame_number = 0
        self.capture_path = None
        self.capture_image = None
        self.capture_dimensions = None
        self.drag_start = self.drag_last = None
        self.dragging = False

    def initializeGL(self):
        name = glGetString(GL_RENDERER)
        self.renderer_name = name.decode(errors="replace") if name else "OpenGL"
        self.context().aboutToBeDestroyed.connect(self.release_gl)

    def release_gl(self):
        if self.mjr is not None:
            self.makeCurrent()
            self.mjr.free()
            self.mjr = None
            self.gl_model = None
            self.doneCurrent()

    def set_simulation(self, simulation):
        previous = self.geometry
        self.sim = simulation
        self.geometry = SceneGeometry(simulation)
        if not (getattr(simulation, "is_external", False) or getattr(previous.sim, "is_external", False)):
            self.geometry.camera.lookat[:] = previous.camera.lookat
            self.geometry.camera.distance = previous.camera.distance
        self.geometry.camera.azimuth = previous.camera.azimuth
        self.geometry.camera.elevation = previous.camera.elevation
        self.geometry.selected = previous.selected
        self.geometry.frames, self.geometry.labels = previous.frames, previous.labels
        self.geometry.visible = previous.visible.copy()
        self.failed = False
        self.update()

    def paintGL(self):
        if self.failed:
            return
        started = time.perf_counter()
        try:
            if self.gl_model is not self.sim.model:
                if self.mjr is not None:
                    self.mjr.free()
                self.mjr = mujoco.MjrContext(self.sim.model, mujoco.mjtFontScale.mjFONTSCALE_150)
                self.gl_model = self.sim.model
            geometry = self.geometry
            geometry.width, geometry.height = self.width(), self.height()
            annotations = geometry.update()
            geometry.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = self.quality != "Fast"
            geometry.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = self.quality == "High"

            painter = QPainter(self)
            painter.beginNativePainting()
            glPushAttrib(GL_ALL_ATTRIB_BITS)
            glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS)
            # Qt's text painter uses shaders/buffers; MuJoCo uses the legacy
            # compatibility pipeline. Restore that pipeline before 3D drawing.
            glUseProgram(0)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)
            glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0)
            glActiveTexture(GL_TEXTURE0)
            glDisable(GL_SCISSOR_TEST)
            glDisable(GL_STENCIL_TEST)
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, self.mjr)
            ratio = self.devicePixelRatio()
            # Qt rounds half-pixels upward; Python round() uses ties-to-even.
            rectangle = mujoco.MjrRect(0, 0, int(self.width()*ratio+0.5), int(self.height()*ratio+0.5))
            mujoco.mjr_render(rectangle, geometry.scene, self.mjr)
            glPopClientAttrib()
            glPopAttrib()

            # Reset pixel-store state before Qt uploads glyphs to its texture
            # atlas; otherwise MuJoCo's unpack settings corrupt the glyphs.
            glActiveTexture(GL_TEXTURE0)
            glPixelStorei(GL_UNPACK_ALIGNMENT, 4)
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0)
            painter.endNativePainting()
            try:
                painter.setRenderHint(QPainter.RenderHint.Antialiasing)
                self.draw_annotations(painter, annotations)
                self.draw_gizmo(painter)
            finally:
                painter.end()

            self.frame_number += 1
            self.paint_times.append((time.perf_counter() - started) * 1000)
            self.paint_timestamps.append(time.perf_counter())
            if self.capture_path is not None:
                # Readback is performed only for an explicit export/test.
                path = self.capture_path
                self.capture_path = None
                self.capture_image = self.grabFramebuffer()
                self.capture_dimensions = (rectangle.width,rectangle.height)
                Path(path).parent.mkdir(parents=True, exist_ok=True)
                if not self.capture_image.save(path):
                    raise RuntimeError(f"Could not save image to {path}")
                self.captureSaved.emit(path)
        except Exception:
            self.failed = True
            self.renderFailed.emit(traceback.format_exc())

    def draw_annotations(self, painter, annotations):
        font = QFont("Segoe UI")
        font.setPointSizeF(9 * self.ui_scale)
        painter.setFont(font)
        metrics = QFontMetricsF(font)
        occupied = []
        annotations.sort(key=lambda item: item[1].endswith((".X", ".Y", ".Z")))
        for point, text, rgb in annotations:
            position = self.geometry.project(point)
            if position is None:
                continue
            x, y = position
            axis_label = text.endswith((".X", ".Y", ".Z"))
            width = metrics.horizontalAdvance(text) + (6 if axis_label else 8 if text=='tool0' else 16)
            height = metrics.height() + (4 if text=='tool0' else 8)
            candidates = []
            for dx in (12, -width - 12):
                for dy in (0, -height - 4, height + 4, -2 * height - 8, 2 * height + 8):
                    rect = QRectF(min(max(6, x + dx), max(6, self.width() - width - 6)),
                                  min(max(10, y - height / 2 + dy), max(10, self.height() - height - 80)), width, height)
                    overlap = sum(rect.intersected(other).width() * rect.intersected(other).height()
                                  for other in occupied if rect.intersects(other))
                    cost = (rect.center().x() - x) ** 2 + (rect.center().y() - y) ** 2 + overlap * 35
                    candidates.append((cost, rect))
            rect = min(candidates, key=lambda item: item[0])[1]
            occupied.append(rect)
            color = QColor(*rgb)
            painter.setPen(QPen(color, 1))
            if axis_label:
                # A compact dark backing keeps axes readable over the links.
                painter.setBrush(QColor(13, 21, 32, 190))
                painter.setPen(Qt.PenStyle.NoPen)
                painter.drawRoundedRect(rect, 3, 3)
                painter.setPen(color)
            else:
                painter.drawLine(QPointF(x, y), rect.center())
                painter.setBrush(QColor(18, 28, 42, 235))
                painter.drawRoundedRect(rect, 5, 5)
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, text)

    def draw_gizmo(self, painter):
        """Screen-space world-axis compass, using the same live camera basis."""
        origin = QPointF(57, self.height() - 53)
        camera = self.geometry.scene.camera[0]
        forward, up = np.asarray(camera.forward), np.asarray(camera.up)
        horizontal = np.cross(forward, up)
        colors = (QColor("#f45b69"), QColor("#57df87"), QColor("#60a7ff"))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(15, 24, 37, 220))
        painter.drawRoundedRect(QRectF(12, self.height() - 100, 93, 88), 8, 8)
        for index in sorted(range(3), key=lambda i: forward[i], reverse=True):
            end = origin + QPointF(float(horizontal[index]) * 29, -float(up[index]) * 29)
            painter.setPen(QPen(colors[index], 2))
            painter.drawLine(origin, end)
            painter.drawText(QRectF(end.x() - 9, end.y() - 10, 18, 20), Qt.AlignmentFlag.AlignCenter, "XYZ"[index])

    def capture(self, path):
        self.capture_path = str(path)
        self.update()

    def camera_preset(self, preset):
        camera = self.geometry.camera
        if preset == "Home":
            self.geometry.home_camera()
        elif preset == "Front":
            camera.azimuth, camera.elevation = 90, 0
        elif preset == "Side":
            camera.azimuth, camera.elevation = 0, 0
        elif preset == "Top":
            camera.azimuth, camera.elevation = 90, -89.9
        self.update()

    def mousePressEvent(self, event):
        self.requestActivate()
        self.drag_start = self.drag_last = event.position()
        self.dragging = False

    def mouseMoveEvent(self, event):
        if self.drag_last is None or not event.buttons():
            return
        position = event.position()
        delta = position - self.drag_last
        self.dragging |= (position - self.drag_start).manhattanLength() > 4
        action = (mujoco.mjtMouse.mjMOUSE_MOVE_V if event.buttons() & (Qt.MouseButton.RightButton | Qt.MouseButton.MiddleButton)
                  else mujoco.mjtMouse.mjMOUSE_ROTATE_V)
        self.geometry.move_camera(action, delta.x() / max(1, self.height()), delta.y() / max(1, self.height()))
        self.drag_last = position
        self.update()

    def mouseReleaseEvent(self, event):
        if not self.dragging and event.button() == Qt.MouseButton.LeftButton:
            self.pick(event.position())
        self.drag_last = None

    def pick(self, position):
        point = np.zeros(3)
        ids = [np.full(1, -1, dtype=np.int32) for _ in range(3)]
        body = mujoco.mjv_select(self.sim.model, self.sim.data, self.geometry.option,
                                self.width() / max(1, self.height()), position.x() / max(1, self.width()),
                                1 - position.y() / max(1, self.height()), self.geometry.scene, point, *ids)
        if ids[0][0]>=0:
            name=self.sim.model.geom(int(ids[0][0])).name or ""
            match=re.match(r'^(?:[JGMO]([1-6])_|input_(?:pulley|belt)_([1-6])_)',name)
            if match:
                self.jointPicked.emit(int(match.group(1) or match.group(2))-1)
                return
        while body > 0:
            name = self.sim.model.body(body).name or ""
            component = re.match(r'^(?:motor|gearbox|reducer|output_coupling)_([1-6])(?:$|_)', name)
            if component:
                self.jointPicked.emit(int(component.group(1)) - 1)
                return
            if name in LINKS[1:]:
                self.jointPicked.emit(LINKS.index(name) - 1)
                return
            if self.sim.model.body_jntnum[body]:
                self.jointPicked.emit(int(self.sim.model.body_jntadr[body]))
                return
            body = self.sim.model.body_parentid[body]
        self.jointPicked.emit(-1)

    def wheelEvent(self, event):
        steps = event.angleDelta().y() / 120
        self.geometry.camera.distance = np.clip(self.geometry.camera.distance * math.exp(-0.10 * steps), 0.35, 7.0)
        self.update()

    def keyPressEvent(self, event):
        # Forward navigation keys to the main window when the native child has
        # focus; window shortcuts remain accessible after camera interaction.
        if hasattr(self, "key_handler"):
            self.key_handler(event)


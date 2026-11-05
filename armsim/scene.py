"""MuJoCo rendering, moving coordinate frames, and projected component labels."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from armsim.catalog import AXIS_NAMES, LINKS

AXIS_COLORS = ((0.96, 0.27, 0.30, 1), (0.28, 0.90, 0.46, 1), (0.28, 0.60, 1.0, 1))


class SceneGeometry:
    """Camera, annotation geometry, and projection shared by both render paths."""

    def __init__(self, simulation, width=960, height=720, scene=None):
        self.sim = simulation
        if getattr(simulation, "is_external", False):
            self._classify_imported_geometry()
        self.scene = scene if scene is not None else mujoco.MjvScene(simulation.model, maxgeom=1024)
        self.width, self.height = width, height
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.home_camera()
        self.option = mujoco.MjvOption()
        self.option.sitegroup[:] = 0
        self.option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False
        self.visible = dict(structure=True, motors=True, gearboxes=True, routes=True, brackets=True, floor=True)
        self.frames = "Selected joint"
        self.labels = "Selected drive"
        self.selected = -1
        self.perturb = mujoco.MjvPerturb()

    def update(self):
        for index, name in enumerate(("structure", "motors", "gearboxes", "routes", "brackets", "floor")):
            self.option.geomgroup[index] = self.visible[name]
        self.option.tendongroup[:] = 0
        self.option.tendongroup[3] = self.visible["routes"]
        mujoco.mjv_updateScene(self.sim.model, self.sim.data, self.option, self.perturb,
                              self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        return self.annotations()

    def home_camera(self):
        self.camera.lookat[:] = [0, 0.02, 0.48]
        self.camera.distance = 1.9
        self.camera.azimuth = -45
        self.camera.elevation = -22
        if getattr(self.sim, "is_external", False):
            model, data = self.sim.model, self.sim.data
            points = [data.xanchor.copy()]
            for geom in range(model.ngeom):
                if model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_PLANE:
                    continue
                radius = model.geom_rbound[geom]
                points.extend((data.geom_xpos[geom][None, :]-radius,
                               data.geom_xpos[geom][None, :]+radius))
            points = np.vstack(points)
            lower, upper = points.min(axis=0), points.max(axis=0)
            self.camera.lookat[:] = (lower+upper)/2
            self.camera.distance = max(.65, 2.3*float(np.max(upper-lower)))

    def _classify_imported_geometry(self):
        """Give known ArmSim components display groups without editing the file."""
        model = self.sim.model
        for geom in range(model.ngeom):
            body = int(model.geom_bodyid[geom])
            while body > 0:
                name = model.body(body).name or ""
                if name.startswith("motor_"):
                    group, color = 1, (.16, .76, .62, 1.)
                elif name.startswith(("reducer_", "gearbox_")):
                    group, color = 2, (1., .65, .22, 1.)
                elif name.startswith("output_coupling_"):
                    group, color = 2, (.62, .67, .74, 1.)
                else:
                    body = int(model.body_parentid[body])
                    continue
                model.geom_group[geom] = group
                model.geom_matid[geom] = -1
                model.geom_rgba[geom] = color
                break
            else:
                if model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_PLANE:
                    model.geom_group[geom] = 5

    def marker(self, prefix, joint):
        """Use authored marker sites, or the corresponding imported body frame."""
        model, data = self.sim.model, self.sim.data
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{prefix}{joint}")
        if site_id >= 0:
            return data.site_xpos[site_id].copy(), data.site_xmat[site_id].reshape(3, 3).copy()
        if prefix == "J":
            index = joint-1
            body = model.jnt_bodyid[index]
            return data.xanchor[index].copy(), data.xmat[body].reshape(3, 3).copy()
        names = {"M": (f"motor_{joint}",),
                 "G": (f"gearbox_{joint}", f"reducer_{joint}"),
                 "O": (f"output_coupling_{joint}",)}
        for name in names.get(prefix, ()):
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id >= 0:
                return data.xpos[body_id].copy(), data.xmat[body_id].reshape(3, 3).copy()
        return None

    def tool_transform(self):
        model, data = self.sim.model, self.sim.data
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tool_frame")
        if site >= 0:
            return data.site_xpos[site].copy(), data.site_xmat[site].reshape(3, 3).copy()
        body = int(model.jnt_bodyid[-1])
        return data.xpos[body].copy(), data.xmat[body].reshape(3, 3).copy()


    def move_camera(self, action, dx, dy):
        mujoco.mjv_moveCamera(self.sim.model, action, dx, dy, self.camera)


    def project(self, point):
        # Renderer uses a stereo camera pair even for a mono image. Average
        # their positions to match the rendered cyclopean (central) view.
        left, right = self.scene.camera
        position = (left.pos + right.pos) / 2
        forward = np.array(left.forward, dtype=float)
        forward /= np.linalg.norm(forward)
        up = np.array(left.up, dtype=float)
        up /= np.linalg.norm(up)
        horizontal = np.cross(forward, up)
        relative = np.asarray(point) - position
        depth = np.dot(relative, forward)
        if depth <= left.frustum_near:
            return None
        factor = 1.0 if left.orthographic else left.frustum_near / depth
        span = left.frustum_top - left.frustum_bottom
        x = self.width / 2 + np.dot(relative, horizontal) * factor * self.height / span
        y = (left.frustum_top - np.dot(relative, up) * factor) * self.height / span
        return (float(x), float(y)) if 0 <= x < self.width and 0 <= y < self.height else None


    def arrow(self, start, end, color, width=0.003):
        scene = self.scene
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.array(color, dtype=np.float32))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, width, np.asarray(start), np.asarray(end))
        geom.category = mujoco.mjtCatBit.mjCAT_DECOR
        scene.ngeom += 1


    def frame(self, origin, rotation, name, labels, length=0.075):
        for index, color in enumerate(AXIS_COLORS):
            endpoint = origin + rotation[:, index] * length
            self.arrow(origin, endpoint, color)
            axis_label = f"{name}.{ 'XYZ'[index] }"
            if name == 'J4' and index == 2 and self.sim.axis_names[3] == 'Z':
                axis_label += ' (L3 roll)'
            if name == 'J6' and index == 2 and self.sim.axis_names[5] == 'Z':
                axis_label += ' (tool roll)'
            labels.append((endpoint, axis_label, tuple(round(c * 255) for c in color[:3])))


    def annotations(self):
        model, data = self.sim.model, self.sim.data
        labels = []
        if self.frames != "None":
            self.frame(np.array([0., 0., 0.008]), np.eye(3), "W", labels, 0.12)
            ids = range(6) if self.frames == "All joints" else ([self.selected] if self.selected >= 0 else [])
            for index in ids:
                body = model.jnt_bodyid[index]
                self.frame(data.xanchor[index], data.xmat[body].reshape(3, 3), f"J{index + 1}", labels, 0.105 if index == self.selected else 0.06)
            if self.frames == "Selected drivetrain" and self.selected >= 0:
                j = self.selected + 1
                for prefix in ("M", "G", "O"):
                    marker = self.marker(prefix, j)
                    if marker is not None:
                        self.frame(*marker, f"{prefix}{j}", labels, 0.065)
            self.frame(*self.tool_transform(), "T", labels)
        if self.labels != "None":
            ids = [] if self.labels == "Links" else range(6) if self.labels in ("All components", "Joints") else ([self.selected] if self.selected >= 0 else [])
            for index in ids:
                j = index + 1
                labels.append((self.marker("J", j)[0], f"J{j}  {self.sim.axis_names[index]}", (226, 234, 248)))
                if self.labels != "Joints":
                    if self.visible["motors"]:
                        marker = self.marker("M", j)
                        if marker is not None:
                            labels.append((marker[0], f"M{j} motor case", (80, 223, 184)))
                    if self.visible["gearboxes"]:
                        for prefix, caption, color in (("G", "fixed case", (255, 185, 86)),
                                                       ("O", "moving flange", (214, 224, 238))):
                            marker = self.marker(prefix, j)
                            if marker is not None:
                                labels.append((marker[0], f"{prefix}{j} {caption}", color))
            if self.labels in ("Links", "All components") and self.visible["structure"]:
                names = LINKS if not getattr(self.sim, "is_external", False) else tuple(dict.fromkeys(
                    model.body(body).name for body in model.jnt_bodyid))
                for name in names:
                    if not name:
                        continue
                    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                    if body < 0:
                        continue
                    length_labels = {"upper_arm_link": f"L1 ({getattr(self.sim.layout, 'l1_length', .32)*1000:.0f} mm)",
                                     "forearm_link": f"L2 ({self.sim.layout.l2_length*1000:.0f} mm)",
                                     "wrist_1_link": f"L3 ({self.sim.layout.l3_length*1000:.0f} mm)"}
                    labels.append((data.xipos[body].copy(), length_labels.get(name, name), (141, 187, 248)))
            labels.append((self.tool_transform()[0], "tool0", (255, 185, 86)))
        return labels


class Scene(SceneGeometry):
    """Offscreen image export. The interactive Qt viewport renders directly to GL."""

    def __init__(self, simulation, width=960, height=720):
        simulation.model.vis.global_.offwidth = max(width, simulation.model.vis.global_.offwidth)
        simulation.model.vis.global_.offheight = max(height, simulation.model.vis.global_.offheight)
        self.renderer = mujoco.Renderer(simulation.model, width=width, height=height, max_geom=1024)
        super().__init__(simulation, width, height, self.renderer.scene)
        self.last_image = None
        self.text_scale = None
        self.set_text_scale(1.0)

    def set_text_scale(self, scale):
        if scale == self.text_scale:
            return
        self.text_scale = scale
        font_path = Path("C:/Windows/Fonts/segoeui.ttf")
        self.font = ImageFont.truetype(str(font_path), round(15 * scale)) if font_path.exists() else ImageFont.load_default()
        self.font_bold = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", round(17 * scale)) if font_path.exists() else self.font


    def resize(self, width, height):
        """Render at the viewport's real resolution, preserving the camera."""
        self.renderer.close()
        self.sim.model.vis.global_.offwidth = max(width, self.sim.model.vis.global_.offwidth)
        self.sim.model.vis.global_.offheight = max(height, self.sim.model.vis.global_.offheight)
        self.renderer = mujoco.Renderer(self.sim.model, width=width, height=height, max_geom=1024)
        self.width, self.height = width, height
        self.scene = self.renderer.scene


    def close(self):
        self.renderer.close()


    def render(self):
        for index, name in enumerate(("structure", "motors", "gearboxes", "routes", "brackets", "floor")):
            self.option.geomgroup[index] = self.visible[name]
        self.option.tendongroup[:] = 0
        self.option.tendongroup[3] = self.visible["routes"]
        self.renderer.update_scene(self.sim.data, camera=self.camera, scene_option=self.option)
        annotations = self.annotations()
        image = Image.fromarray(self.renderer.render())
        draw = ImageDraw.Draw(image)
        s = lambda value: round(value * self.text_scale)
        occupied = []
        # Component labels take priority. Axis labels stay close to their arrow
        # tips, with small text instead of boxes that obscure nearby frames.
        annotations.sort(key=lambda item: item[1].endswith((".X", ".Y", ".Z")))
        for point, text, color in annotations:
            projected = self.project(point)
            if projected is None:
                continue
            x, y = projected
            box = draw.textbbox((0, 0), text, font=self.font)
            axis_label = text.endswith((".X", ".Y", ".Z"))
            w, h = (box[2] + s(4), s(22)) if axis_label else (box[2] + s(12), s(27))
            candidates = []
            offsets = tuple(s(value) for value in ((0, -24, 24, -48, 48) if axis_label else (0, -32, 32, -64, 64)))
            for dx in ((s(5), -w - s(5)) if axis_label else (s(14), -w - s(14))):
                for dy in offsets:
                    cx = min(max(6, x + dx), max(6, self.width - w - 6))
                    cy = min(max(s(86), y - h / 2 + dy), max(s(86), self.height - h - s(38)))
                    overlap = sum(max(0, min(cx + w, ox + ow) - max(cx, ox)) *
                                  max(0, min(cy + h, oy + oh) - max(cy, oy))
                                  for ox, oy, ow, oh in occupied)
                    distance = (cx + w / 2 - x) ** 2 + (cy + h / 2 - y) ** 2
                    candidates.append((distance + overlap * 30, cx, cy))
            _, lx, ly = min(candidates)
            occupied.append((lx, ly, w, h))
            if axis_label:
                draw.text((lx + s(2), ly), text, fill=color, font=self.font, stroke_width=s(2), stroke_fill=(15, 22, 32))
            else:
                draw.line((x, y, lx + w / 2, ly + h / 2), fill=color, width=1)
                draw.rounded_rectangle((lx, ly, lx + w, ly + h), radius=4, fill=(17, 24, 36), outline=color, width=1)
                draw.text((lx + s(6), ly + s(2)), text, fill=color, font=self.font)
        title = (f"IMPORTED  {self.sim.layout.title.upper()}" if getattr(self.sim, "is_external", False)
                 else f"{self.sim.layout.number:02d}  {self.sim.layout.title.upper()}")
        draw.rounded_rectangle((s(14), s(14), min(self.width - s(14), s(480)), s(77)), radius=s(6), fill=(14, 21, 33))
        draw.text((s(25), s(21)), title, font=self.font_bold, fill=(234, 242, 255))
        subtitle = "POSE EXPLORATION" if not self.sim.dynamic else "TORQUE-LIMITED DYNAMICS"
        draw.text((s(25), s(49)), subtitle + "   |   " + " - ".join(self.sim.axis_names), font=self.font, fill=(150, 172, 201))
        draw.text((s(18), self.height - s(29)), "X red   Y green   Z blue  |  W world   J joint   T tool", font=self.font, fill=(205, 216, 232), stroke_width=1, stroke_fill=(18, 23, 33))
        self.last_image = image
        return image



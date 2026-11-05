"""Qt desktop explorer with a direct OpenGL MuJoCo viewport."""

from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path
import sys
import time
import traceback

import numpy as np
from PySide6.QtCore import QSettings, QSignalBlocker, Qt, QTimer
from PySide6.QtGui import QAction, QFont, QKeySequence, QPalette, QColor, QSurfaceFormat
from PySide6.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QFrame, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton, QScrollArea,
    QSizePolicy, QSlider, QSplitter, QTabWidget, QTextBrowser, QToolBar, QVBoxLayout, QWidget,
)

from armsim.catalog import AXIS_NAMES, LAYOUTS, MODEL_DIR, layout_by_number, WRIST_PROFILES, DEFAULT_WRIST
from armsim.qt_viewport import ArmViewport
from armsim.simulation import Simulation
from armsim.kinematics import tool_mobility

STYLE = """
QWidget { color: #e6edf8; font-family: 'Segoe UI'; }
QMainWindow, QWidget#workspace { background: #101823; }
QFrame#header { background: #141f2e; border-bottom: 1px solid #2b3b50; }
QFrame#sidebar { background: #182332; border: 1px solid #2b3c51; border-radius: 10px; }
QFrame#jointCard { background: #1d2b3e; border: 1px solid #31435a; border-radius: 8px; }
QFrame#jointCard[selected='true'] { background: #1d3342; border: 1px solid #49cfb3; }
QLabel { background: transparent; border: none; }
QLabel#muted { color: #a1b3c9; }
QLabel#brand { color: #f1f6ff; font-weight: 700; font-size: 20pt; }
QLabel#sectionTitle { color: #eaf2ff; font-weight: 600; }
QLabel#logo { background: #42c9ad; color: #10272b; font-size: 16pt; font-weight: 800; border-radius: 10px; padding: 9px; }
QLabel#pill { background: #23374b; color: #95d9ca; border-radius: 5px; padding: 5px 9px; }
QPushButton { background: #26394f; border: 1px solid #3b5069; border-radius: 6px; padding: 7px 11px; }
QPushButton:hover { background: #324c65; border-color: #62819c; }
QPushButton:pressed { background: #1d3043; }
QPushButton:checked { background: #245449; color: #aeffe6; border-color: #46cfad; }
QPushButton#primary { background: #3bc4a7; color: #0a2825; border-color: #59ddc0; font-weight: 600; }
QPushButton:focus, QComboBox:focus, QDoubleSpinBox:focus, QSlider:focus { border-color: #56dbbf; }
QPushButton:disabled { color: #6f8196; background: #202c3b; border-color: #29384a; }
QComboBox, QDoubleSpinBox { background: #101e2e; border: 1px solid #3a5069; border-radius: 6px; padding: 6px 8px; selection-background-color: #2d6458; }
QComboBox { padding-right: 22px; }
QComboBox::drop-down { border: none; width: 22px; }
QComboBox QAbstractItemView { background: #1a2a3c; color: #e7f1fc; selection-background-color: #2b685d; padding: 4px; }
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { width: 18px; border-left: 1px solid #3a5069; }
QSlider::groove:horizontal { height: 5px; background: #384d65; border-radius: 2px; }
QSlider::sub-page:horizontal { background: #47cdb1; border-radius: 2px; }
QSlider::handle:horizontal { background: #e1fff5; border: 2px solid #42c9ad; width: 15px; margin: -6px 0; border-radius: 8px; }
QTabWidget::pane { border: none; }
QTabBar::tab { background: #182332; color: #9dafc5; border-bottom: 2px solid #293c51; padding: 10px 16px; }
QTabBar::tab:selected { color: #dffff4; border-bottom: 2px solid #4cdbb9; }
QTabBar::tab:hover { background: #22354a; }
QScrollArea, QTextBrowser { background: transparent; border: none; }
QScrollBar:vertical { background: #172333; width: 10px; margin: 2px; }
QScrollBar::handle:vertical { background: #49627c; border-radius: 4px; min-height: 30px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QCheckBox { spacing: 8px; padding: 5px 0; }
QCheckBox::indicator { width: 15px; height: 15px; border: 1px solid #5a738e; border-radius: 3px; background: #122133; }
QCheckBox::indicator:checked { background: #44caaa; border: 2px solid #a1ffe2; }
QSplitter::handle { background: #101823; width: 9px; }
QSplitter::handle:hover { background: #34566b; }
QToolTip { background: #263d52; color: #f1f7ff; border: 1px solid #68859e; padding: 7px; }
QToolBar { background: transparent; border: none; spacing: 5px; }
QToolButton { background: #26394f; border: 1px solid #3b5069; border-radius: 6px; padding: 7px 10px; }
QToolButton:hover { background: #324c65; }
QMenuBar, QMenu { background: #162333; color: #deebf9; }
QMenu::item:selected { background: #2b514f; }
"""


def label(text="", muted=False, wrap=False):
    widget = QLabel(text)
    if muted:
        widget.setObjectName("muted")
    widget.setWordWrap(wrap)
    return widget


def button(text, callback, checkable=False, primary=False):
    widget = QPushButton(text)
    widget.setCheckable(checkable)
    if primary:
        widget.setObjectName("primary")
    widget.clicked.connect(callback)
    return widget


def scroll_page():
    scroll = QScrollArea()
    scroll.setWidgetResizable(True)
    scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    content = QWidget()
    layout = QVBoxLayout(content)
    layout.setContentsMargins(5, 14, 12, 12)
    layout.setSpacing(10)
    scroll.setWidget(content)
    return scroll, layout


class JointControl(QFrame):
    def __init__(self, index, limits, select, target):
        super().__init__()
        self.index = index
        self.setObjectName("jointCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 9, 12, 9)
        layout.setSpacing(6)
        top = QHBoxLayout()
        self.select_button = button(f"J{index + 1}  ·  {AXIS_NAMES[index]}", lambda: select(index, toggle=True))
        self.select_button.setFlat(True)
        self.select_button.setToolTip("Select this joint and display its coordinate frame")
        top.addWidget(self.select_button)
        top.addStretch()
        self.spin = QDoubleSpinBox()
        self.spin.setRange(*np.degrees(limits))
        self.spin.setDecimals(1)
        self.spin.setSingleStep(1)
        self.spin.setSuffix("°")
        self.spin.setKeyboardTracking(False)
        self.spin.setAccessibleName(f"Joint {index + 1} {AXIS_NAMES[index]} target angle in degrees")
        self.spin.setMinimumWidth(104)
        top.addWidget(self.spin)
        layout.addLayout(top)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(round(math.degrees(limits[0]) * 10), round(math.degrees(limits[1]) * 10))
        self.slider.setSingleStep(10)
        self.slider.setPageStep(50)
        self.slider.setAccessibleName(f"Joint {index + 1} target angle")
        self.slider.sliderPressed.connect(lambda: select(index))
        self.slider.valueChanged.connect(lambda value: target(index, value / 10))
        self.spin.valueChanged.connect(lambda value: target(index, value))
        layout.addWidget(self.slider)
        self.readout = label("", muted=True)
        layout.addWidget(self.readout)

    def set_target(self, degrees):
        with QSignalBlocker(self.spin), QSignalBlocker(self.slider):
            self.spin.setValue(degrees)
            self.slider.setValue(round(degrees * 10))

    def set_selected(self, selected):
        if self.property("selected") == selected:
            return
        self.setProperty("selected", selected)
        self.style().unpolish(self)
        self.style().polish(self)
        self.update()


class Explorer(QMainWindow):
    def __init__(self, number=1, smoke_test=False, wrist_profile=DEFAULT_WRIST, model_path=None):
        super().__init__()
        self.setWindowTitle("ArmSim | Drivetrain explorer · Qt")
        self.setMinimumSize(920, 640)
        self.resize(1540, 960)
        self.smoke_test = smoke_test
        self.settings = QSettings("ArmSim", "QtExplorer")
        self.exit_code = 0
        self.running = True
        self.sim = (Simulation.from_path(model_path) if model_path is not None
                    else Simulation(layout_by_number(number), wrist_profile))
        self._extra_model_paths = set()
        if self.is_imported_model:
            self._extra_model_paths.add(self.sim.layout.path.resolve())
        self.viewport = ArmViewport(self.sim)
        self.viewport.jointPicked.connect(lambda index: self.select_joint(index, reveal=True, toggle=True))
        self.viewport.renderFailed.connect(self.render_failed)
        self.viewport.key_handler = self.native_key
        self.manifest = {(entry["number"],entry["wrist_profile"]): entry for entry in json.loads((MODEL_DIR / "layouts.json").read_text(encoding="utf-8"))}
        self.last_tick = time.perf_counter()
        self.animation_timer = QTimer(self)
        self.animation_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.animation_timer.setInterval(16)
        self.animation_timer.timeout.connect(self.advance)
        self.info_timer = QTimer(self)
        self.info_timer.setInterval(150)
        self.info_timer.timeout.connect(self.update_readouts)
        self.info_timer.start()
        self.build_ui()
        if not smoke_test:
            saved_splitter = self.settings.value("splitter_state")
            if saved_splitter is not None:
                self.splitter.restoreState(saved_splitter)
        self.build_menu()
        self.apply_layout_info()
        self.clear_selection()
        zoom = 115 if smoke_test else int(self.settings.value("interface_zoom", 115))
        self.zoom_combo.setCurrentText(f"{zoom}%")
        self.set_zoom(self.zoom_combo.currentText())
        self.sync_targets()
        self.update_readouts()

    @property
    def is_imported_model(self):
        return bool(getattr(self.sim, "is_external", False))

    def build_ui(self):
        central = QWidget()
        central.setObjectName("workspace")
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        header = QFrame()
        header.setObjectName("header")
        header_row = QHBoxLayout(header)
        header_row.setContentsMargins(20, 13, 20, 13)
        logo = label("AS")
        logo.setObjectName("logo")
        header_row.addWidget(logo)
        brand = QVBoxLayout()
        title = label("ArmSim")
        title.setObjectName("brand")
        brand.addWidget(title)
        brand.addWidget(label("Motor & gearbox explorer", muted=True))
        header_row.addLayout(brand)
        header_row.addStretch()
        header_row.addWidget(label("Interface size", muted=True))
        self.zoom_combo = QComboBox()
        self.zoom_combo.addItems(["100%", "115%", "130%", "150%"])
        self.zoom_combo.setAccessibleName("Interface text and control size")
        self.zoom_combo.currentTextChanged.connect(self.set_zoom)
        header_row.addWidget(self.zoom_combo)
        header_row.addWidget(button("Shortcuts  ?", self.show_help))
        outer.addWidget(header)

        self.splitter = QSplitter(Qt.Orientation.Horizontal)
        self.splitter.setChildrenCollapsible(False)
        self.splitter.setHandleWidth(10)
        body = QWidget()
        body_row = QHBoxLayout(body)
        body_row.setContentsMargins(14, 14, 14, 14)
        body_row.addWidget(self.splitter)
        outer.addWidget(body, 1)
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setMinimumWidth(340)
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(16, 16, 16, 10)
        side.setSpacing(12)
        side.addWidget(label("ROBOT LAYOUT", muted=True))
        selector = QHBoxLayout()
        selector.addWidget(button("‹", lambda: self.switch(-1)))
        self.layout_combo = QComboBox()
        self.refresh_models()
        self.layout_combo.setMinimumContentsLength(10)
        self.layout_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.layout_combo.setAccessibleName("Robot motor and gearbox layout")
        self.layout_combo.currentIndexChanged.connect(self.load_model_selection)
        selector.addWidget(self.layout_combo, 1)
        selector.addWidget(button("›", lambda: self.switch(1)))
        side.addLayout(selector)
        file_actions = QHBoxLayout()
        file_actions.addWidget(button("Open MJCF...", self.open_mjcf))
        file_actions.addWidget(button("Refresh models", self.refresh_models))
        side.addLayout(file_actions)
        wrist_row = QHBoxLayout()
        self.wrist_combo = QComboBox()
        for key,value in WRIST_PROFILES.items():
            self.wrist_combo.addItem(value[0],key)
        self.wrist_combo.setCurrentIndex(list(WRIST_PROFILES).index(self.sim.wrist_profile))
        self.wrist_combo.setAccessibleName("Wrist rotation axes J4 and J6")
        self.wrist_combo.setEnabled(len(WRIST_PROFILES)>1)
        self.wrist_combo.currentIndexChanged.connect(lambda _: self.load_layout(self.sim.layout.number,self.wrist_combo.currentData()))
        wrist_row.addWidget(self.wrist_combo,1)
        if len(WRIST_PROFILES)>1:
            wrist_row.addWidget(button("Compare",self.compare_wrist))
        side.addLayout(wrist_row)
        self.description = label(wrap=True, muted=True)
        self.description.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        side.addWidget(self.description)
        self.mass_info = label()
        self.mass_info.setObjectName("pill")
        side.addWidget(self.mass_info)
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        side.addWidget(self.tabs, 1)
        self.build_joint_page()
        self.build_view_page()
        self.details = QTextBrowser()
        self.details.setOpenExternalLinks(False)
        self.tabs.addTab(self.details, "Drivetrain")
        side.addWidget(label("Drag the divider to resize this panel.", muted=True))
        self.splitter.addWidget(sidebar)

        workspace = QWidget()
        work = QVBoxLayout(workspace)
        work.setContentsMargins(4, 0, 0, 0)
        work.setSpacing(10)
        self.workspace_title = label()
        self.workspace_title.setWordWrap(True)
        self.workspace_title.setObjectName("sectionTitle")
        work.addWidget(self.workspace_title)
        self.kinematic_note = label(muted=True,wrap=True)
        work.addWidget(self.kinematic_note)
        self.mobility_info = label(wrap=True)
        self.mobility_info.setToolTip('Independent tool motions at the current pose, computed from the full position-and-orientation Jacobian. Six joints can lose independent tool motion at a singular pose.')
        work.addWidget(self.mobility_info)
        tools = QToolBar()
        tools.setMovable(False)
        tools.setFloatable(False)
        self.mode_badge = label("POSE EXPLORATION")
        self.mode_badge.setObjectName("pill")
        tools.addWidget(self.mode_badge)
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        tools.addWidget(spacer)
        for name in ("Front", "Side", "Top", "Home"):
            item = QAction(name, self)
            item.triggered.connect(lambda _, value=name: self.viewport.camera_preset(value))
            item.setToolTip(f"{name} camera view")
            tools.addAction(item)
        tools.addAction("Save PNG", self.save_image)
        tools.addAction("Deselect", self.clear_selection)
        work.addWidget(tools)
        self.viewport_container = QWidget.createWindowContainer(self.viewport, workspace)
        self.viewport_container.setMinimumSize(320, 300)
        self.viewport_container.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.viewport_container.setAccessibleName("Interactive 3D robot viewport: drag to orbit, right-drag to pan, wheel to zoom")
        work.addWidget(self.viewport_container, 1)
        legend = label("<span style='color:#79a7ef'>● Links</span> &nbsp; "
                       "<span style='color:#55dfbd'>● Motor cases</span> &nbsp; "
                       "<span style='color:#ffc26f'>● Fixed cases</span> &nbsp; "
                       "<span style='color:#bbc5d4'>● Moving outputs</span> &nbsp; "
                       "<span style='color:#bf95f7'>● Routing</span>"
                       " &nbsp; · &nbsp; Click a component to select its joint", wrap=True)
        legend.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        work.addWidget(legend)
        self.status = label(muted=True, wrap=True)
        work.addWidget(self.status)
        self.splitter.addWidget(workspace)
        self.splitter.setStretchFactor(0, 0)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setSizes([450, 1090])

    def build_joint_page(self):
        page, layout = scroll_page()
        self.joint_scroll = page
        mode_row = QHBoxLayout()
        self.mode_group = QButtonGroup(self)
        self.pose_button = button("Pose", lambda: self.change_mode(False), checkable=True)
        self.dynamics_button = button("Dynamics", lambda: self.change_mode(True), checkable=True)
        self.mode_group.addButton(self.pose_button)
        self.mode_group.addButton(self.dynamics_button)
        self.pose_button.setChecked(True)
        mode_row.addWidget(self.pose_button)
        mode_row.addWidget(self.dynamics_button)
        layout.addLayout(mode_row)
        self.mode_hint = label("Set joint angles directly. Use Dynamics to test the servos.", muted=True, wrap=True)
        layout.addWidget(self.mode_hint)
        self.assist = QCheckBox("Gravity / bias assist")
        self.assist.setToolTip("Adds external bias-compensation forces outside the motor torque limits.")
        self.assist.toggled.connect(self.set_assist)
        self.assist.hide()
        layout.addWidget(self.assist)
        actions = QHBoxLayout()
        actions.addWidget(button("Bent pose", lambda: self.reset(self.sim.layout.bent_pose)))
        actions.addWidget(button("Zero", lambda: self.reset(np.zeros(6))))
        self.pause_button = button("Pause", self.toggle_pause)
        actions.addWidget(self.pause_button)
        layout.addLayout(actions)
        self.demo_button = button("Animate demo", self.toggle_demo, checkable=True, primary=True)
        layout.addWidget(self.demo_button)
        self.joint_controls = []
        for index in range(6):
            control = JointControl(index, self.sim.limits[index], self.select_joint, self.set_target)
            self.joint_controls.append(control)
            layout.addWidget(control)
        self.mount_info = label(muted=True, wrap=True)
        layout.addWidget(self.mount_info)
        layout.addStretch()
        self.tabs.addTab(page, "Joints")

    def build_view_page(self):
        page, layout = scroll_page()
        layout.addWidget(label("Coordinate frames"))
        self.frame_combo = QComboBox()
        self.frame_combo.addItems(["Selected joint", "Selected drivetrain", "All joints", "None"])
        self.frame_combo.currentTextChanged.connect(lambda value: self.set_overlay("frames", value))
        layout.addWidget(self.frame_combo)
        layout.addWidget(label("Component labels"))
        self.label_combo = QComboBox()
        self.label_combo.addItems(["Selected drive", "Joints", "Links", "All components", "None"])
        self.label_combo.currentTextChanged.connect(lambda value: self.set_overlay("labels", value))
        layout.addWidget(self.label_combo)
        layout.addWidget(label("Render quality"))
        self.quality_combo = QComboBox()
        self.quality_combo.addItems(["Fast", "Balanced", "High"])
        self.quality_combo.setCurrentText("Balanced")
        self.quality_combo.setToolTip("Fast: no shadows. Balanced: shadows. High: shadows and floor reflections.")
        self.quality_combo.currentTextChanged.connect(self.set_quality)
        layout.addWidget(self.quality_combo)
        layout.addWidget(label("Visible components"))
        self.visibility = {}
        for name, title in (("structure", "Structural links · blue / gray"), ("motors", "Motor housings · teal"),
                            ("gearboxes", "Fixed housings · orange / moving outputs · grey"), ("routes", "Input drives · purple"),
                            ("brackets", "Mounting brackets"), ("floor", "Floor grid")):
            check = QCheckBox(title)
            check.setChecked(True)
            check.toggled.connect(lambda checked, key=name: self.set_visibility(key, checked))
            self.visibility[name] = check
            layout.addWidget(check)
        layout.addWidget(label("Right-handed frames: X red, Y green, Z blue.\nW = world · J = joint · T = tool0\nM = motor · G = reducer · O = moving output flange\n\nJ4 rolls L3 about its length (local Z), perpendicular to J3 X. Motor and reducer shafts stay parallel to their joint axes. Small mounting offsets keep the housings beside the open links.", muted=True, wrap=True))
        layout.addStretch()
        self.tabs.addTab(page, "View")

    def build_menu(self):
        file_menu = self.menuBar().addMenu("File")
        file_menu.addAction("Open MJCF...", self.open_mjcf, QKeySequence.StandardKey.Open)
        file_menu.addAction("Refresh model folder", self.refresh_models, QKeySequence("F5"))
        file_menu.addAction("Save viewport PNG…", self.save_image, QKeySequence.StandardKey.Save)
        file_menu.addAction("Exit", self.close, QKeySequence.StandardKey.Quit)
        shortcuts = [("Previous layout", "[", lambda: self.switch(-1)), ("Next layout", "]", lambda: self.switch(1)),
                     ("Pause / resume", "Space", self.toggle_pause), ("Animate demo", "D", lambda: self.demo_button.click()),
                     ("Bent pose", "R", lambda: self.reset(self.sim.layout.bent_pose)), ("Zero pose", "H", lambda: self.reset(np.zeros(6))),
                     ("Cycle frames", "F", lambda: self.cycle_combo(self.frame_combo)),
                     ("Cycle labels", "L", lambda: self.cycle_combo(self.label_combo)),
                     ("Clear selection", "Escape", self.clear_selection)]
        view = self.menuBar().addMenu("View")
        for title, key, callback in shortcuts:
            action = QAction(title, self)
            action.setShortcut(QKeySequence(key))
            action.triggered.connect(callback)
            self.addAction(action)
            view.addAction(action)
        for index in range(6):
            action = QAction(f"Select J{index + 1}", self)
            action.setShortcut(str(index + 1))
            action.triggered.connect(lambda _, i=index: self.select_joint(i, reveal=True))
            self.addAction(action)
        self.menuBar().addMenu("Help").addAction("Controls and shortcuts", self.show_help)

    def native_key(self, event):
        key = event.key()
        mapping = {Qt.Key.Key_BracketLeft: lambda: self.switch(-1), Qt.Key.Key_BracketRight: lambda: self.switch(1),
                   Qt.Key.Key_Space: self.toggle_pause, Qt.Key.Key_D: lambda: self.demo_button.click(),
                   Qt.Key.Key_R: lambda: self.reset(self.sim.layout.bent_pose), Qt.Key.Key_H: lambda: self.reset(np.zeros(6)),
                   Qt.Key.Key_F: lambda: self.cycle_combo(self.frame_combo), Qt.Key.Key_L: lambda: self.cycle_combo(self.label_combo),
                   Qt.Key.Key_Plus: lambda: self.adjust_selected(2), Qt.Key.Key_Equal: lambda: self.adjust_selected(2),
                   Qt.Key.Key_Minus: lambda: self.adjust_selected(-2), Qt.Key.Key_Escape: self.clear_selection}
        if Qt.Key.Key_1 <= key <= Qt.Key.Key_6:
            self.select_joint(key - Qt.Key.Key_1, reveal=True)
        elif key in mapping:
            mapping[key]()

    def set_zoom(self, text):
        scale = int(text.rstrip("%")) / 100
        font = QFont("Segoe UI")
        font.setPointSizeF(10 * scale)
        QApplication.instance().setFont(font)
        self.viewport.ui_scale = scale
        self.viewport.update()

    def refresh_models(self):
        """Keep preset indices stable, then discover additional MJCF files."""
        selected = (self.sim.layout.path.resolve() if self.is_imported_model
                    else self.sim.layout.number)
        known = {layout.path.resolve() for layout in LAYOUTS}
        paths = {path.resolve() for path in MODEL_DIR.glob("*.xml") if path.resolve() not in known}
        paths.update(path for path in self._extra_model_paths if path.is_file())
        with QSignalBlocker(self.layout_combo):
            self.layout_combo.clear()
            for layout in LAYOUTS:
                self.layout_combo.addItem(f"{layout.number:02d}  {layout.title}", layout.number)
                self.layout_combo.setItemData(self.layout_combo.count()-1, layout.description,
                                              Qt.ItemDataRole.ToolTipRole)
            for path in sorted(paths, key=lambda item: item.name.lower()):
                self.layout_combo.addItem(f"File  /  {path.stem}", str(path))
                self.layout_combo.setItemData(self.layout_combo.count()-1, str(path),
                                              Qt.ItemDataRole.ToolTipRole)
            for index in range(self.layout_combo.count()):
                value = self.layout_combo.itemData(index)
                if value == selected or (isinstance(value, str) and Path(value) == selected):
                    self.layout_combo.setCurrentIndex(index)
                    break

    def refresh_model_list(self):
        self.refresh_models()

    def load_model_selection(self, index):
        if index < 0:
            return
        value = self.layout_combo.itemData(index)
        if isinstance(value, int):
            self.load_layout(value)
        elif value:
            self.load_file(value)

    def open_mjcf(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open six-axis MJCF model", str(MODEL_DIR),
                                            "MuJoCo XML (*.xml);;All files (*)")
        if path:
            self.load_file(path)

    def load_file(self, path):
        """Open a standalone file without changing it or inventing preset metadata."""
        try:
            simulation = Simulation.from_path(path)
            self._extra_model_paths.add(Path(path).resolve())
            self.install_simulation(simulation, preserve_pose=False)
            self.refresh_models()
            self.viewport.camera_preset("Home")
            return True
        except Exception:
            if self.smoke_test:
                self.render_failed(traceback.format_exc())
            else:
                QMessageBox.critical(self, "Could not open MJCF", traceback.format_exc())
                self.refresh_models()
            return False

    def install_simulation(self, simulation, preserve_pose=True):
        previous = self.sim
        if preserve_pose:
            at_bent = np.allclose(previous.targets, previous.layout.bent_pose, atol=1e-9) and not previous.demo
            simulation.reset(simulation.layout.bent_pose if at_bent else previous.data.qpos.copy())
            simulation.targets[:] = np.clip(simulation.layout.bent_pose if at_bent else previous.targets,
                                            *simulation.limits.T)
        supports_dynamics = bool(getattr(simulation, "supports_dynamics", True))
        simulation.dynamic = previous.dynamic and supports_dynamics
        simulation.gravity_assist = previous.gravity_assist and supports_dynamics
        simulation.demo = previous.demo if preserve_pose else False
        simulation.demo_time = previous.demo_time if preserve_pose else 0.
        self.sim = simulation
        self.viewport.set_simulation(simulation)
        with QSignalBlocker(self.wrist_combo):
            self.wrist_combo.setCurrentIndex(list(WRIST_PROFILES).index(simulation.wrist_profile))
        self.apply_layout_info()
        self.sync_targets()
        self.select_joint(self.viewport.geometry.selected)
        self.demo_button.setChecked(simulation.demo)
        self.demo_button.setText("Stop animation" if simulation.demo else "Animate demo")
        self.update_animation_timer()
        self.update_readouts()

    def load_layout(self, number, wrist_profile=None):
        wrist_profile = wrist_profile or self.sim.wrist_profile
        if number == self.sim.layout.number and wrist_profile == self.sim.wrist_profile:
            return
        try:
            simulation = Simulation(layout_by_number(number),wrist_profile)
            self.install_simulation(simulation)
            with QSignalBlocker(self.layout_combo):
                self.layout_combo.setCurrentIndex(number - 1)
        except Exception:
            self.render_failed(traceback.format_exc())

    def switch(self, delta):
        self.layout_combo.setCurrentIndex((self.layout_combo.currentIndex() + delta) % self.layout_combo.count())

    def compare_wrist(self):
        keys=list(WRIST_PROFILES)
        self.load_layout(self.sim.layout.number,keys[(keys.index(self.sim.wrist_profile)+1)%len(keys)])

    def apply_layout_info(self):
        for index, control in enumerate(self.joint_controls):
            lower, upper = np.degrees(self.sim.limits[index])
            with QSignalBlocker(control.spin), QSignalBlocker(control.slider):
                control.spin.setRange(float(lower), float(upper))
                control.slider.setRange(round(lower*10), round(upper*10))
        self.update_mode_controls()
        if self.is_imported_model:
            self.apply_imported_info()
            return
        self.description.setText(self.sim.layout.description)
        self.mass_info.setText(f"6 joints   ·   {self.sim.model.body_mass.sum():.2f} kg   ·   {' '.join(self.sim.axis_names)}")
        self.workspace_title.setText(f"{self.sim.layout.number:02d}  /  {self.sim.layout.title}")
        entry = self.manifest[(self.sim.layout.number,self.sim.wrist_profile)]
        relationship='L2 perpendicular to J4 axle (90°)' if self.sim.layout.perpendicular_j4 else 'L2 parallel to J4 axle'
        self.kinematic_note.setText(f"{relationship} · J4: L3 roll · J5: bend · J6: tool roll")
        for index,control in enumerate(self.joint_controls):
            control.select_button.setText(f'J{index+1}  ·  {self.sim.axis_names[index]}')
            if index == 3:
                control.select_button.setText('J4 · L3 roll · Z')
            if index == 5:
                control.select_button.setText('J6 · Tool roll · Z')
            control.spin.setAccessibleName(f'Joint {index+1} {self.sim.axis_names[index]} target angle in degrees')
        lengths = entry['link_lengths_m']
        family = "Short L2 / extended L3" if self.sim.layout.short_l2 else "J4 at end of long L2"
        geometry_note = ("All six joints remain. J4 is closer to J3; L3 extends from J4. Selected wrist axes apply."
                         if self.sim.layout.short_l2 else "Original URDF joint origins retained; the wrist axes follow the selector.")
        geometry_note += (' The J4 joint frame is turned 90 degrees: L2 is perpendicular to the coaxial motor/reducer axle; L3 follows that axle.'
                          if self.sim.layout.perpendicular_j4 else ' L2 and the J4/L3 axle are parallel.')
        rows = [f"<h2>{family}</h2><p>L1: {lengths['L1']*1000:.0f} mm · L2: {lengths['L2']*1000:.0f} mm · L3: {lengths['L3']*1000:.0f} mm.<br>{geometry_note}</p>"
                f"<p>Home motor/reducer CoM offset from YZ plane: <b>{entry['home_hardware_com_x_mm']:+.2f} mm</b><br>"
                f"Largest motor/reducer center offset: {entry['max_home_hardware_center_x_mm']:.1f} mm</p>"
                f"<p>J2 locked-axis inertia at the inspection pose: {entry['joint2_locked_inertia_inspect_kg_m2']:.3f} kg m²</p>"]
        for drive in entry["drives"]:
            rows.append(f"<h3 style='color:#83e5ca'>J{drive['joint']} · {drive['axis']} · {drive['ratio']:g}:1</h3>"
                        f"<b style='color:#5dd9bb'>Motor</b><br>{drive['motor_mount']}<br>"
                        f"<b style='color:#ffbf71'>Gearbox</b><br>{drive['gearbox_mount']}<br>"
                        f"<b>Moving output flange</b><br>{drive['output_mount']}<br>"
                        f"Motor mass {drive['motor_mass_kg']:.2f} kg · Reflected inertia {drive['reflected_armature_kg_m2']:.4g} kg m²"
                        f"<br>One reducer; transfer: {drive['belt_side'].replace('_', ' ')}"
                        f"<br>Load path: {drive['load_path']}"
                        + (f"<br>Nominal belt tension difference at rated torque: {drive['nominal_belt_tension_difference_N']:.0f} N" if drive['belt_side'] != 'none' else ""))
        self.details.setHtml("<h2>Drivetrain mounting</h2>" + "".join(rows) +
            "<hr><h3>Mechanical model</h3><p>All structure uses edges forming rectangular or trapezoidal spatial frames, without face diagonals or shared apexes. Every orange reducer and green motor case has four equal-length attachment edges ending at four distinct mid-height rim ports. These supports belong to the parent link. Grey outputs belong to the moving child; their planar rim surrounds feed the outgoing frame through four distinct connectors.</p>"
            "<p>Any timing belt feeds the reducer input only, with parallel shafts and a 1:1 pulley ratio. No output-side belts or extra reducers. Belt rating, pretension, fastener sizes and bearing loads still require detailed design.</p>"
            "<p>Structure uses finite-diameter aluminium edges (2700 kg/m³). Their volumes determine mass, COM and inertia. Motor/reducer masses remain component estimates. Orange and green are fixed relative to their parent link; grey output disks move with the child.</p>"
            "<p>Hardware envelopes and belts have contact disabled. Reductions are ideal; backlash and flexibility are not simulated.</p>"
            "<p>Dynamics servos obey the URDF output torque bounds. Targets are rate-limited, rather than imposing hard physical velocity limits.</p>"
            "<p>Gravity / bias assist applies external forces outside actuator limits. Leave it off to compare unassisted dynamics.</p>")

    def apply_imported_info(self):
        source = self.sim.layout.path
        metadata = getattr(self.sim, "export_metadata", None)
        if metadata is not None:
            self.apply_optimized_info(source, metadata)
            return
        self.description.setText(f"Imported MJCF: {source.name}")
        self.description.setToolTip(str(source))
        self.mass_info.setText(f"6 joints   |   {self.sim.model.body_mass.sum():.2f} kg   |   {' '.join(self.sim.axis_names)}")
        self.workspace_title.setText(f"Imported  /  {self.sim.layout.title}")
        self.kinematic_note.setText("Pose inspection of the model's own geometry and mass properties.")
        self.wrist_combo.setEnabled(False)
        for index, control in enumerate(self.joint_controls):
            control.select_button.setText(f"J{index+1}  |  {self.sim.axis_names[index]}")
            control.spin.setAccessibleName(f"Joint {index+1} {self.sim.axis_names[index]} target angle in degrees")
        actuator_note = ("This model has no actuators. Joint sliders inspect poses; servo dynamics are unavailable."
                         if self.sim.model.nu == 0 else
                         "This imported model is opened for pose inspection. Its actuators are not controlled by this viewer.")
        self.details.setHtml(
            f"<h2>{html.escape(self.sim.layout.title)}</h2>"
            f"<p><b>Source file</b><br>{html.escape(str(source))}</p>"
            f"<p>Six hinge joints. Total modeled mass: {self.sim.model.body_mass.sum():.3f} kg.</p>"
            f"<p>{actuator_note}</p>"
            "<p>The file's masses, inertias, component placements and geometry are used directly. "
            "Preset drivetrain ratios and rated-torque values are not substituted.</p>"
            "<p>Unlimited joints use a finite slider inspection range; this does not add physical joint stops.</p>")

    def apply_optimized_info(self, source, metadata):
        self.description.setText(self.sim.layout.description)
        self.description.setToolTip(str(source))
        self.mass_info.setText(f"6 joints   |   {self.sim.model.body_mass.sum():.2f} kg   |   {' '.join(self.sim.axis_names)}")
        self.workspace_title.setText(self.sim.layout.title)
        relationship = "L2 perpendicular to J4 axle" if self.sim.layout.perpendicular_j4 else "L2 parallel to J4 axle"
        self.kinematic_note.setText(f"{relationship} | J4: L3 roll | J5: bend | J6: tool roll")
        for index, control in enumerate(self.joint_controls):
            control.select_button.setText(f"J{index+1} | {self.sim.axis_names[index]}")
            control.spin.setAccessibleName(f"Joint {index+1} {self.sim.axis_names[index]} target angle in degrees")
        lengths = metadata["link_lengths_m"]
        rows = [f"<h2>{html.escape(self.sim.layout.title)}</h2>"
                f"<p>L1: {lengths['L1']*1000:.1f} mm | L2: {lengths['L2']*1000:.1f} mm | "
                f"L3: {lengths['L3']*1000:.1f} mm</p>"
                f"<p><b>Source</b><br>{html.escape(str(source))}</p>"]
        for drive in metadata["drives"]:
            from .drivetrain_limits import joint_torque_limit
            joint_cap = joint_torque_limit(drive)
            rows.append(
                f"<h3>J{drive['joint']} | {drive['axis']} | {drive['ratio']:g}:1</h3>"
                f"<b>Motor:</b> {html.escape(drive['motor_id'])}<br>"
                f"<b>Reducer:</b> {html.escape(drive['reducer_id'])}<br>"
                f"<b>Fixed cases:</b> {html.escape(drive['motor_mount'])}<br>"
                f"<b>Moving output:</b> {html.escape(drive['output_mount'])}<br>"
                f"Motor {drive['motor_mass_kg']:.3f} kg | reducer {drive['gearbox_mass_kg']:.3f} kg<br>"
                f"Continuous joint cap: {joint_cap:.3g} N m<br>"
                f"Input drive: {html.escape(drive['drive'].replace('_', ' '))}")
        rows.append("<hr><p>Optimized mass, COM, inertia and hardware placement are preserved. "
                    "Edge frames, balanced case mounts, moving flanges and routes depict the conceptual "
                    "structure without replacing the optimized mass properties.</p>"
                    "<p>Six catalog-limited position servos and reflected rotor inertia are included. "
                    "Dynamics uses the model's own ratios and motor torque-speed screening envelope. "
                    "Joint stops and contact remain disabled as specified for this study.</p>")
        self.details.setHtml("".join(rows))

    def update_mode_controls(self):
        supported = bool(getattr(self.sim, "supports_dynamics", True))
        if not supported:
            self.sim.dynamic = False
            self.sim.gravity_assist = False
        self.dynamics_button.setEnabled(supported)
        self.dynamics_button.setToolTip("Torque-limited servo dynamics" if supported else
                                       "Imported MJCF is available for pose inspection only")
        self.pose_button.setChecked(not self.sim.dynamic)
        self.dynamics_button.setChecked(self.sim.dynamic)
        self.assist.setVisible(self.sim.dynamic and supported)
        self.assist.setEnabled(supported)
        if not supported:
            self.mode_hint.setText("Pose inspection only. No viewer-controlled actuators are available.")
            self.mode_badge.setText("IMPORTED POSE INSPECTION")
        else:
            self.mode_hint.setText("Torque-limited servos under gravity. Readouts show actual angles." if self.sim.dynamic
                                   else "Set joint angles directly. Use Dynamics to test the servos.")
            self.mode_badge.setText("SERVO DYNAMICS" if self.sim.dynamic else "POSE EXPLORATION")
        self.wrist_combo.setEnabled(supported and len(WRIST_PROFILES) > 1)

    def select_joint(self, index, reveal=False, toggle=False):
        if index is None or index < 0 or (toggle and self.viewport.geometry.selected == index):
            index = -1
        self.viewport.geometry.selected = index
        for j, control in enumerate(self.joint_controls):
            control.set_selected(j == index)
        if index < 0:
            self.mount_info.setText("No joint selected. Click a component or a joint heading to inspect it.")
            self.viewport.update()
            return
        if self.is_imported_model:
            model = self.sim.model
            child = model.body(int(model.jnt_bodyid[index]))
            parent = model.body(int(model.body_parentid[child.id]))
            metadata = getattr(self.sim, "export_metadata", None)
            if metadata is not None:
                drive = metadata["drives"][index]
                self.mount_info.setText(f"Selected J{index+1} | {self.sim.axis_names[index]} | {drive['ratio']:g}:1\n"
                                       f"Motor: {drive['motor_id']} | reducer: {drive['reducer_id']}\n"
                                       f"Fixed cases: {drive['motor_mount']}\nMoving output: {drive['output_mount']}")
            else:
                self.mount_info.setText(f"Selected J{index+1} | {self.sim.axis_names[index]} axis\n"
                                        f"Parent: {parent.name or 'world'}\nMoving body: {child.name}")
            if reveal and self.tabs.currentIndex() == 0:
                self.joint_scroll.ensureWidgetVisible(self.joint_controls[index], 0, 16)
            self.viewport.update()
            return
        drive = self.manifest[(self.sim.layout.number,self.sim.wrist_profile)]["drives"][index]
        motion = 'Rolls L3 about its length' if index == 3 else 'Rolls the tool about its axle' if index == 5 else f'{self.sim.axis_names[index]} axis'
        self.mount_info.setText(f"Selected J{index + 1} · {motion} · {drive['ratio']:g}:1\nFixed housing: {drive['gearbox_mount']}\nMoving flange: {drive['output_mount']}")
        if reveal and self.tabs.currentIndex() == 0:
            self.joint_scroll.ensureWidgetVisible(self.joint_controls[index], 0, 16)
        self.viewport.update()

    def clear_selection(self):
        self.select_joint(-1)

    def adjust_selected(self, degrees):
        index = self.viewport.geometry.selected
        if index < 0:
            return
        self.set_target(index, math.degrees(self.sim.targets[index]) + degrees)

    def set_target(self, index, degrees):
        self.sim.demo = False
        self.demo_button.setChecked(False)
        self.demo_button.setText("Animate demo")
        self.sim.set_target(index, math.radians(degrees))
        self.joint_controls[index].set_target(math.degrees(self.sim.targets[index]))
        self.select_joint(index)
        self.refresh_pose()
        self.update_animation_timer()

    def sync_targets(self):
        for index, control in enumerate(self.joint_controls):
            control.set_target(math.degrees(self.sim.targets[index]))

    def refresh_pose(self):
        if not self.sim.dynamic:
            self.sim.advance(0, running=False)
        self.viewport.update()
        self.update_readouts()

    def change_mode(self, dynamic):
        self.sim.dynamic = dynamic and bool(getattr(self.sim, "supports_dynamics", True))
        self.sim.command[:] = self.sim.data.qpos
        self.sim.data.qvel[:] = 0
        self.update_mode_controls()
        self.refresh_pose()
        self.update_animation_timer()

    def set_assist(self, checked):
        self.sim.gravity_assist = checked

    def toggle_pause(self):
        self.running = not self.running
        self.pause_button.setText("Resume" if not self.running else "Pause")
        self.update_animation_timer()
        self.update_readouts()

    def toggle_demo(self):
        self.sim.demo = self.demo_button.isChecked()
        self.demo_button.setText("Stop animation" if self.sim.demo else "Animate demo")
        if self.sim.demo:
            self.running = True
            self.pause_button.setText("Pause")
        self.update_animation_timer()

    def update_animation_timer(self):
        active = self.running and (self.sim.dynamic or self.sim.demo)
        if active and not self.animation_timer.isActive():
            self.last_tick = time.perf_counter()
            self.animation_timer.start()
        elif not active:
            self.animation_timer.stop()

    def advance(self):
        now = time.perf_counter()
        seconds = min(0.05, now - self.last_tick)
        self.last_tick = now
        self.sim.advance(seconds, self.running)
        if self.sim.demo:
            self.sync_targets()
        self.viewport.update()

    def reset(self, pose):
        self.sim.demo = False
        self.demo_button.setChecked(False)
        self.demo_button.setText("Animate demo")
        self.sim.reset(pose)
        self.sync_targets()
        self.refresh_pose()
        self.update_animation_timer()

    def set_overlay(self, name, value):
        setattr(self.viewport.geometry, name, value)
        self.viewport.update()

    def set_quality(self, value):
        self.viewport.quality = value
        self.viewport.update()

    def set_visibility(self, name, checked):
        self.viewport.geometry.visible[name] = checked
        self.viewport.update()

    def update_readouts(self):
        mobility = tool_mobility(self.sim.model, self.sim.data)
        rank = mobility['rank']
        if rank < 6:
            text = f'Singular pose · tool motion: {rank}/6 · try Bent pose'
        elif mobility['near_singular']:
            text = 'Tool motion: 6/6 · near singular · try Bent pose'
        else:
            text = 'Tool motion: 6/6 · full position + orientation control'
        self.mobility_info.setText(text)
        style = 'color: #ffbf71;' if mobility['near_singular'] else 'color: #83e5ca;'
        if self.mobility_info.styleSheet() != style:
            self.mobility_info.setStyleSheet(style)
        for index, control in enumerate(self.joint_controls):
            torque = ("no actuators" if not self.sim.supports_dynamics and self.sim.model.nu == 0 else
                      "pose inspection" if not self.sim.supports_dynamics else
                      f"{self.sim.output_torques[index]:+.2f} N m")
            control.readout.setText(f"Actual {math.degrees(self.sim.data.qpos[index]):+.1f}°   ·   {torque}")
        xyz = self.sim.data.site("tool_frame").xpos
        paint_ms = np.median(self.viewport.paint_times) if self.viewport.paint_times else 0
        active = self.animation_timer.isActive()
        activity = "Playing" if active else "Paused" if not self.running else "Ready · redraw on demand"
        self.status.setText(f"{activity}     Tool X {xyz[0]:+.3f}   Y {xyz[1]:+.3f}   Z {xyz[2]:+.3f} m     Draw {paint_ms:.1f} ms")
        self.status.setToolTip(self.viewport.renderer_name)

    @staticmethod
    def cycle_combo(combo):
        combo.setCurrentIndex((combo.currentIndex() + 1) % combo.count())

    def save_image(self):
        filename = (f"{self.sim.layout.path.stem}.png" if self.is_imported_model
                    else f"{self.sim.layout.number}-base-6dof.png")
        path, _ = QFileDialog.getSaveFileName(self, "Save annotated viewport", filename, "PNG image (*.png)")
        if path:
            self.viewport.capture(path)

    def show_help(self):
        QMessageBox.information(self, "ArmSim controls", "Left-drag: orbit\nRight / middle-drag: pan\nWheel: zoom\nClick a component: select its joint\nClick it again / empty space / Escape: deselect\n\n[ / ]: previous / next layout\n1–6: select joint\nSpace: pause / resume\nD: animate demo\nR: bent pose\nH: zero pose\nF: cycle coordinate frames\nL: cycle labels\nCtrl+S: save viewport image\n\nRoll-bend-roll wrist: J4 Z / J5 X / J6 Z\nDrag the sidebar divider to resize it.\nUse Interface size to enlarge text and controls.\nThe View tab controls frames, labels and render quality.")

    def render_failed(self, error):
        self.exit_code = 1
        self.animation_timer.stop()
        if self.smoke_test:
            print(error, file=sys.stderr, flush=True)
            QApplication.instance().exit(1)
        else:
            QMessageBox.critical(self, "ArmSim rendering error", error)

    def closeEvent(self, event):
        self.animation_timer.stop()
        self.info_timer.stop()
        if not self.smoke_test:
            self.settings.setValue("interface_zoom", int(self.zoom_combo.currentText().rstrip("%")))
            self.settings.setValue("splitter_state", self.splitter.saveState())
        self.viewport.release_gl()
        event.accept()


def configure_qt():
    surface = QSurfaceFormat()
    surface.setVersion(2, 1)
    surface.setProfile(QSurfaceFormat.OpenGLContextProfile.CompatibilityProfile)
    surface.setDepthBufferSize(24)
    surface.setStencilBufferSize(8)
    surface.setSamples(4)
    surface.setSwapInterval(1)
    QSurfaceFormat.setDefaultFormat(surface)


def main():
    parser = argparse.ArgumentParser(description="Qt / OpenGL explorer for six-axis motor and gearbox layouts.")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--model", type=int, choices=[layout.number for layout in LAYOUTS], default=1)
    source.add_argument("--file", type=Path, help="Open a standalone six-axis MJCF, including optimization exports.")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--wrist",choices=list(WRIST_PROFILES),default=DEFAULT_WRIST)
    parser.add_argument("--smoke-test", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--screenshot", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.list:
        for layout in LAYOUTS:
            print(f"{layout.number}: {layout.title} ({layout.filename})")
        known = {layout.path.resolve() for layout in LAYOUTS}
        for path in sorted(MODEL_DIR.glob("*.xml")):
            if path.resolve() not in known:
                print(f"file: {path.name} ({path.resolve()})")
        return 0
    if not args.file and any(not layout.path.exists() for layout in LAYOUTS):
        parser.error("Missing MJCF files. Run: uv run python -m armsim.generate_models")
    configure_qt()
    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor("#101823"))
    palette.setColor(QPalette.ColorRole.Base, QColor("#182332"))
    palette.setColor(QPalette.ColorRole.Text, QColor("#e6edf8"))
    palette.setColor(QPalette.ColorRole.WindowText, QColor("#e6edf8"))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor("#e6edf8"))
    palette.setColor(QPalette.ColorRole.Button, QColor("#26394f"))
    palette.setColor(QPalette.ColorRole.Highlight, QColor("#306d60"))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#eafff8"))
    app.setPalette(palette)
    try:
        window = Explorer(args.model, args.smoke_test, args.wrist, model_path=args.file)
    except Exception as exc:
        parser.error(str(exc))
    if args.smoke_test:
        window.show()
        from armsim.qt_smoke import SmokeTest
        window.smoke = SmokeTest(window, args.screenshot)
        QTimer.singleShot(600, window.smoke.start)
    else:
        window.showMaximized()
    result = app.exec()
    return window.exit_code or result


if __name__ == "__main__":
    raise SystemExit(main())

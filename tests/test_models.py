"""Independent kinematic and dynamic checks for the generated layouts."""

import json
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from armsim.catalog import AXES, BENT_POSE, LAYOUTS, MODEL_DIR, URDF_PATH, WRIST_PROFILES, axes_for
from armsim.simulation import Simulation
from armsim.kinematics import tool_jacobian, tool_mobility


def rotation(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def urdf_poses(robot, angles, layout=None, wrist_profile='reference'):
    """Independent forward kinematics; no MuJoCo or generator constants."""
    poses = {"base_link": np.eye(4)}
    index = 0
    for joint in robot.findall("joint"):
        transform = np.eye(4)
        origin = joint.find("origin").attrib
        transform[:3, 3] = np.fromstring(origin["xyz"], sep=" ")
        if layout is not None and layout.short_l2:
            if joint.attrib['name'] == 'joint_4':
                transform[:3, 3] = [0, 0, layout.l2_length]
            elif joint.attrib['name'] == 'joint_5':
                transform[:3, 3] = [0, 0, layout.l3_length]
        roll, pitch, yaw = np.fromstring(origin["rpy"], sep=" ")
        if layout is not None and layout.perpendicular_j4 and joint.attrib['name']=='joint_4':
            roll,pitch,yaw=-np.pi/2,0.,0.
        transform[:3, :3] = rotation(np.array([0, 0, 1]), yaw) @ rotation(np.array([0, 1, 0]), pitch) @ rotation(np.array([1, 0, 0]), roll)
        if joint.attrib["type"] == "revolute":
            axis=axes_for(wrist_profile)[index]
            transform[:3, :3] = transform[:3, :3] @ rotation(axis, angles[index])
            index += 1
        poses[joint.find("child").attrib["link"]] = poses[joint.find("parent").attrib["link"]] @ transform
    return poses


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = ET.parse(URDF_PATH).getroot()
        cls.joints = [j for j in cls.urdf.findall("joint") if j.attrib["type"] == "revolute"]
        cls.all_metadata = json.loads((MODEL_DIR / "layouts.json").read_text(encoding="utf-8"))
        cls.metadata = [item for item in cls.all_metadata if item["wrist_profile"]=="reference"]

    def test_every_layout_matches_declared_geometry_and_urdf_axes(self):
        generator = np.random.default_rng(42)
        poses = [np.zeros(6), BENT_POSE] + [generator.uniform(-0.8, 0.8, 6) for _ in range(5)]
        for layout in LAYOUTS:
            with self.subTest(layout=layout.title):
                model = mujoco.MjModel.from_xml_path(str(layout.path))
                data = mujoco.MjData(model)
                self.assertEqual((model.nq, model.nv, model.njnt, model.nu), (6, 6, 6, 6))
                np.testing.assert_allclose(model.jnt_axis, AXES)
                for index, joint in enumerate(self.joints):
                    self.assertEqual(model.joint(index).name, joint.attrib["name"])
                    np.testing.assert_allclose(model.jnt_range[index], [float(joint.find("limit").attrib[key]) for key in ("lower", "upper")])
                for pose in poses:
                    data.qpos[:] = pose
                    mujoco.mj_forward(model, data)
                    for name, expected in urdf_poses(self.urdf, pose, layout).items():
                        np.testing.assert_allclose(data.body(name).xpos, expected[:3, 3], atol=2e-10)
                        np.testing.assert_allclose(data.body(name).xmat.reshape(3, 3), expected[:3, :3], atol=2e-10)
                    np.testing.assert_allclose(data.site("tool_frame").xpos, data.body("tool0").xpos)

    def test_short_l2_is_finite_with_extended_l3_and_recomputed_inertia(self):
        for layout in [item for item in LAYOUTS if item.short_l2]:
            with self.subTest(layout=layout.title):
                sim = Simulation(layout)
                model, data = sim.model, sim.data
                np.testing.assert_allclose(model.body('wrist_1_link').pos, [0,0,0.09])
                np.testing.assert_allclose(model.body('wrist_2_link').pos, [0,0,0.27])
                self.assertEqual(layout.drives[3], 'coaxial')
                for name in ('forearm_link','wrist_1_link'):
                    body = model.body(name)
                    self.assertGreater(float(body.mass[0]),0)
                    self.assertTrue(np.all(body.inertia > 0))
                    self.assertTrue(np.isfinite(body.ipos).all())
                jacp, jacr = np.zeros((3,6)), np.zeros((3,6))
                mujoco.mj_jacBody(model, data, jacp, jacr, model.body('tool0').id)
                self.assertEqual(np.linalg.matrix_rank(np.vstack([jacp,jacr]), tol=1e-8), 6)
                old_tool = urdf_poses(self.urdf, BENT_POSE)['tool0'][:3,3]
                # Moving a roll joint along the same L2/L3 centerline changes
                # packaging, but not the tool FK when L2+L3 is conserved.
                np.testing.assert_allclose(data.body('tool0').xpos,old_tool,atol=1e-12)
                old_j4 = urdf_poses(self.urdf,layout.bent_pose)['wrist_1_link'][:3,3]
                self.assertAlmostEqual(np.linalg.norm(data.body('wrist_1_link').xpos-old_j4),0.19)
                sim.reset(np.zeros(6))
                expected=[0,0.42,0.63] if layout.perpendicular_j4 else [0,0,1.05]
                np.testing.assert_allclose(data.body('tool0').xpos,expected,atol=1e-12)

    def test_mounting_mass_and_transmission_parameters(self):
        for layout, entry in zip(LAYOUTS, self.metadata):
            with self.subTest(layout=layout.title):
                model = mujoco.MjModel.from_xml_path(str(layout.path))
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                for index, drive in enumerate(entry["drives"]):
                    j = index + 1
                    motor, gearbox = model.body(f"motor_{j}"), model.body(f"gearbox_{j}")
                    self.assertEqual(model.body(model.body_parentid[motor.id]).name, drive["motor_mount"])
                    self.assertEqual(model.body(model.body_parentid[gearbox.id]).name, drive["gearbox_mount"])
                    self.assertAlmostEqual(float(model.body_mass[motor.id]), drive["motor_mass_kg"])
                    self.assertAlmostEqual(float(model.body_mass[gearbox.id]), drive["gearbox_mass_kg"])
                    self.assertAlmostEqual(model.dof_armature[index], drive["rotor_inertia_kg_m2"] * layout.ratios[index] ** 2)
                    self.assertTrue(np.all(model.body_inertia[motor.id] > 0))
                    self.assertTrue(np.all(model.body_inertia[gearbox.id] > 0))
                    motor_axis = data.site(f"M{j}").xmat.reshape(3, 3)[:, 2]
                    gear_axis = data.site(f"G{j}").xmat.reshape(3, 3)[:, 2]
                    output_axis = data.site(f"O{j}").xmat.reshape(3, 3)[:, 2]
                    np.testing.assert_allclose(output_axis, data.xaxis[index], atol=1e-12)
                    np.testing.assert_allclose(gear_axis, data.xaxis[index], atol=1e-12)
                    np.testing.assert_allclose(motor_axis, data.xaxis[index], atol=1e-12)

    def test_packaging_is_local_parallel_and_near_the_arm_plane(self):
        for layout, entry in zip(LAYOUTS, self.metadata):
            with self.subTest(layout=layout.title):
                model = mujoco.MjModel.from_xml_path(str(layout.path))
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                self.assertTrue(all(ratio > 1 for ratio in layout.ratios))
                self.assertLess(abs(entry['home_hardware_com_x_mm']), 30)
                self.assertLess(entry['max_home_hardware_center_x_mm'], 45)
                self.assertEqual(model.ntendon, 0)
                for index, drive in enumerate(entry['drives']):
                    j=index+1
                    expected_parent = self.joints[index].find('parent').attrib['link']
                    expected_child = self.joints[index].find('child').attrib['link']
                    self.assertEqual(drive['motor_mount'], expected_parent)
                    self.assertEqual(drive['gearbox_mount'], expected_parent)
                    self.assertEqual(drive['output_mount'], expected_child)
                    self.assertFalse(drive['belt_crosses_joint'])
                    # Small actuator/belt offsets must actually clear the rails,
                    # not put an input shaft or pulley through a structural bar.
                    parent=model.body(expected_parent).id
                    frame=[i for i in range(model.ngeom) if model.geom_bodyid[i]==parent
                           and model.geom(i).name.startswith(expected_parent+'_')]
                    hardware=[model.geom(f'M{j}_stator').id,model.geom(f'G{j}_housing').id]
                    hardware += [i for i in range(model.ngeom) if model.geom(i).name.startswith(
                        (f'input_pulley_{j}_',f'input_belt_{j}_span_',f'M{j}_input_stub'))]
                    for first in frame:
                        for second in hardware:
                            self.assertGreater(mujoco.mj_geomDistance(model,data,first,second,1,None),0.001,
                                               (layout.number,j,model.geom(first).name,model.geom(second).name))
                    if drive['drive'] != 'coaxial':
                        centers = np.asarray(drive['belt_centers_m'])
                        axis = np.asarray(drive['belt_axis'])
                        self.assertAlmostEqual(float((centers[1]-centers[0]) @ axis), 0)
                        self.assertGreater(np.linalg.norm(centers[1]-centers[0]), 2*drive['pulley_radius_m'])
                        self.assertGreater(drive['belt_width_m'], 0)
                    output = model.body(f'output_coupling_{index+1}')
                    self.assertEqual(model.body(model.body_parentid[output.id]).name, expected_child)

    def test_servo_units_reduction_and_output_torque_limits(self):
        for layout in LAYOUTS:
            with self.subTest(layout=layout.title):
                sim = Simulation(layout)
                sim.data.ctrl[:] = (layout.bent_pose + 0.01) * sim.ratios
                mujoco.mj_forward(sim.model, sim.data)
                np.testing.assert_allclose(sim.output_torques, np.array([180, 240, 180, 70, 55, 40]) * 0.01, atol=1e-7)
                effort = np.array([float(j.find("limit").attrib["effort"]) for j in self.joints])
                np.testing.assert_allclose(sim.model.actuator_forcerange[:, 1] * sim.ratios, effort)
                np.testing.assert_allclose(sim.model.actuator_ctrlrange, sim.limits * sim.ratios[:, None])
                for sign in (-1, 1):
                    sim.data.ctrl[:] = sign * 1e6
                    mujoco.mj_forward(sim.model, sim.data)
                    self.assertTrue(np.all(np.abs(sim.output_torques) <= effort + 1e-9))

    def test_all_wrist_profiles_and_supported_single_reducer_load_paths(self):
        entries={(item['number'],item['wrist_profile']):item for item in self.all_metadata}
        for layout in LAYOUTS:
            for profile in WRIST_PROFILES:
                with self.subTest(layout=layout.number,wrist=profile):
                    sim=Simulation(layout,profile)
                    model,data=sim.model,sim.data
                    entry=entries[(layout.number,profile)]
                    for name,expected in urdf_poses(self.urdf,layout.bent_pose,layout,profile).items():
                        np.testing.assert_allclose(data.body(name).xpos,expected[:3,3],atol=1e-10)
                    np.testing.assert_allclose(model.jnt_axis,axes_for(profile))
                    self.assertEqual(model.nv,6)
                    self.assertEqual(entry['tool_rank_inspect'],6)
                    self.assertAlmostEqual(float(model.jnt_axis[2] @ model.jnt_axis[3]),0)
                    self.assertLess(model.ngeom,400)
                    self.assertEqual(sum(model.body(i).name.startswith('gearbox_') for i in range(model.nbody)),6)
                    for drive in entry['drives']:
                        j=drive['joint']
                        parent=model.body(drive['gearbox_mount']).id
                        child=model.body(drive['output_mount']).id
                        self.assertEqual(drive['reducer_count'],1)
                        self.assertIn(drive['belt_side'],('none','before_reducer'))
                        delta=np.asarray(drive['gearbox_center_m'])-drive['joint_center_m']
                        np.testing.assert_allclose(np.cross(delta,drive['shaft_axis']),0,atol=1e-12)
                        for name in drive['fixed_support_geoms']+drive['motor_mount_geoms']:
                            self.assertEqual(model.geom_bodyid[model.geom(name).id],parent)
                        for name in drive['moving_support_geoms']:
                            self.assertEqual(model.geom_bodyid[model.geom(name).id],child)
                        flange=model.body(f'output_coupling_{j}')
                        self.assertEqual(model.body_parentid[flange.id],child)
                        # The integrated bearing separates a fixed housing from
                        # a moving face flange; no long shaft/cage is modeled.
                        flange_geom=model.geom(f'O{j}_moving_flange').id
                        gear_geom=model.geom(f'G{j}_housing').id
                        distance=mujoco.mj_geomDistance(model,data,flange_geom,gear_geom,1,None)
                        self.assertGreater(distance,0.007)
                    sim.dynamic=True
                    for _ in range(20):sim.advance(0.02)
                    self.assertTrue(np.isfinite(data.qpos).all())
                    self.assertFalse(any(item.number for item in data.warning))

    def test_structural_mass_and_com_follow_actual_frame_geometry(self):
        for layout in LAYOUTS:
            root=ET.parse(layout.path).getroot()
            model=mujoco.MjModel.from_xml_path(str(layout.path))
            for name in ('base_link','shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','tool_flange_link'):
                body=next(item for item in root.iter('body') if item.attrib.get('name')==name)
                geoms=body.findall('geom')
                masses=np.array([float(geom.attrib['mass']) for geom in geoms])
                centers=[]
                for geom in geoms:
                    if 'fromto' in geom.attrib:
                        points=np.fromstring(geom.attrib['fromto'],sep=' ').reshape(2,3)
                        centers.append(points.mean(axis=0))
                    else:
                        centers.append(np.fromstring(geom.attrib.get('pos','0 0 0'),sep=' '))
                np.testing.assert_allclose(model.body(name).ipos,np.average(centers,axis=0,weights=masses),atol=1e-10)
                self.assertAlmostEqual(float(model.body(name).mass[0]),masses.sum())

    def test_moving_frames_clear_their_own_housings_in_sampled_sweep(self):
        entries={(item['number'],item['wrist_profile']):item for item in self.all_metadata}
        for layout in LAYOUTS:
            for profile in WRIST_PROFILES:
                sim=Simulation(layout,profile)
                model,data=sim.model,sim.data
                for drive in entries[(layout.number,profile)]['drives']:
                    j=drive['joint']
                    moving=[model.geom(name).id for name in drive['moving_support_geoms']]
                    child=model.body(drive['output_mount']).id
                    moving += [i for i in range(model.ngeom) if model.geom_bodyid[i]==child
                               and model.geom(i).name.startswith(drive['output_mount']+'_')]
                    fixed=[model.geom(f'{prefix}{j}_{part}').id
                           for prefix,part in (('G','housing'),('G','bearing_housing'),('M','stator'))]
                    for angle in np.linspace(-1,1,9):
                        data.qpos[:]=0
                        data.qpos[j-1]=angle
                        mujoco.mj_forward(model,data)
                        for first in moving:
                            for second in fixed:
                                self.assertGreater(mujoco.mj_geomDistance(model,data,first,second,1,None),0.001,
                                                   (layout.number,profile,j,float(angle),model.geom(first).name,model.geom(second).name))

    def test_exported_wrist_urdfs_keep_the_actual_axis_alternatives(self):
        for profile in WRIST_PROFILES:
            path=URDF_PATH if profile=='reference' else MODEL_DIR/f'base-6dof-{profile}.urdf'
            model=mujoco.MjModel.from_xml_path(str(path))
            np.testing.assert_allclose(model.jnt_axis,axes_for(profile))

    def test_six_independent_tool_motions_and_independent_fk_derivatives(self):
        samples=np.random.default_rng(20261031).uniform(-0.9,0.9,(64,6))
        for layout in LAYOUTS:
            for profile in WRIST_PROFILES:
                with self.subTest(layout=layout.number,wrist=profile):
                    sim=Simulation(layout,profile)
                    for pose in [BENT_POSE,*samples]:
                        sim.reset(pose)
                        self.assertEqual(tool_mobility(sim.model,sim.data)['rank'],6)
                    # Differentiate the independent URDF forward kinematics,
                    # including rotation, rather than just counting six hinges.
                    for pose in [BENT_POSE,*samples[:2]]:
                        sim.reset(pose)
                        expected=np.empty((6,6))
                        center=urdf_poses(self.urdf,pose,layout,profile)['tool0']
                        for j in range(6):
                            delta=np.eye(6)[j]*1e-6
                            plus=urdf_poses(self.urdf,pose+delta,layout,profile)['tool0']
                            minus=urdf_poses(self.urdf,pose-delta,layout,profile)['tool0']
                            expected[:3,j]=(plus[:3,3]-minus[:3,3])/2e-6
                            skew=(plus[:3,:3]-minus[:3,:3])/2e-6 @ center[:3,:3].T
                            expected[3:,j]=[skew[2,1],skew[0,2],skew[1,0]]
                        np.testing.assert_allclose(tool_jacobian(sim.model,sim.data),expected,atol=2e-9)
                        self.assertEqual(np.linalg.matrix_rank(expected,tol=1e-7),6)
                    sim.reset(np.zeros(6))
                    mobility=tool_mobility(sim.model,sim.data)
                    self.assertEqual(mobility['rank'],5 if layout.perpendicular_j4 else 3)
                    self.assertTrue(mobility['near_singular'])

    def test_j4_rolls_l3_without_swinging_its_centerline(self):
        for layout in LAYOUTS:
            for profile in WRIST_PROFILES:
                with self.subTest(layout=layout.number,wrist=profile):
                    sim=Simulation(layout,profile)
                    pose=BENT_POSE.copy()
                    pose[3]=0
                    sim.reset(pose)
                    model,data=sim.model,sim.data
                    origin=data.body('wrist_1_link').xpos.copy()
                    tip=data.body('wrist_2_link').xpos.copy()
                    direction=(tip-origin)/layout.l3_length
                    np.testing.assert_allclose(data.xaxis[3],direction,atol=1e-12)
                    self.assertAlmostEqual(float(data.xaxis[2]@data.xaxis[3]),0)
                    rotation0=data.body('wrist_1_link').xmat.reshape(3,3).copy()
                    for angle in np.linspace(-np.pi,np.pi,13):
                        pose[3]=angle
                        sim.reset(pose)
                        np.testing.assert_allclose(data.body('wrist_1_link').xpos,origin,atol=1e-12)
                        np.testing.assert_allclose(data.body('wrist_2_link').xpos,tip,atol=1e-12)
                        rotation_now=data.body('wrist_1_link').xmat.reshape(3,3)
                        center=origin+rotation_now@np.array([0.,0.,layout.l3_length/2])
                        np.testing.assert_allclose(center,(origin+tip)/2,atol=1e-12)
                        # A material point off the centerline must spin around
                        # it, even though both link endpoints stay in place.
                        marker=origin+rotation_now@np.array([0.033,0.,layout.l3_length/2])
                        expected=center+rotation(direction,angle)@rotation0[:,0]*0.033
                        np.testing.assert_allclose(marker,expected,atol=1e-12)

    def test_relocating_motors_changes_real_joint_inertia(self):
        matrices = []
        for layout in (LAYOUTS[0], LAYOUTS[2]):
            sim = Simulation(layout)
            matrix = np.empty((6, 6))
            mujoco.mj_fullM(sim.model, sim.data, matrix)
            self.assertGreater(np.linalg.eigvalsh(matrix).min(), 0)
            matrices.append(matrix)
        self.assertLess(matrices[1][1, 1], matrices[0][1, 1])

    def test_j6_rolls_tool_without_moving_its_tip(self):
        for layout in LAYOUTS:
            sim=Simulation(layout)
            pose=BENT_POSE.copy()
            pose[5]=0
            sim.reset(pose)
            origin=sim.data.body('tool_flange_link').xpos.copy()
            tip=sim.data.site('tool_frame').xpos.copy()
            rotation0=sim.data.body('tool_flange_link').xmat.reshape(3,3).copy()
            direction=(tip-origin)/0.08
            np.testing.assert_allclose(sim.data.xaxis[5],direction,atol=1e-12)
            for angle in np.linspace(-np.pi,np.pi,13):
                pose[5]=angle
                sim.reset(pose)
                np.testing.assert_allclose(sim.data.site('tool_frame').xpos,tip,atol=1e-12)
                np.testing.assert_allclose(sim.data.body('tool_flange_link').xpos,origin,atol=1e-12)
                r=sim.data.body('tool_flange_link').xmat.reshape(3,3)
                np.testing.assert_allclose(r,rotation(direction,angle)@rotation0,atol=1e-12)

    def test_paired_layouts_differ_only_in_j2_drive(self):
        entries={entry['number']:entry for entry in self.metadata}
        for first,second in ((2,3),(5,6),(8,9)):
            self.assertEqual(LAYOUTS[first-1].drives[1],'input_belt')
            self.assertEqual(LAYOUTS[second-1].drives[1],'coaxial')
            for index in (0,2,3,4,5):
                self.assertEqual(entries[first]['drives'][index],entries[second]['drives'][index])
            # The complete geometry and zero-pose placement outside J2's motor
            # and input transmission must also agree, including brackets.
            first_sim,second_sim=Simulation(LAYOUTS[first-1]),Simulation(LAYOUTS[second-1])
            for sim in (first_sim,second_sim):sim.reset(np.zeros(6))
            for index in range(first_sim.model.ngeom):
                geom=first_sim.model.geom(index)
                if geom.name.startswith(('M2_','input_pulley_2_','input_belt_2_','G2_input_stub')):continue
                other=second_sim.model.geom(geom.name)
                np.testing.assert_allclose(first_sim.data.geom_xpos[geom.id],second_sim.data.geom_xpos[other.id],atol=1e-12)
                np.testing.assert_allclose(first_sim.data.geom_xmat[geom.id],second_sim.data.geom_xmat[other.id],atol=1e-12)
                np.testing.assert_allclose(geom.size,other.size,atol=1e-12)
        for entry in self.metadata:
            for index in (3,5):self.assertEqual(entry['drives'][index]['drive'],'coaxial')

    def test_every_structure_is_spatial_and_uses_only_thin_edges(self):
        for layout in LAYOUTS:
            root=ET.parse(layout.path).getroot()
            self.assertFalse(root.find('asset').findall('mesh'))
            geoms={geom.get('name'):geom for geom in root.iter('geom')}
            for geom in geoms.values():
                if geom.get('group') not in ('0','4'):continue
                self.assertEqual(geom.get('type'),'cylinder')
                self.assertIn('fromto',geom.attrib)
                self.assertLessEqual(float(geom.get('size')),0.002)
            entry=self.metadata[layout.number-1]
            for drive in entry['drives']:
                groups=[drive['fixed_support_geoms'],drive['motor_mount_geoms'],drive['moving_support_geoms']]
                for names in groups:
                    points=np.vstack([np.fromstring(geoms[name].get('fromto'),sep=' ').reshape(2,3) for name in names])
                    self.assertEqual(np.linalg.matrix_rank(points-points.mean(axis=0),tol=1e-10),3)
            for body in root.iter('body'):
                frame=[geom for geom in body.findall('geom') if geom.get('name','').startswith(body.get('name','')+'_frame_')]
                if not frame:continue
                points=np.vstack([np.fromstring(geom.get('fromto'),sep=' ').reshape(2,3) for geom in frame])
                self.assertEqual(np.linalg.matrix_rank(points-points.mean(axis=0),tol=1e-10),3)

    def test_each_green_and_orange_case_has_a_mid_height_edge_port(self):
        for layout,entry in zip(LAYOUTS,self.metadata):
            root=ET.parse(layout.path).getroot()
            geoms={geom.get('name'):geom for geom in root.iter('geom')}
            sim=Simulation(layout)
            for drive in entry['drives']:
                j=drive['joint']
                axis=np.array(drive['shaft_axis'])
                for prefix,center_key,contact_key,names in (
                        ('M','motor_center_m','motor_contact_m',drive['motor_mount_geoms']),
                        ('G','gearbox_center_m','gearbox_contact_m',drive['fixed_support_geoms'])):
                    center=np.array(drive[center_key]);contact=np.array(drive[contact_key])
                    delta=contact-center
                    radius=sim.model.geom(f'{prefix}{j}_stator' if prefix=='M' else f'G{j}_housing').size[0]
                    self.assertAlmostEqual(float(delta@axis),0)
                    self.assertAlmostEqual(np.linalg.norm(delta),radius)
                    endpoints=np.vstack([np.fromstring(geoms[name].get('fromto'),sep=' ').reshape(2,3) for name in names])
                    self.assertLess(np.linalg.norm(endpoints-contact,axis=1).min(),1e-11)
                    for name in names:
                        self.assertEqual(sim.model.geom_bodyid[sim.model.geom(name).id],sim.model.body(drive['motor_mount']).id)

    def test_grey_outputs_have_planar_rim_contacts_and_spatial_child_cages(self):
        for layout,entry in zip(LAYOUTS,self.metadata):
            root=ET.parse(layout.path).getroot()
            geoms={geom.get('name'):geom for geom in root.iter('geom')}
            sim=Simulation(layout)
            sim.reset(np.zeros(6))
            model,data=sim.model,sim.data
            for drive in entry['drives']:
                j=drive['joint'];child=model.body(drive['output_mount']).id
                center=model.body(f'output_coupling_{j}').pos
                axis=np.array(drive['output_axis_child'])
                radius=model.geom(f'O{j}_moving_flange').size[0]
                contacts=np.array(drive['output_contact_points_m'])
                np.testing.assert_allclose((contacts-center)@axis,0,atol=1e-12)
                np.testing.assert_allclose(np.linalg.norm(contacts-center,axis=1),radius,atol=1e-12)
                endpoints=np.vstack([np.fromstring(geoms[name].get('fromto'),sep=' ').reshape(2,3) for name in drive['moving_support_geoms']])
                for contact in contacts:self.assertLess(np.linalg.norm(endpoints-contact,axis=1).min(),1e-11)
                for name in drive['moving_support_geoms']:
                    self.assertEqual(model.geom_bodyid[model.geom(name).id],child)
                parent=model.body(drive['gearbox_mount']).id
                fixed=[i for i in range(model.ngeom) if model.geom_bodyid[i]==parent and model.geom_group[i] in (0,4)]
                for index in fixed:
                    distance=mujoco.mj_geomDistance(model,data,index,model.geom(f'O{j}_moving_flange').id,1,None)
                    self.assertGreater(distance,0.0005,(layout.number,j,model.geom(index).name,distance))
            # Grey disks are moving outputs; fixed bearing exteriors are orange.
            for index in range(model.ngeom):
                if model.geom_matid[index]!=model.material('shaft_metal').id:continue
                body=model.body(model.geom_bodyid[index]).name
                self.assertTrue(body.startswith('output_coupling_') or body=='tool_flange_link')

    def test_07_to_09_put_l2_perpendicular_to_the_actual_j4_axle(self):
        for layout in LAYOUTS[6:9]:
            sim=Simulation(layout)
            for q4 in (-1.,0.,1.):
                pose=BENT_POSE.copy();pose[3]=q4
                sim.reset(pose)
                data=sim.data
                l2=data.xanchor[3]-data.xanchor[2]
                l2/=np.linalg.norm(l2)
                l3=data.xanchor[4]-data.xanchor[3]
                l3/=np.linalg.norm(l3)
                self.assertAlmostEqual(float(l2@data.xaxis[3]),0)
                np.testing.assert_allclose(l3,data.xaxis[3],atol=1e-12)
                for site in ('M4','G4','O4'):
                    np.testing.assert_allclose(data.site(site).xmat.reshape(3,3)[:,2],data.xaxis[3],atol=1e-12)
        urdf=mujoco.MjModel.from_xml_path(str(MODEL_DIR/'base-6dof-perpendicular-j4.urdf'))
        data=mujoco.MjData(urdf);mujoco.mj_forward(urdf,data)
        np.testing.assert_allclose(data.xaxis[3],[0,1,0],atol=1e-11)

    def test_unassisted_servo_motion_is_finite_and_torque_limited(self):
        for layout in LAYOUTS:
            with self.subTest(layout=layout.title):
                sim = Simulation(layout)
                sim.dynamic = True
                sim.targets[:] += [0.2, 0.1, -0.1, 0.2, -0.1, 0.2]
                effort = sim.model.actuator_forcerange[:, 1] * sim.ratios
                previous_command = sim.command.copy()
                for _ in range(200):
                    sim.advance(0.02)
                    self.assertTrue(np.isfinite(sim.data.qpos).all())
                    self.assertTrue(np.isfinite(sim.data.qvel).all())
                    self.assertTrue(np.all(np.abs(sim.output_torques) <= effort + 1e-8))
                    self.assertTrue(np.all(np.abs(sim.command - previous_command) <= sim.speed_limits * 0.02 + 1e-9))
                    previous_command[:] = sim.command
                self.assertFalse(any(w.number for w in sim.data.warning))
                self.assertLess(np.max(np.abs(sim.data.qpos - sim.targets)), 0.35)

    def test_default_chevron_poses_match_tool_position_and_orientation(self):
        reference=Simulation(LAYOUTS[0])
        expected=reference.data.site('tool_frame')
        horizontal=np.array([np.sin(BENT_POSE[0]),-np.cos(BENT_POSE[0]),0.])
        for layout in LAYOUTS:
            sim=Simulation(layout)
            np.testing.assert_allclose(sim.data.qpos,layout.bent_pose,atol=1e-12)
            np.testing.assert_allclose(sim.model.key('inspect').qpos,layout.bent_pose,atol=1e-12)
            np.testing.assert_allclose(sim.data.site('tool_frame').xpos,expected.xpos,atol=1e-11)
            np.testing.assert_allclose(sim.data.site('tool_frame').xmat,expected.xmat,atol=1e-11)
            shoulder=sim.data.xanchor[1]
            tool=sim.data.site('tool_frame').xpos
            apex=max((sim.data.xanchor[2],sim.data.xanchor[3]),key=lambda point:point@horizontal)
            self.assertGreater(float((apex-shoulder)@horizontal),0.15)
            self.assertGreater(float((apex-tool)@horizontal),0.15)
            self.assertGreater(apex[2],shoulder[2])
            self.assertLess(apex[2],tool[2])

    def test_four_equal_case_stays_and_no_rectangular_face_diagonals(self):
        for layout in LAYOUTS:
            root=ET.parse(layout.path).getroot()
            geoms={geom.get('name'):geom for geom in root.iter('geom')}
            self.assertFalse(any('_brace_' in name for name in geoms))
            for j in range(1,7):
                for prefix in (f'J{j}_fixed_mount',f'M{j}_case_mount'):
                    segments=np.array([np.fromstring(geoms[f'{prefix}_case_contact_{i}'].get('fromto'),sep=' ').reshape(2,3) for i in range(4)])
                    lengths=np.linalg.norm(segments[:,1]-segments[:,0],axis=1)
                    self.assertLess(np.ptp(lengths),1e-10)
                    self.assertEqual(len(np.unique(np.round(segments[:,0],12),axis=0)),4)
                    self.assertEqual(len(np.unique(np.round(segments[:,1],12),axis=0)),4)
                bridges=np.array([np.fromstring(geoms[f'J{j}_output_frame_to_link_{i}'].get('fromto'),sep=' ').reshape(2,3) for i in range(4)])
                self.assertEqual(len(np.unique(np.round(bridges[:,0],12),axis=0)),4)
                self.assertLess(np.ptp(np.linalg.norm(bridges[:,1]-bridges[:,0],axis=1)),1e-10)
            for body in root.iter('body'):
                frame=[geom for geom in body.findall('geom') if geom.get('name','').startswith(body.get('name','')+'_frame_')]
                if not frame:continue
                self.assertEqual(len(frame),12)
                for geom in frame:
                    endpoints=np.fromstring(geom.get('fromto'),sep=' ').reshape(2,3)
                    self.assertEqual(np.count_nonzero(abs(endpoints[1]-endpoints[0])>1e-10),1)

    def test_tool_frame_is_compact_and_input_belts_are_inboard(self):
        for layout,entry in zip(LAYOUTS,self.metadata):
            root=ET.parse(layout.path).getroot()
            frame=[g for g in root.iter('geom') if g.get('name','').startswith('tool_flange_link_frame_')]
            points=np.vstack([np.fromstring(g.get('fromto'),sep=' ').reshape(2,3) for g in frame])
            np.testing.assert_allclose(np.ptp(points[:,:2],axis=0),[0.048,0.048],atol=1e-12)
            for drive in entry['drives']:
                if drive['drive']!='input_belt':continue
                centers=np.array(drive['belt_centers_m'])
                axis=np.array(drive['shaft_axis'])
                self.assertLess(abs(float((centers[1]-drive['joint_center_m'])@axis)),0.050)


if __name__ == "__main__":
    unittest.main()

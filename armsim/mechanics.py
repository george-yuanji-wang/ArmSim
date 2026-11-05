"""Edge-only spatial structures with explicit fixed-case and moving-output ports."""

import math
import numpy as np

from armsim.catalog import LINKS, PARENTS
from armsim.model_primitives import add, values, cylinder
from armsim.brackets import edge, box_frame, case_mount, output_frame
from armsim.mechanism_parameters import GeometryError, MechanismParameters


def volume(geom):
    size=np.fromstring(geom.attrib['size'],sep=' ')
    if geom.attrib['type']=='box':return 8*np.prod(size)
    ends=np.fromstring(geom.attrib.get('fromto',''),sep=' ')
    length=np.linalg.norm(ends[3:]-ends[:3]) if len(ends) else 2*size[1]
    return math.pi*size[0]**2*length


def _template_cases(layout, axes, parameters):
    """Place catalog envelopes using the existing fixed/coaxial/offset rules."""
    offsets = layout.joint_offsets.copy()
    offsets[2] = parameters.l1_length
    cases = []
    u2, v2, s3, s5 = parameters.motor_offsets
    for index, drive in enumerate(layout.drives):
        joint = np.array([0., 0., offsets[index]])
        axis = axes[index]
        shift = .018 if index == 3 else .024 if index == 5 else 0.
        housing = parameters.gear_lengths[index]*22/30
        bearing = parameters.gear_lengths[index]*8/30
        output_plane = joint+axis*shift
        gear = output_plane-axis*(housing/2+bearing)
        motor = output_plane-axis*(parameters.gear_lengths[index]+.002+
                                    parameters.motor_lengths[index]/2)
        if drive == 'input_belt':
            if index == 1:
                displacement = np.array([0., .052+u2, -.057+v2])
                clearance = .006
            elif index == 2:
                displacement = np.array([0., .020, -.145*parameters.l1_length/.32+s3])
                clearance = .005
            elif index == 4:
                displacement = np.array([0., .020, -.130*layout.l3_length/.27+s5])
                clearance = .006
            else:
                raise GeometryError('Only J2, J3 and J5 have approved offset-motor mounts')
            motor = joint+displacement-axis*(parameters.motor_lengths[index]/2+clearance)
            if index == 1 and motor[1] <= .004:
                raise GeometryError('J2 motor left the approved side of its mounting plane')
        elif drive != 'coaxial':
            raise GeometryError(f'Unknown approved drive arrangement: {drive}')
        cases.append((joint, gear, motor, housing, bearing))
    return cases


def _check_mount(name, center, radius, frame, vertices):
    """Reject unsupported placements; never invent an alternate bracket."""
    if not np.isfinite(vertices).all():
        raise GeometryError(f'Nonfinite template mount: {name}')
    if np.linalg.matrix_rank(vertices-vertices.mean(axis=0), tol=1e-10) != 3:
        raise GeometryError(f'Template mount is not spatial: {name}')
    lower = np.array([-frame['width'], -frame['depth'], frame['low']])
    upper = np.array([frame['width'], frame['depth'], frame['high']])
    outside = np.maximum(np.maximum(lower-center, center-upper), 0.)
    if np.linalg.norm(outside) > 2*radius+.016:
        raise GeometryError(f'Case is outside the local template mount envelope: {name}')


def _frame_case_clearance(body, frame, center, axis, radius, length, name):
    """Check local frame stock against its own case envelope at assembly zero.

    This is a mounting-fit check, not an operating-range self-collision test.
    Intended case-port stays are excluded; only the original twelve cage edges
    are considered.  The expanded finite cylinder conservatively includes the
    edge's stock radius.
    """
    center, axis = np.asarray(center), np.asarray(axis)
    geoms = {geom.get('name'): geom for geom in body.findall('geom')}
    for geom_name in frame['geoms']:
        geom = geoms[geom_name]
        ends = np.fromstring(geom.get('fromto'), sep=' ').reshape(2, 3)-center
        edge_radius = float(geom.get('size'))
        a, delta = ends[0], ends[1]-ends[0]
        axial, slope = float(a@axis), float(delta@axis)
        half = length/2+edge_radius
        low, high = 0., 1.
        if abs(slope) < 1e-12:
            if abs(axial) >= half:
                continue
        else:
            first, second = sorted(((-half-axial)/slope, (half-axial)/slope))
            low, high = max(low, first), min(high, second)
            if high <= low:
                continue
        radial = a-axis*axial
        tangent = delta-axis*slope
        square = float(tangent@tangent)
        t = low if square < 1e-20 else float(np.clip(-(radial@tangent)/square, low, high))
        if np.linalg.norm(radial+t*tangent) < radius+edge_radius-1e-10:
            raise GeometryError(f'{name} intersects its parent template cage ({geom_name})')


def build_mechanics(layout,bodies,axes,parameters=None):
    parent_axes=axes.copy()
    parent_axes[3]=layout.j4_rotation@axes[3]
    lengths=(0.10,0.12,0.32,layout.l2_length,layout.l3_length,layout.wrist2_length,0.08)
    motor_masses=(0.90,0.90,0.65,0.22,0.18,0.15)
    gear_masses=(0.30,0.35,0.28,0.12,0.10,0.08)
    radii=(0.037,0.035,0.033,0.026,0.024,0.022)
    motor_radii = radii
    gear_radii = tuple(radius+.002 for radius in radii)
    motor_lengths = (.020, .020, .020, .018, .018, .018)
    cases = None
    if parameters is not None:
        if not isinstance(parameters, MechanismParameters):
            raise TypeError('parameters must be MechanismParameters')
        lengths = (.10, .12, parameters.l1_length, layout.l2_length, layout.l3_length, layout.wrist2_length, .08)
        motor_masses, gear_masses = parameters.motor_masses, parameters.gear_masses
        motor_radii, gear_radii = parameters.motor_radii, parameters.gear_radii
        motor_lengths = parameters.motor_lengths
        radii = tuple(max(motor, gear-.002) for motor, gear in zip(motor_radii, gear_radii))
        cases = _template_cases(layout, parent_axes, parameters)
    z=np.array([0.,0.,1.])
    frames={}
    for index,(name,length) in enumerate(zip(LINKS,lengths)):
        incoming=radii[index-1] if index else 0.037
        outgoing=radii[min(index,5)]
        low=min(incoming+0.016,length*0.45)
        high=length-min(outgoing+0.016,length*0.40)
        if index<6 and abs(parent_axes[index,2])>0.9:
            shift=0.018 if index==3 else 0.024 if index==5 else 0.
            motor_length=0.020 if index<3 else 0.018
            if parameters is None:
                high=min(high,length+shift-0.032-motor_length-0.0048)
            else:
                high=min(high,length+shift-parameters.gear_lengths[index]-.002-
                         motor_lengths[index]-.0048)
        if index==0:low,high=0.030,0.040
        if index==6:high=0.078
        if high<=low:raise GeometryError(f'No room for spatial link frame: {name}')
        width=0.060 if index==0 else 0.024 if index==6 else 0.041 if index<4 else 0.033
        depth=0.030 if index==0 else 0.024 if index==6 else 0.014
        if parameters is not None and index < 6 and layout.drives[index] == 'input_belt':
            # The existing side rails must surround a complete framed motor,
            # whose rear extends beyond the original thin nominal motor case.
            # Keep the same twelve cage edges and four stays.
            clearance = .005 if index == 2 else .006
            width = max(width, motor_lengths[index]+clearance+.0048)
        if parameters is not None and index < 6:
            joint, gear_center, motor_center, housing_length, bearing_length = cases[index]
            axis = parent_axes[index]
            if abs(axis[0]) > .9:
                depth = min(depth, gear_radii[index]/1.6,
                            motor_radii[index]/1.6 if layout.drives[index] == 'coaxial' else depth)
                if layout.drives[index] == 'input_belt':
                    width = max(width, abs(motor_center[0])+.008)
            if abs(axis[2]) < .9:
                high = min(high, joint[2]-gear_radii[index]-.0018)
                if layout.drives[index] == 'coaxial':
                    high = min(high, motor_center[2]-motor_radii[index]-.0018)
            if abs(axis[2]) > .9:
                for center, radius, span in ((motor_center, motor_radii[index], motor_lengths[index]),
                                              (gear_center, gear_radii[index], parameters.gear_lengths[index])):
                    if center[2]-span/2 < high and center[2]+span/2 > low:
                        width, depth = max(width, radius+.005), max(depth, radius+.005)
            width_limit = .120 if index == 0 else .082
            if layout.drives[index] == 'input_belt':
                # Axial motor length sets the minimum side-rail clearance.
                # The old fixed 82 mm limit rejected a 74 mm body needing
                # 84.8 mm, even though no physical contact was present.
                # Permit that derived width, with a finite 110 mm envelope;
                # depth, stock-span and all local contact checks still apply.
                clearance = .005 if index == 2 else .006
                width_limit = max(width_limit, min(.110, motor_lengths[index]+clearance+.0048))
            if width > width_limit+1e-12 or depth > (.060 if index == 0 else .028):
                raise GeometryError(f'Catalog envelope exceeds approved frame cross-section: {name}')
            if depth < .006 or high-low < .0072:
                raise GeometryError(f'Insufficient spatial frame span for catalog hardware: {name}')
        frames[name]=box_frame(bodies[name],name+'_frame',low,high,width,depth,
                               'base_gray' if index==0 else 'arm_blue')
    base_foot=box_frame(bodies['base_link'],'base_foot',0.0018,0.024,0.085,0.075,'base_gray')
    for index,(start,end) in enumerate(zip(base_foot['corners'][4:],frames['base_link']['corners'][:4])):
        edge(bodies['base_link'],f'base_riser_{index}',start,end,'base_gray')

    manifest=[]
    for index,drive in enumerate(layout.drives):
        j=index+1
        local_axis,axis=axes[index],parent_axes[index]
        parent_name,child_name=PARENTS[index],LINKS[index+1]
        parent,child=bodies[parent_name],bodies[child_name]
        joint=np.array([0.,0.,layout.joint_offsets[index]])
        r=radii[index]
        shift=0.018 if j==4 else 0.024 if j==6 else 0.
        motor_length=0.020 if j<=3 else 0.018
        gear_center=joint+axis*(shift-0.019)
        motor_center=joint+axis*(shift-0.032-motor_length/2)
        if drive=='input_belt':
            if j in (4,6):raise ValueError('Roll motors must remain coaxial')
            displacement=np.array([0.,0.052,-0.057]) if j==2 else np.array([0.,0.020,-{3:0.145,5:0.130}[j]])
            # Keep clearance to the compact moving rim at the lower J2 motor.
            motor_center=joint+displacement-axis*(0.016 if j==2 else 0.015)
        elif drive!='coaxial':raise ValueError('Unknown drive arrangement')

        housing_length, bearing_length, bearing_offset = .022, .008, .015
        if cases is not None:
            joint, gear_center, motor_center, housing_length, bearing_length = cases[index]
            motor_length = motor_lengths[index]
            bearing_offset = (housing_length+bearing_length)/2

        gear=add(parent,'body',name=f'gearbox_{j}',pos=values(gear_center),zaxis=values(axis))
        cylinder(gear,f'G{j}_housing',[0,0,0],z,gear_radii[index],housing_length,gear_masses[index],'gearbox_amber',2)
        # Fixed bearing exterior uses the fixed housing colour, not output grey.
        gear_core = r if parameters is None else max(gear_radii[index]-.002, gear_radii[index]*.5)
        cylinder(gear,f'G{j}_bearing_housing',[0,0,bearing_offset],z,gear_core*0.82,bearing_length,0,'gearbox_amber',2)
        add(gear,'site',name=f'G{j}',size='0.003',rgba='1 1 0 0')
        motor=add(parent,'body',name=f'motor_{j}',pos=values(motor_center),zaxis=values(axis))
        cylinder(motor,f'M{j}_stator',[0,0,0],z,motor_radii[index],motor_length,motor_masses[index],'motor_teal',1)
        cylinder(motor,f'M{j}_end_cap',-z*(motor_length/2+0.001),z,motor_radii[index]*0.94,0.002,0,'motor_teal',1)
        add(motor,'site',name=f'M{j}',size='0.003',rgba='0 1 0 0')

        try:
            with np.errstate(invalid='raise', divide='raise'):
                fixed_connections,gear_contacts,gear_vertices=case_mount(parent,f'J{j}_fixed_mount',
                    gear_center,axis,gear_radii[index],frames[parent_name])
                motor_mounts,motor_contacts,motor_vertices=case_mount(parent,f'M{j}_case_mount',
                    motor_center,axis,motor_radii[index],frames[parent_name])
        except (ValueError, FloatingPointError) as exc:
            raise GeometryError(f'Approved case mount cannot fit J{j}: {exc}') from exc
        if parameters is not None:
            _check_mount(f'J{j} reducer', gear_center, gear_radii[index], frames[parent_name], gear_vertices)
            _check_mount(f'J{j} motor', motor_center, motor_radii[index], frames[parent_name], motor_vertices)
            _frame_case_clearance(parent, frames[parent_name], motor_center, axis,
                                  motor_radii[index], motor_length, f'J{j} motor')
            _frame_case_clearance(parent, frames[parent_name], gear_center, axis,
                                  gear_radii[index], housing_length, f'J{j} reducer')
        output_center=local_axis*(shift+0.003)
        output=add(child,'body',name=f'output_coupling_{j}',pos=values(output_center),zaxis=values(local_axis))
        output_radius=gear_core*0.88
        output_mass=2700*math.pi*output_radius**2*0.006
        cylinder(output,f'O{j}_moving_flange',[0,0,0],z,output_radius,0.006,output_mass,'shaft_metal',2)
        add(output,'site',name=f'O{j}',size='0.003',rgba='1 0.8 0 0')
        moving_connections,output_contacts,output_vertices=output_frame(child,f'J{j}_output_frame',
            output_center,local_axis,output_radius,frames[child_name])

        belt_centers=[]
        pulley_radius,belt_width=0.,0.
        if drive=='coaxial':
            # The small input coupling is drivetrain hardware, not a support.
            coupling = joint+axis*(shift-0.031) if parameters is None else joint+axis*(shift-parameters.gear_lengths[index]-.001)
            cylinder(parent,f'M{j}_input_coupling',coupling,axis,
                     0.004,0.004,0.002,'belt_purple',3)
        else:
            gear_input=joint-axis*0.049
            motor_input=motor_center+axis*(-0.049-float((motor_center-joint)@axis))
            if parameters is not None:
                gear_back = joint+axis*(shift-parameters.gear_lengths[index])
                motor_back = motor_center-axis*motor_length/2
                plane = min(float(gear_back@axis), float(motor_back@axis))-.019
                gear_input = gear_center+axis*(plane-float(gear_center@axis))
                motor_input = motor_center+axis*(plane-float(motor_center@axis))
                if np.linalg.norm(gear_input-motor_input) < .028:
                    raise GeometryError(f'J{j} input pulleys do not fit the approved belt layout')
            belt_centers=[motor_input,gear_input]
            pulley_radius,belt_width=0.013,0.002
            for k,center in enumerate(belt_centers):
                cylinder(parent,f'input_pulley_{j}_{k}',center,axis,pulley_radius,0.008,0.012,'belt_purple',3)
            direction=(gear_input-motor_input)/np.linalg.norm(gear_input-motor_input)
            normal=np.cross(axis,direction)
            for sign in (-1,1):
                start=motor_input+normal*sign*pulley_radius
                end=gear_input+normal*sign*pulley_radius
                edge(parent,f'input_belt_{j}_span_{sign}',start,end,'belt_purple',0.001,3)
            if parameters is None:
                cylinder(parent,f'G{j}_input_stub',joint-axis*0.0395,axis,0.004,0.019,0.003,'belt_purple',3)
            else:
                cylinder(parent,f'G{j}_input_stub',(gear_back+gear_input)/2,axis,.004,
                         np.linalg.norm(gear_back-gear_input),.003,'belt_purple',3)
            shaft_start=motor_center-axis*motor_length/2
            edge(parent,f'M{j}_input_stub',shaft_start,motor_input,'belt_purple',0.003,3)

        rotor=2e-5 if j<=3 else 3e-6
        manifest.append({'joint':j,'axis':'XYZ'[int(np.argmax(np.abs(local_axis)))],
            'shaft_axis':axis.tolist(),'output_axis_child':local_axis.tolist(),
            'drive':drive,'ratio':layout.ratios[index],'reducer_count':1,
            'motor_mount':parent_name,'gearbox_mount':parent_name,'output_mount':child_name,
            'shaft_support':parent_name,'motor_center_m':motor_center.tolist(),
            'gearbox_center_m':gear_center.tolist(),'joint_center_m':joint.tolist(),
            'motor_mass_kg':motor_masses[index],'gearbox_mass_kg':gear_masses[index],
            'output_assembly_mass_kg':output_mass,'rotor_inertia_kg_m2':rotor,
            'reflected_armature_kg_m2':rotor*layout.ratios[index]**2,
            'route':drive,'belt_side':'before_reducer' if belt_centers else 'none',
            'belt_crosses_joint':False,'belt_axis':axis.tolist(),
            'belt_centers_m':[point.tolist() for point in belt_centers],
            'pulley_radius_m':pulley_radius,'belt_width_m':belt_width,
            'nominal_belt_tension_difference_N':0 if not belt_centers else
                [100,100,80,20,15,10][index]/layout.ratios[index]/pulley_radius,
            'fixed_support_geoms':fixed_connections,'moving_support_geoms':moving_connections,
            'motor_mount_geoms':motor_mounts,'motor_contact_m':motor_contacts[0].tolist(),
            'gearbox_contact_m':gear_contacts[0].tolist(),
            'motor_contact_points_m':motor_contacts.tolist(),'gearbox_contact_points_m':gear_contacts.tolist(),
            'motor_support_vertices_m':motor_vertices.tolist(),
            'gearbox_support_vertices_m':gear_vertices.tolist(),
            'output_contact_points_m':output_contacts.tolist(),'output_frame_vertices_m':output_vertices.tolist(),
            'load_path':f'{parent_name} edge cage -> orange case mid-height rim -> reducer/bearing -> grey moving output -> planar rim surround + spatial edge cage -> {child_name}'})
        if parameters is not None:
            manifest[-1].update({
                'geometry_template': 'approved_base_mounts_v1',
                'geometry_parameterization': 'existing cage, case_mount and output_frame topology; no fallback brackets',
                'motor_offset_reference': 'delta from approved local placement',
                'parent_frame_dimensions_m': {key: float(frames[parent_name][key])
                                              for key in ('low', 'high', 'width', 'depth')},
                'child_frame_dimensions_m': {key: float(frames[child_name][key])
                                             for key in ('low', 'high', 'width', 'depth')},
            })

    # The terminal grey tool disk is surrounded by the tool cage's end loop.
    tool=bodies['tool_flange_link']
    terminal_center=np.array([0.,0.,0.078])
    cylinder(tool,'tool_mounting_flange',terminal_center,z,0.0195,0.004,
             2700*math.pi*0.0195**2*0.004,'shaft_metal',2)
    for index,direction in enumerate((np.array([1.,0.,0.]),np.array([-1.,0.,0.]),
                                      np.array([0.,1.,0.]),np.array([0.,-1.,0.]))):
        edge(tool,f'tool_tip_contact_{index}',terminal_center+direction*0.0195,
             terminal_center+direction*0.024,radius=0.0015)
    return manifest

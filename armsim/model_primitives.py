"""Small geometric primitives shared by the model builders."""
import math
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

def values(vector) -> str:
    return " ".join(f"{float(value):.16g}" for value in vector)


def rpy_quaternion(rpy):
    roll,pitch,yaw=np.asarray(rpy)/2
    cr,sr=math.cos(roll),math.sin(roll)
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]


def add(parent, tag, **attributes):
    return ET.SubElement(parent, tag, {key: str(value) for key, value in attributes.items()})


def cylinder(parent, name, center, axis, radius, length, mass, material, group):
    axis = np.array(axis, dtype=float)
    center = np.array(center, dtype=float)
    return add(parent, "geom", name=name, type="cylinder",
               fromto=values(np.r_[center - axis * length / 2, center + axis * length / 2]),
               size=values([radius]), mass=mass, material=material,
               contype="0", conaffinity="0", group=group)


def capsule(parent, name, start, end, radius, material, group=3, collision=False):
    return add(parent, "geom", name=name, type="capsule", fromto=values(np.r_[start, end]),
               size=radius, mass="0", material=material, contype=int(collision), conaffinity=int(collision), group=group)


def beam(parent, name, start, end, axis, width, thickness, material, group=3):
    start, end, axis = np.asarray(start), np.asarray(end), np.asarray(axis)
    direction = end - start
    length = np.linalg.norm(direction)
    if length < 1e-8:
        return
    direction /= length
    matrix = np.column_stack((axis, np.cross(direction, axis), direction))
    quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(quaternion, matrix.ravel())
    add(parent, "geom", name=name, type="box", pos=values((start + end) / 2),
        quat=values(quaternion), size=values([width / 2, thickness / 2, length / 2]),
        material=material, mass="0", contype="0", conaffinity="0", group=group)


def closed_belt(parent, name, input_center, output_center, axis, radius, width):
    """Two flat tangent spans; the smooth pulley rims represent the wraps."""
    delta = np.asarray(output_center) - np.asarray(input_center)
    direction = delta / np.linalg.norm(delta)
    normal = np.cross(axis, direction)
    for sign in (-1, 1):
        beam(parent, f"{name}_span_{sign}", input_center + sign * radius * normal,
             output_center + sign * radius * normal, axis, width, 0.002, "belt_purple")
    # The two smooth pulley cylinders communicate wrap; omit segmented arc
    # geometry in the lightweight conceptual display.

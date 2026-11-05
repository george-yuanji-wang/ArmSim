"""Audit downloaded robot kinematics without loading or redistributing meshes."""

import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIR = ROOT / 'references' / 'menagerie'


def main():
    manifest = json.loads((SOURCE_DIR/'sources.json').read_text())
    results = []
    for folder,filename in (('universal_robots_ur5e','ur5e.xml'),
                            ('ufactory_lite6','lite6.xml'),('fanuc_crx5ia','crx5ia.xml')):
        path=SOURCE_DIR/folder/filename
        root=ET.parse(path).getroot()
        # Source definitions are data. Strip all file-backed visual assets and
        # control fixtures; keep body transforms, joints, inertials and defaults.
        for parent in list(root.iter()):
            for child in list(parent):
                if child.tag in ('asset','geom','mesh','material','texture','actuator','sensor','keyframe'):
                    parent.remove(child)
        root.find('compiler').attrib.pop('meshdir',None)
        model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        data=mujoco.MjData(model)
        mujoco.mj_forward(model,data)
        joints=[]
        for index in range(model.njnt):
            joints.append({'name':model.joint(index).name,
                           'local_axis':model.jnt_axis[index].tolist(),
                           'zero_world_axis':np.round(data.xaxis[index],6).tolist(),
                           'zero_world_position_m':np.round(data.xanchor[index],6).tolist()})
        results.append({'model':folder,'revision':manifest['revision'],'joints':joints,
                        'source':next(item['source'] for item in manifest['files'] if item['path'].endswith(folder+'/'+filename))})
    (ROOT/'references/kinematic-audit.json').write_text(json.dumps(results,indent=2)+'\n')
    for item in results:
        print(item['model'],[(j['name'],j['zero_world_axis']) for j in item['joints'][-3:]])


if __name__=='__main__':
    main()

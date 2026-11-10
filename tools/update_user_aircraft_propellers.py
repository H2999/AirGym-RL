"""Replace display-only prop discs with approximate 50.8mm three-blade meshes.

Blade chord, sweep and twist are illustrative, not measured GF2023 airfoils.
This does not change mass, collisions or the measured thrust mapping.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def propeller_obj(mirror=False):
    vertices=[];faces=[]
    # Closed central hub.
    count=32
    for z in [-.0007,.0007]:
        for i in range(count):
            angle=2*math.pi*i/count
            vertices.append([.0042*math.cos(angle),.0042*math.sin(angle),z])
    for i in range(count):
        j=(i+1)%count
        faces.extend([[i,j,count+j],[i,count+j,count+i]])
    for i in range(1,count-1):
        faces.extend([[0,i+1,i],[count,count+i,count+i+1]])
    stations=[(.0038,.0040,38),(.007,.0065,34),(.012,.0075,28),
              (.018,.0064,23),(.023,.0040,19),(.02538,.0015,17)]
    for blade in range(3):
        offset=len(vertices)
        for radius,chord,twist in stations:
            angle=2*math.pi*blade/3+.15*(radius/.0254)**2
            c,s=math.cos(angle),math.sin(angle);pitch=math.radians(twist)
            for y,z in [(-chord/2,-.0002),(chord/2,-.0002),
                        (chord/2,.0002),(-chord/2,.0002)]:
                tangent=y*math.cos(pitch)-z*math.sin(pitch)
                height=y*math.sin(pitch)+z*math.cos(pitch)
                vertices.append([radius*c-tangent*s,radius*s+tangent*c,height])
        for i in range(len(stations)-1):
            for side in range(4):
                a=offset+4*i+side;b=offset+4*i+(side+1)%4
                faces.extend([[a,b,b+4],[a,b+4,a+4]])
        faces.extend([[offset,offset+2,offset+1],[offset,offset+3,offset+2]])
        last=offset+4*(len(stations)-1)
        faces.extend([[last,last+1,last+2],[last,last+2,last+3]])
    if mirror:
        vertices=[[x,-y,z] for x,y,z in vertices]
        faces=[f[::-1] for f in faces]
    lines=['# Approximate three-blade display mesh, metres; no blade collision/aerodynamics.']
    lines.extend('v '+' '.join(f'{v:.9f}' for v in p) for p in vertices)
    lines.extend('f '+' '.join(str(v+1) for v in f) for f in faces)
    return '\n'.join(lines)+'\n'


def set_three_blade_visuals(root,folder):
    (folder/'meshes').mkdir(parents=True,exist_ok=True)
    for hand,mirror in [('cw',False),('ccw',True)]:
        (folder/f'meshes/propeller_3blade_{hand}.obj').write_text(propeller_obj(mirror))
    for i in range(1,5):
        link=root.find(f"./link[@name='prop_{i}']")
        if link is None:raise ValueError(f'Missing prop_{i}')
        for visual in list(link.findall('visual')):
            cylinder=visual.find('geometry/cylinder');mesh=visual.find('geometry/mesh')
            if ((cylinder is not None and float(cylinder.get('radius'))>.02) or
                (mesh is not None and 'propeller_3blade_' in mesh.get('filename',''))):
                link.remove(visual)
        hand='cw' if i<=2 else 'ccw'
        visual=ET.SubElement(link,'visual');ET.SubElement(visual,'origin',xyz='0 0 0.008')
        ET.SubElement(ET.SubElement(visual,'geometry'),'mesh',filename=f'meshes/propeller_3blade_{hand}.obj')
        color='0.25 0.50 0.85 1' if i<=2 else '0.90 0.38 0.12 1'
        ET.SubElement(ET.SubElement(visual,'material',name=f'propeller_{hand}'),'color',rgba=color)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot',type=Path,default=Path('airgym/assets/robots/user160'))
    args=parser.parse_args();path=args.robot/'model.urdf';tree=ET.parse(path)
    set_three_blade_visuals(tree.getroot(),args.robot)
    ET.indent(tree);tree.write(path,encoding='utf-8',xml_declaration=True)
    params=args.robot/'parameters.json'
    if params.exists():
        data=json.loads(params.read_text())
        data['propeller_visual']={'blade_count':3,'nominal_diameter_m':.0508,
            'geometry':'approximate chord/sweep/twist, not measured GF2023 geometry',
            'display_only':True,'blade_collision':False,'blade_aerodynamics':False}
        params.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    print('Updated four three-blade propeller visuals; physical properties unchanged.')


if __name__=='__main__':main()

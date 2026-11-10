"""Build the user's rigid URDF from locally extracted SolidWorks meshes.

Offline dependencies: numpy, trimesh, scipy, coacd. None are needed at runtime.
Geometry comes from CAD; component masses and motor Z placement are estimates.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import coacd
from update_user_aircraft_propellers import set_three_blade_visuals


def vector(values):
    return ' '.join(f'{v:.10g}' for v in values)


def inertia_xml(parent, mass, center, inertia):
    e = ET.SubElement(parent, 'inertial')
    ET.SubElement(e, 'origin', xyz=vector(center))
    ET.SubElement(e, 'mass', value=str(mass))
    ET.SubElement(e, 'inertia', **{k: f'{inertia[i,j]:.10g}' for k,i,j in
        [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})


def geometry_xml(parent, tag, filename, transform=None, color=None):
    e = ET.SubElement(parent, tag)
    if transform is not None:
        ET.SubElement(e, 'origin', xyz=vector(transform))
    ET.SubElement(ET.SubElement(e, 'geometry'), 'mesh', filename=filename)
    if color:
        name='material_'+color.replace(' ','_').replace('.','_')
        ET.SubElement(ET.SubElement(e, 'material', name=name), 'color', rgba=color)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cad', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output; (out/'meshes').mkdir(parents=True, exist_ok=True)
    data = json.loads((args.cad/'parts.json').read_text())
    root = ET.parse(args.cad/'assembly.xml').getroot()
    ns = {'s': root.tag.split('}')[0][1:]}
    files = {a.get('id'): a.get('swPath').split('\\')[-1] for a in root.findall('.//s:swFile',ns)}
    models = {a.get('id'): files[a.get('swFileRef')] for a in root.findall('.//s:swModel',ns)}
    meshes = {p['filename']: trimesh.load(args.cad/p['mesh'], force='mesh') for p in data['parts']}
    # Relative mass estimates, not measured component masses. Scale to 142g,
    # reserving the datasheet's 4 x 4.5g for the absent motor geometry.
    relative = {'2s电池':35, 'DM飞控板':5, 'M2x22铝柱':.45, 'MicoVIO':5,
        '下机架':8, '下边机架二板子':6, '中间件':8, '中间机架二板子':6,
        '四合一电调':4, '摄像头':2, '摄像头固定':2, '香橙派':25, '香橙派固定':5}
    refs = root.findall('.//s:swReference',ns)
    if any(r.get('swSuppressed') != 'NO' or r.get('swHidden') != 'NO' for r in refs):
        raise ValueError('Review hidden/suppressed components before converting')
    if any(r.findall('s:swReference',ns) for r in refs):
        raise ValueError('Nested assemblies require composed transforms')
    assemblies = []; instances = []
    for r in refs:
        filename = models[r.get('swModelRef')]; m = meshes[filename].copy()
        t = np.array(list(map(float,r.get('swTransform').split()))).reshape(4,4).T
        if not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-8):
            raise ValueError('Non-rigid component transform')
        m.apply_transform(t); assemblies.append(m)
        instances.append({'filename':filename, 'name':r.get('swName'),
            'instance':r.get('swReferenceNumber'), 'transform':t})
    assembled = trimesh.util.concatenate(assemblies)
    expected = np.array(list(map(float,root.findall('.//s:swModel',ns)[-1].get('swBoundingBox').split()))).reshape(2,3)
    error = float(np.abs(expected-assembled.bounds).max())
    if error > .0005:
        raise ValueError(f'Assembly bounds disagree with CAD metadata by {error} m')
    # Camera lies towards CAD +Y; body X is forward, body Y is left.
    rot = np.array([[0,1,0],[-1,0,0],[0,0,1]])
    anchor = np.array([-.004956630200534,-.0144192878560917,1.0])
    transform = np.eye(4); transform[:3,:3]=rot; transform[:3,3]=-rot@anchor
    weights = np.array([relative[i['name']] for i in instances]); masses=.142*weights/weights.sum()
    centers=[]; inertias=[]
    for m,mass in zip(assemblies,masses):
        m.apply_transform(transform)
        if m.is_watertight and m.volume>0:
            center=m.center_mass; inertia=m.moment_inertia*mass/m.volume
        else:
            center=m.bounds.mean(0); a,b,c=m.extents
            inertia=np.diag([b*b+c*c,a*a+c*c,a*a+b*b])*mass/12
        centers.append(center); inertias.append(inertia)
    motor_z=.002+.0089/2  # estimate: bottom motor face on the 2mm lower plate
    motors=np.array([[.03,-.0405,motor_z],[-.03,.0405,motor_z],
                     [.03,.0405,motor_z],[-.03,-.0405,motor_z]])
    centers=np.array(centers)
    com=((masses[:,None]*centers).sum(0)+.0045*motors.sum(0))/.160
    base_com=(masses[:,None]*centers).sum(0)/.142-com
    base_inertia=np.zeros((3,3))
    for mass,c,i in zip(masses,centers,inertias):
        delta=c-com-base_com
        base_inertia+=i+mass*(np.dot(delta,delta)*np.eye(3)-np.outer(delta,delta))
    urdf=ET.Element('robot',name='user160_cad'); base=ET.SubElement(urdf,'link',name='base_link')
    inertia_xml(base,.142,base_com,base_inertia)
    coacd.set_log_level('error'); hulls={}; part_records=[]
    for number,(filename,m) in enumerate(meshes.items()):
        stem='part_'+str(number).zfill(2); meshfile='meshes/'+stem+'.obj'
        m.export(out/meshfile)
        # Keep small holes out of the solver where possible. Each result is
        # exported as an explicit convex shape, avoiding one giant frame hull.
        structural = Path(filename).stem in {'下机架','下边机架二板子','中间件','中间机架二板子'}
        if not structural or m.is_convex:
            hull=m.convex_hull
            pieces=[(hull.vertices,hull.faces)]
        else:
            pieces=coacd.run_coacd(coacd.Mesh(m.vertices,m.faces),threshold=.025,
                max_convex_hull=32,preprocess_resolution=80,resolution=1000,
                mcts_nodes=10,mcts_iterations=50,max_ch_vertex=64,decimate=True,seed=42)
        hulls[filename]=[]
        for j,(v,f) in enumerate(pieces):
            name=f'meshes/{stem}_collision_{j:02d}.obj'
            trimesh.Trimesh(v,f).convex_hull.export(out/name); hulls[filename].append(name)
        part_records.append({'file':filename,'visual':meshfile,'collision_hulls':len(pieces),
            'watertight':bool(m.is_watertight)})
        print(filename,len(pieces),'convex shapes',flush=True)
    # Bake each saved assembly transform into meshes. Leaves exactly 5 links
    # so the established motor body indices 1..4 remain valid.
    collision_count=0
    for n,(instance,m,mass) in enumerate(zip(instances,assemblies,masses)):
        m.vertices-=com
        visual=f'meshes/instance_{n:02d}.obj';m.export(out/visual)
        color='0.65 0.08 0.08 1' if instance['name']=='M2x22铝柱' else '0.16 0.18 0.20 1'
        geometry_xml(base,'visual',visual,color=color)
        t=transform@instance['transform'];t[:3,3]-=com
        for j,h in enumerate(hulls[instance['filename']]):
            c=trimesh.load(out/h,force='mesh');c.apply_transform(t)
            dest=f'meshes/contact_{n:02d}_{j:02d}.obj';c.export(out/dest)
            geometry_xml(base,'collision',dest);collision_count+=1
        instance['transform']=instance['transform'].tolist(); instance['estimated_mass_kg']=float(mass)
    motor_inertia=np.diag([.0045*(3*.0079**2+.0089**2)/12]*2+[.0045*.0079**2/2])
    for j,position in enumerate(motors-com):
        link=ET.SubElement(urdf,'link',name=f'prop_{j+1}')
        inertia_xml(link,.0045,[0,0,0],motor_inertia)
        for tag in ['visual','collision']:
            e=ET.SubElement(link,tag);ET.SubElement(ET.SubElement(e,'geometry'),'cylinder',radius='.0079',length='.0089')
        joint=ET.SubElement(urdf,'joint',name=f'base_link2prop_{j+1}',type='fixed',dont_collapse='true')
        ET.SubElement(joint,'parent',link='base_link');ET.SubElement(joint,'child',link=f'prop_{j+1}')
        ET.SubElement(joint,'origin',xyz=vector(position))
    set_three_blade_visuals(urdf,out)
    ET.indent(urdf);ET.ElementTree(urdf).write(out/'model.urdf',encoding='utf-8',xml_declaration=True)
    total_inertia=base_inertia+.142*(base_com@base_com*np.eye(3)-np.outer(base_com,base_com))
    for pos in motors-com:
        total_inertia+=motor_inertia+.0045*(pos@pos*np.eye(3)-np.outer(pos,pos))
    cad_to_body=transform.copy();cad_to_body[:3,3]-=com
    report={'profile_id':'user160_cad_v1','mass_kg':.160,'mass_distribution':'estimated relative component weights normalized to 142g + four 4.5g motors',
        'cad_to_body_transform_m':cad_to_body.tolist(),
        'cad_bounds_error_m':error,'cad_size_m':assembled.extents.tolist(),
        'body_axes':'X towards CAD +Y camera; Y towards CAD -X; Z up',
        'body_com_from_lower_plate_center_m':com.tolist(),'estimated_inertia_kg_m2':total_inertia.tolist(),
        'motor_positions_m':(motors-com).tolist(),'estimated_motor_body_z_above_lower_plate_m':motor_z,
        'motor_positions_z_estimated':True,'motor_spin_directions_assumed':[-1,-1,1,1],
        'collision_shapes':collision_count+4,'collision_method':'frame plates: CoACD threshold 0.025, at most 32 hulls/part; electronics/posts: single convex hull/component',
        'parts':part_records,'instances':instances,'source':data}
    (out/'build_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    # Preserve an assembled visual for independent viewing and comparison.
    whole=trimesh.util.concatenate(assemblies);whole.export(out/'assembled_visual.stl')
    referenced={e.get('filename') for e in urdf.findall('.//mesh')}
    referenced.update(p['visual'] for p in part_records)
    for generated in (out/'meshes').glob('*.obj'):
        if generated.relative_to(out).as_posix() not in referenced:
            generated.unlink()
    print('Wrote',out/'model.urdf','with',collision_count+4,'collision shapes',flush=True)


if __name__=='__main__':main()

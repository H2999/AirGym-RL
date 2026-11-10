"""Cross-check the assembled URDF visual against an independently exported STEP.

Offline Python dependencies: cadquery-ocp, numpy, scipy, trimesh, rtree.
OpenCascade's STEP reader uses millimetres internally; generated robot meshes
use metres. This validates geometry, not the assumed mass distribution.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def counts(shape,kind):
    explorer=TopExp_Explorer(shape,kind);count=0
    while explorer.More():
        count+=1;explorer.Next()
    return count


def distances(source,target,n,seed):
    np.random.seed(seed)
    points,_=trimesh.sample.sample_surface(source,n)
    # Add every mesh vertex: a small omitted feature may be missed by uniform
    # surface sampling alone. Query in batches to bound temporary allocation.
    points=np.vstack([points,source.vertices])
    result=[]
    for start in range(0,len(points),1024):
        _,d,_=trimesh.proximity.closest_point(target,points[start:start+1024])
        result.append(d)
    d=np.concatenate(result)
    return {'points':len(d),'mean_m':float(d.mean()),'p99_m':float(np.quantile(d,.99)),
        'max_m':float(d.max()),'fraction_over_0_1mm':float((d>.0001).mean())}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step',type=Path,required=True)
    parser.add_argument('--robot',type=Path,default=Path('airgym/assets/robots/user160'))
    parser.add_argument('--report',type=Path,default=Path('docs/user_aircraft_assembly_step_validation.json'))
    args=parser.parse_args();build=json.loads((args.robot/'build_report.json').read_text())
    reader=STEPControl_Reader()
    if reader.ReadFile(str(args.step))!=IFSelect_RetDone:
        raise ValueError('STEP read failed')
    if not reader.TransferRoots():raise ValueError('STEP has no transferred geometry')
    shape=reader.OneShape()
    if not BRepCheck_Analyzer(shape).IsValid():raise ValueError('Invalid STEP topology')
    properties=GProp_GProps();BRepGProp.VolumeProperties_s(shape,properties)
    BRepMesh_IncrementalMesh(shape,.02,False,.15,True)
    with tempfile.TemporaryDirectory(prefix='airgym-step-check-') as folder:
        stl=Path(folder)/'assembly.stl'
        if not StlAPI_Writer().Write(shape,str(stl)):raise ValueError('STEP tessellation export failed')
        step_mesh=trimesh.load(stl,force='mesh');step_mesh.apply_scale(.001)
    step_size=step_mesh.extents.copy()
    step_mesh.apply_transform(np.array(build['cad_to_body_transform_m']))
    # Read the actual URDF visual references, not only a parallel preview file.
    # Parametric motor/propeller additions are absent from the CAD export.
    visuals=[]
    urdf=ET.parse(args.robot/'model.urdf')
    for visual in urdf.findall('./link[@name="base_link"]/visual'):
        mesh=visual.find('geometry/mesh')
        if mesh is None:raise ValueError('Expected CAD mesh visual on base_link')
        m=trimesh.load(args.robot/mesh.get('filename'),force='mesh')
        m.apply_scale([float(v) for v in mesh.get('scale','1 1 1').split()])
        origin=visual.find('origin')
        if origin is not None:
            rpy=[float(v) for v in origin.get('rpy','0 0 0').split()]
            transform=trimesh.transformations.euler_matrix(*rpy,axes='sxyz')
            transform[:3,3]=[float(v) for v in origin.get('xyz','0 0 0').split()]
            m.apply_transform(transform)
        visuals.append(m)
    if not visuals:raise ValueError('URDF has no CAD visuals')
    robot_mesh=trimesh.util.concatenate(visuals)
    bound_error=float(np.abs(step_mesh.bounds-robot_mesh.bounds).max())
    print('Comparing full STEP and saved assembly mesh surfaces...',flush=True)
    forward=distances(step_mesh,robot_mesh,8192,171)
    backward=distances(robot_mesh,step_mesh,8192,172)
    passed=bound_error<.0005 and all(r['p99_m']<.0005 and r['max_m']<.001 for r in [forward,backward])
    report={'profile':build['profile_id'],'source_step':str(args.step.resolve()),
        'source_step_sha256':sha(args.step),'urdf_sha256':sha(args.robot/'model.urdf'),
        'assembled_visual_sha256':sha(args.robot/'assembled_visual.stl'),
        'urdf_cad_visual_instances':len(visuals),
        'step_topology_valid':True,'step_solids':counts(shape,TopAbs_SOLID),
        'step_faces':counts(shape,TopAbs_FACE),'step_size_cad_xyz_m':step_size.tolist(),
        'exact_step_volume_m3':properties.Mass()*1e-9,'step_meshing_deflection_mm':.02,
        'body_bounds_error_m':bound_error,'step_to_urdf_surface':forward,
        'urdf_to_step_surface':backward,'passed':passed,
        'criteria':{'bounds_max_m':.0005,'surface_p99_max_m':.0005,'surface_max_m':.001},
        'scope':'URDF base_link CAD visuals only; added parametric motors/props, collision convex decomposition, mass distribution, inertia and contact material are not calibrated by this check'}
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    if not passed:raise ValueError('Assembly differs from STEP; inspect geometry before using it')


if __name__=='__main__':main()

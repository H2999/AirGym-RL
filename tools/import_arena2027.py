"""Convert Arena-1 2027 static structures. Requires trimesh and numpy (offline after conversion)."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import trimesh


def convert(source, destination):
    destination.mkdir(parents=True, exist_ok=True)
    rules = json.loads((source / 'config/rules.json').read_text())
    arena = rules['maps']['rmuc2027']
    # glTF (x, up, depth) -> Isaac Gym (x, -depth, up), metres.
    transform = np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]], dtype=float)
    records = []
    hashes = {}
    for kind in ('base', 'outpost'):
        asset = source / 'frontend/public' / arena['structure_assets'][kind].lstrip('/')
        hashes[asset.name] = hashlib.sha256(asset.read_bytes()).hexdigest()
        scene = trimesh.load(asset, force='scene')
        for team in ('blue',):
            name = 'arena2027_' + team + '_' + kind
            folder = destination / name
            folder.mkdir(exist_ok=True)
            robot = ET.Element('robot', name=name)
            link = ET.SubElement(robot, 'link', name='base_link')
            parts = []
            if kind == 'outpost':
                for tag in ('visual', 'collision'):
                    part = ET.SubElement(link, tag, name='plinth')
                    ET.SubElement(part, 'origin', xyz='0 0 -0.05')
                    shape = ET.SubElement(part, 'geometry')
                    ET.SubElement(shape, 'cylinder', radius='0.4', length='0.1')
                    if tag == 'visual':
                        mat = ET.SubElement(part, 'material', name='plinth_gray')
                        ET.SubElement(mat, 'color', rgba='0.25 0.25 0.25 1')
            for i, node in enumerate(scene.graph.nodes_geometry):
                pose, geometry = scene.graph[node]
                mesh = scene.geometry[geometry].copy()
                material = mesh.visual.material
                rgba = np.array(material.baseColorFactor if material.baseColorFactor is not None else [180,180,180,255], dtype=float) / 255
                if material.name and material.name.startswith(('TeamPrimary', 'TeamGlow')):
                    rgba = [0.85,0.055,0.07,1] if team == 'red' else [0.04,0.32,0.95,1]
                mesh.apply_transform(transform @ pose)
                # Bake each node transform and material: OBJ works with Isaac Gym's importer.
                mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh)
                filename = 'part_%03d.obj' % i
                mesh.export(folder / filename)
                parts.append(dict(name=node, mesh=filename, local_bounds=mesh.bounds.tolist()))
                visual = ET.SubElement(link, 'visual', name=node)
                geom = ET.SubElement(visual, 'geometry')
                ET.SubElement(geom, 'mesh', filename=filename)
                mat = ET.SubElement(visual, 'material', name='material_%d' % i)
                ET.SubElement(mat, 'color', rgba=' '.join(map(str,rgba)))
                # Per-component convex hulls preserve gaps between components, unlike one
                # convex hull around the whole building. These are approximate collision shapes.
                if len(mesh.vertices) >= 4:
                    collision = ET.SubElement(link, 'collision', name='collision_%d' % i)
                    geom = ET.SubElement(collision, 'geometry')
                    ET.SubElement(geom, 'mesh', filename=filename)
            ET.ElementTree(robot).write(folder / 'model.urdf', encoding='utf-8', xml_declaration=True)
            (folder / 'parts.json').write_text(json.dumps(parts, indent=2)+'\n')
            x,y = arena[kind+'s'][team]
            z = min(y,arena['height']-y) * math.tan(math.radians(arena['floor_slope_deg']))
            if kind == 'outpost': z += arena['outpost_plinth_m']
            records.append(dict(name=name, kind=kind, team=team, position=[x-arena['width']/2,y-arena['height']/2,z], yaw=math.pi if kind=='base' and team=='blue' else 0))
    (destination/'layout.json').write_text(json.dumps(dict(source='https://github.com/RM-Arena/Arena-1', source_sha256=hashes, season=2027, arena_size=[arena['width'],arena['height']], floor_slope_deg=arena['floor_slope_deg'], structures=records), indent=2)+'\n')
    for name in ('LICENSE','THIRD_PARTY_NOTICES.md'):
        shutil.copyfile(source/name, destination/('Arena-1-'+name))

if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1]/'airgym/assets/env_assets/arena2027')
    args=parser.parse_args()
    convert(args.source,args.output)

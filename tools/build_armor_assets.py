"""Split selected armor into its own fixed actor for unambiguous PhysX contacts.

Run from any directory. Original arena URDFs and meshes are never modified.
"""
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1] / 'airgym/assets/env_assets/arena2027'


def build():
    for kind, panel_name in [('outpost', 'ArmorPanel'), ('base', 'SmallArmor')]:
        folder = ROOT / ('arena2027_blue_' + kind)
        parts = json.loads((folder / 'parts.json').read_text())
        panel = next(p for p in parts if p['name'] == panel_name)
        # Include the outward light/weakspot overlay in the same contact actor.
        names = {panel_name}
        if kind == 'outpost':
            names |= {'OutpostArmor_0_TeamGlow', 'WeakSpot'}
        else:
            names |= {'TopArmor_0_TeamGlow'}
        meshes = {p['mesh'] for p in parts if p['name'] in names}
        original = ET.parse(folder / 'model.urdf')
        link = original.getroot().find('link')
        plate = ET.Element('robot', name='impact_' + kind + '_plate')
        plink = ET.SubElement(plate, 'link', name='armor_plate')
        for node in list(link):
            mesh = node.find('geometry/mesh')
            if mesh is not None and mesh.get('filename') in meshes:
                plink.append(copy.deepcopy(node))
                link.remove(node)
        original.write(folder / 'impact_structure.urdf', encoding='utf-8', xml_declaration=True)
        ET.ElementTree(plate).write(folder / 'impact_plate.urdf', encoding='utf-8', xml_declaration=True)
        print(kind, panel_name, 'bounds:', panel['local_bounds'])


if __name__ == '__main__':
    build()

"""Static Arena-1 RMUC 2027 structures, in metres with arena centre at (0,0)."""
import json
from pathlib import Path
from airgym import AIRGYM_ROOT_DIR
from airgym.assets import registry
from airgym.envs.base.hovering_config import HoveringCfg

ASSET_ROOT = Path(AIRGYM_ROOT_DIR) / 'airgym/assets/env_assets/arena2027'
LAYOUT = json.loads((ASSET_ROOT / 'layout.json').read_text())
# Only the blue side is instantiated for impact training.
STRUCTURES = [s for s in LAYOUT['structures'] if s['team'] == 'blue']
for structure in STRUCTURES:
    registry.register_asset(structure['name'], {
        'path': str(ASSET_ROOT / structure['name'] / 'model.urdf'),
        'fix_base_link': True, 'disable_gravity': True,
        'collapse_fixed_joints': True, 'links_per_asset': 1,
        'set_whole_body_semantic_mask': True,
        'semantic_id': 20 if structure['kind'] == 'base' else 21,
        'collision_mask': 0, 'num_assets': 1,
        'initial_position': structure['position'], 'initial_yaw': structure['yaw'],
    })

class Arena2027Cfg(HoveringCfg):
    class env(HoveringCfg.env):
        num_envs = 1
        env_spacing = 18
        create_ground_plane = True
        spawn_position = [0, -3.9, 1.4]

    class viewer(HoveringCfg.viewer):
        pos = [14, -13, 10]
        lookat = [6, -2, 0.9]

    class sim(HoveringCfg.sim):
        class physx(HoveringCfg.sim.physx):
            # Required when a later impact task reads net contact forces.
            contact_collection = 2

    class asset_config(HoveringCfg.asset_config):
        include_single_asset = {s['name']: {} for s in STRUCTURES}

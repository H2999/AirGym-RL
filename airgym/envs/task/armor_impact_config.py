"""5 m/s armor contact followed by a two-second recovery (rigid-body simulation)."""
import json
from airgym.assets import registry
from airgym.envs.base.hovering_config import HoveringCfg
from .arena2027_config import ASSET_ROOT, STRUCTURES

TARGETS = []
ASSETS = {}
for i, structure in enumerate(STRUCTURES):
    folder = ASSET_ROOT / structure['name']
    panel_name = 'ArmorPanel' if structure['kind'] == 'outpost' else 'SmallArmor'
    panel = next(p for p in json.loads((folder / 'parts.json').read_text()) if p['name'] == panel_name)
    TARGETS.append(dict(structure, panel=panel))
    for suffix, filename in [('structure', 'impact_structure.urdf'), ('plate', 'impact_plate.urdf')]:
        name = 'impact_' + structure['kind'] + '_' + suffix
        registry.register_asset(name, {
            'path': str(folder / filename), 'fix_base_link': True,
            'disable_gravity': True, 'collapse_fixed_joints': True,
            'links_per_asset': 1, 'set_whole_body_semantic_mask': True,
            'semantic_id': 30 + 2*i + (suffix == 'plate'),
            'collision_mask': 0, 'num_assets': 1,
            'initial_position': structure['position'], 'initial_yaw': structure['yaw'],
        })
        ASSETS[name] = {}


class ArmorImpactCfg(HoveringCfg):
    class env(HoveringCfg.env):
        num_envs = 64
        num_observations = 33
        env_spacing = 18
        episode_length_s = 8.0
        create_ground_plane = True
        reset_on_collision = False
        get_privileged_obs = False

    class impact:
        max_body_rate = 3.0               # rad/s; normalized policy rates map to +/-3
        speed = 5.0                       # normal approach speed, m/s
        speed_tolerance = 0.5             # valid range 4.5--5.5 m/s
        max_tangential_speed = 1.0
        recovery_seconds = 2.0
        stable_hold_seconds = 0.25
        max_tilt_deg = 30.0
        max_recovery_speed = 0.7
        max_recovery_angvel = 1.5
        max_recovery_position_error = 1.0
        recovery_standoff = 0.7
        contact_force_threshold = 0.5     # N; numerical contact threshold, NOT damage rating
        # Expanded center corridor accounts for the drone footprint; plate force
        # is still required, so being nearby never constitutes a hit.
        contact_center_margin = 0.22
        spawn_distance = [1.5, 3.0]
        spawn_lateral_noise = 0.08
        spawn_vertical_noise = 0.04
        # Training-only starts near impact to expose PPO to recovery early.
        # Evaluation explicitly sets this to zero and starts from hover.
        warm_start_probability = 0.10
        warm_start_distance = [0.45, 0.8]
        restitution_range = [0.05, 0.35]
        friction_range = [0.3, 0.7]
        min_height = 0.28
        max_height = 4.0
        max_target_distance = 6.0
        max_angular_speed = 40.0

    class viewer(HoveringCfg.viewer):
        pos = [8, -8, 4]
        lookat = [3.3, -3.9, 1.3]

    class sim(HoveringCfg.sim):
        # Keep the PX4 wrapper's 0.01 s control interval. Four PhysX substeps
        # resolve a 5 m/s approach at 12.5 mm per substep.
        substeps = 4
        class physx(HoveringCfg.sim.physx):
            num_position_iterations = 8
            num_velocity_iterations = 4
            contact_offset = 0.003
            rest_offset = 0.0
            contact_collection = 2
            max_gpu_contact_pairs = 2**20
            default_buffer_size_multiplier = 2
            bounce_threshold_velocity = 0.2
            max_depenetration_velocity = 3.0

    class asset_config(HoveringCfg.asset_config):
        include_single_asset = ASSETS

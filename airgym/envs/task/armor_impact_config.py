"""5 m/s armor contact followed by a two-second recovery (rigid-body simulation)."""
import json
from pathlib import Path
import numpy as np
from airgym.assets import registry
from airgym.envs.base.hovering_config import HoveringCfg
from .arena2027_config import ASSET_ROOT, STRUCTURES

USER_AIRCRAFT = Path(__file__).resolve().parents[2]/'assets/robots/user160'
BUILD = json.loads((USER_AIRCRAFT/'build_report.json').read_text())
registry.register_asset('user160', {
    'path':str(USER_AIRCRAFT/'model.urdf'), 'name':'user160',
    'fix_base_link':False, 'disable_gravity':False,
    'collapse_fixed_joints':True, 'replace_cylinder_with_capsule':False,
    'vhacd_enabled':False, 'angular_damping':0., 'linear_damping':0.,
})
# Simulation tuning, not the user's measured flight-controller gains. Choose
# an approximately 50/s small-error body-rate proportional response from
# estimated inertia. Integral feedback compensates the estimated COM offset.
MOTOR_FULL_N = .127*9.80665
ESTIMATED_KAPPA = .005
RATE_P = (np.diag(np.array(BUILD['estimated_inertia_kg_m2']))*50 /
    np.array([4*.0405*MOTOR_FULL_N*.707107,4*.030*MOTOR_FULL_N*.707107,
              4*ESTIMATED_KAPPA*MOTOR_FULL_N])).tolist()

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
    task_profile = 'user160_cad_v1'

    class steel_plate:
        material = 'steel'
        mounting = 'fixed_rigid'
        # Uncalibrated contact uncertainty. Steel's name alone does not specify
        # the friction or restitution of the aircraft-frame/plate contact pair.
        friction_range = [0.3, 0.7]
        restitution_range = [0.0, 0.35]
        parameters_calibrated = False
        color = [0.55, 0.58, 0.62]

    class actuation:
        half_command_thrust_n = .066*9.80665
        full_command_thrust_n = MOTOR_FULL_N
        torque_to_thrust_m = ESTIMATED_KAPPA  # unmeasured reaction torque estimate
        rate_pid_p = RATE_P
        rate_pid_i = [p*8 for p in RATE_P]
        rate_pid_d = [0.,0.,0.]
    class env(HoveringCfg.env):
        num_envs = 64
        num_observations = 30
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
        require_low_speed = False         # primary objective is attitude recovery
        max_recovery_speed = 0.7          # optional extra criterion if enabled
        max_recovery_angvel = 1.5
        contact_force_threshold = 0.5     # N; numerical contact threshold, NOT damage rating
        # Expanded center corridor accounts for the drone footprint; plate force
        # is still required, so being nearby never constitutes a hit.
        contact_center_margin = 0.22
        spawn_distance = [1.5, 3.0]
        spawn_lateral_noise = 0.08
        spawn_vertical_noise = 0.04
        # Default training and evaluation both start stationary. Optional reset
        # curricula may be enabled explicitly; they never set post-impact state.
        warm_start_probability = 0.0
        warm_start_distance = [0.45, 0.8]
        restitution_range = [0.0, 0.35]  # uncalibrated material uncertainty, not a bounce command
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
        include_robot = {'user160': dict(HoveringCfg.asset_config.include_robot['X152b'])}
        include_single_asset = ASSETS

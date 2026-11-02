"""Preview the single-side 2027 scene; defaults to one drone in position control."""
from pathlib import Path
import isaacgym
from isaacgym import gymapi, gymutil
import torch
from airgym.envs import *
from airgym.utils import task_registry
from airgym.envs.task.arena2027_config import STRUCTURES


def parse_args():
    args = gymutil.parse_arguments(description=__doc__, custom_parameters=[
        {'name': '--headless', 'action': 'store_true'},
        {'name': '--num_envs', 'type': int, 'default': 1},
        {'name': '--steps', 'type': int, 'default': None},
        {'name': '--screenshot', 'type': str, 'default': None},
        {'name': '--task', 'type': str, 'default': 'arena2027', 'choices': ['arena2027']},
        {'name': '--ctl_mode', 'type': str, 'default': 'pos', 'choices': ['pos','vel','atti','rate','prop']},
        {'name': '--seed', 'type': int, 'default': 0},
    ])
    args.sim_device_id = args.compute_device_id
    args.sim_device = args.sim_device_type
    if args.sim_device == 'cuda':
        args.sim_device += ':' + str(args.sim_device_id)
    return args


if __name__ == '__main__':
    args = parse_args()
    env, cfg = task_registry.make_env(name=args.task, args=args)
    try:
        camera = None
        if args.screenshot:
            props = gymapi.CameraProperties()
            props.width, props.height = 1280, 720
            camera = env.gym.create_camera_sensor(env.envs[0], props)
            env.gym.set_camera_location(camera, env.envs[0], gymapi.Vec3(*cfg.viewer.pos), gymapi.Vec3(*cfg.viewer.lookat))
        env.reset()
        command = torch.zeros((env.num_envs, env.num_actions), device=env.device)
        if args.ctl_mode == 'pos':
            command[:, :3] = torch.tensor(cfg.env.spawn_position, device=env.device)
        for _ in range(args.steps if args.steps is not None else (300 if args.headless else 1000000)):
            env.step(command)
        assert torch.isfinite(env.root_states).all(), 'Non-finite drone state'
        if args.screenshot:
            env.gym.render_all_camera_sensors(env.sim)
            output = Path(args.screenshot).resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            env.gym.write_camera_image_to_file(env.sim, env.envs[0], camera, gymapi.IMAGE_COLOR, str(output))
            print('Screenshot:', output)
            for structure in STRUCTURES:
                x,y,z = structure['position']
                side = -1 if structure['kind'] == 'base' else 1
                env.gym.set_camera_location(camera, env.envs[0], gymapi.Vec3(x+side*2.4,y-2.4,z+1.8), gymapi.Vec3(x,y,z+.8))
                env.gym.render_all_camera_sensors(env.sim)
                closeup = output.with_name(output.stem+'_'+structure['kind']+output.suffix)
                env.gym.write_camera_image_to_file(env.sim, env.envs[0], camera, gymapi.IMAGE_COLOR, str(closeup))
                print('Screenshot:', closeup)
        print('Arena2027 preview completed; environments:', env.num_envs, 'structures per environment:', env.asset_manager.get_env_asset_count())
    finally:
        if env.viewer:
            env.gym.destroy_viewer(env.viewer)
        env.gym.destroy_sim(env.sim)

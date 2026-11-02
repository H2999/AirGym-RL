"""Physical integration check: fixed layout, partial resets, drone/building contacts."""
import json
import math
import isaacgym
from isaacgym import gymapi, gymtorch
import torch
from airgym.envs import *
from airgym.scripts.arena2027_preview import parse_args
from airgym.utils import task_registry
from airgym.envs.task.arena2027_config import ASSET_ROOT, STRUCTURES


def check(env):
    assert env.asset_manager.get_env_asset_count() == 2
    before = env.env_asset_root_states.clone()
    expected = torch.tensor([s['position'] for s in STRUCTURES], device=env.device)
    assert torch.allclose(before[0, :, :3], expected, atol=1e-5), before
    command = torch.zeros((env.num_envs, 4), device=env.device)
    command[:, :3] = torch.tensor(env.cfg.env.spawn_position, device=env.device)
    for _ in range(25):
        env.step(command)
    env.reset_idx(torch.tensor([0], device=env.device))
    for _ in range(10):
        env.step(command)
    assert torch.allclose(env.env_asset_root_states, before, atol=1e-5), 'Buildings moved on reset'
    assert torch.isfinite(env.root_states).all()
    contacts = gymtorch.wrap_tensor(env.gym.acquire_net_contact_force_tensor(env.sim)).view(env.num_envs, -1, 3)
    peaks = {}
    for index, structure in enumerate(STRUCTURES):
        parts = json.loads((ASSET_ROOT / structure['name'] / 'parts.json').read_text())
        plate = next(p for p in parts if p['name'] == 'SmallArmor')
        bounds = torch.tensor(plate['local_bounds'], device=env.device)
        local = bounds.mean(dim=0)
        yaw = structure['yaw']
        c,s = math.cos(yaw), math.sin(yaw)
        rotation = torch.tensor([[c,-s,0],[s,c,0],[0,0,1]], device=env.device)
        target = rotation @ local + expected[index]
        normal = torch.tensor([c,s,0], device=env.device)
        # A short ballistic approach from outside the front upper plate.
        env.root_states[0] = 0
        env.root_states[0, :3] = target + .35 * normal + torch.tensor([0,0,.03],device=env.device)
        env.root_states[0, 6] = 1
        env.root_states[0, 7:10] = -4 * normal
        env.gym.set_actor_root_state_tensor(env.sim, env.root_tensor)
        peak = 0.0
        building_body = env.asset_manager.get_robot_num_bodies() + index
        for _ in range(15):
            env.gym.simulate(env.sim)
            env.gym.fetch_results(env.sim, True)
            env.gym.refresh_actor_root_state_tensor(env.sim)
            env.gym.refresh_net_contact_force_tensor(env.sim)
            peak = max(peak, contacts[0, building_body].norm().item())
        assert peak > .1, 'No physical drone contact with '+structure['kind']
        peaks[structure['kind']] = peak
    assert torch.allclose(env.env_asset_root_states, before, atol=1e-5)
    print('PASS: 2 buildings/env, fixed poses after partial reset, finite drone states, measured building contact peaks (N):', peaks)

if __name__ == '__main__':
    args = parse_args()
    args.headless = True
    env, _ = task_registry.make_env('arena2027', args=args)
    try:
        check(env)
    finally:
        env.gym.destroy_sim(env.sim)

"""GPU integration checks for physical impact, vector resets and recovery rules."""
import isaacgym
import torch
import numpy as np
from rlPx4Controller.pyParallelControl import ParallelRateControl
from airgym.envs import *
from airgym.utils.helpers import get_args
from airgym.utils import task_registry


def put(env, ids, positions, velocities=None):
    env.root_states[ids,:3] = positions
    env.root_states[ids,7:13] = 0
    if velocities is not None:
        env.root_states[ids,7:10] = velocities
    env.root_states[ids,3:7] = 0
    env.root_states[ids,6] = 1
    actors = env.actor_ids[ids].contiguous()
    from isaacgym import gymtorch
    env.gym.set_actor_root_state_tensor_indexed(env.sim,env.root_tensor,gymtorch.unwrap_tensor(actors),len(actors))


def check(env):
    controller=ParallelRateControl(4)
    fresh=ParallelRateControl(4)
    quaternion=np.tile(np.array([1.,0.,0.,0.]),(4,1))
    controller.set_q_world(quaternion);fresh.set_q_world(quaternion)
    command=np.zeros((4,4));command[:,0]=0.05;command[:,3]=0.2
    angular=np.zeros((4,3))
    for _ in range(50):controller.update(command,angular,0.01)
    controller.reset_idx([0])
    command[:,0]=0
    reset_output=controller.update(command,angular,0.01)
    fresh_output=fresh.update(command,angular,0.01)
    assert np.allclose(reset_output[0],fresh_output[0]), 'Selected PID state not cleared'
    assert not np.allclose(reset_output[1],fresh_output[1]), 'Unselected PID state was cleared'
    print('PASS selected controller integral reset; other controllers unchanged')
    assert env.num_envs >= 4
    env.settings.warm_start_probability = 0
    env.reset()
    buildings = env.env_asset_root_states.clone()
    # Reset only one environment; all other root states and buildings survive.
    others = env.root_states[1:].clone()
    env.reset_idx(env.rows[:1])
    assert torch.equal(env.root_states[1:],others)
    assert torch.equal(buildings,env.env_asset_root_states)
    action = torch.zeros(env.num_envs,4,device=env.device)
    action[:,-1] = -0.6934
    assert env.compute_observations().shape == (env.num_envs,33)
    # Mere proximity, with no force, is not a hit.
    put(env,env.rows,env.target_positions+0.6*env.normals)
    env.step(action)
    assert not env.hit.any()
    results = []
    # Both target actors must register real contacts at measured speed near 5.
    for target_id in range(2):
        env.reset()
        env.target_ids[:] = target_id
        env.target_positions[:] = env.plate_centers[target_id]
        env.normals[:] = env.plate_normals[target_id]
        env.recovery_positions[:] = env.target_positions+env.settings.recovery_standoff*env.normals
        put(env,env.rows,env.target_positions+0.5*env.normals,-5*env.normals)
        hit_info = None
        for _ in range(12):
            _,_,_,done,info=env.step(action)
            if info['episode_metrics']['hit'].any():
                hit_info=info['episode_metrics']
                assert not done[hit_info['hit']].any(), 'Impact incorrectly ended episode'
                break
        assert hit_info is not None, 'No physical contact for target '+str(target_id)
        good=hit_info['hit']
        assert hit_info['valid_hit'][good].all(), hit_info
        assert ((hit_info['impact_speed'][good]-5).abs()<0.5).all()
        rebound=(env.root_linvels[good]*env.normals[good]).sum(-1)
        assert (rebound>0).all(), 'Contact did not physically reverse normal velocity'
        results.append({'target_id':target_id,'speed':hit_info['impact_speed'][good].tolist(),'rebound_speed':rebound.tolist(),'force':hit_info['peak_plate_force'][good].tolist()})
    # Deterministic deadline checks are setup states, not policy performance.
    env.reset()
    env.hit[:] = True
    env.valid_hit[:] = True
    env.impact_step[:] = 0
    env.progress_buf[:] = env.recovery_steps-2
    env.stable_steps[:] = env.hold_steps
    put(env,env.rows,env.recovery_positions)
    _,_,_,done,_ = env.step(action)
    assert not done.any(), 'Ended before 2 second deadline'
    _,_,_,done,info=env.step(action)
    assert done.all() and info['episode_metrics']['success'].all()
    assert torch.allclose(info['episode_metrics']['post_impact_seconds'],torch.full_like(env.impact_speed,2),atol=1e-5)
    assert torch.isfinite(env.obs_buf).all()
    assert torch.equal(buildings,env.env_asset_root_states)
    # Low-speed contact cannot count as task success even after recovery.
    env.hit[:] = True
    env.valid_hit[:] = False
    env.impact_step[:] = 0
    env.progress_buf[:] = env.recovery_steps-1
    env.stable_steps[:] = env.hold_steps
    put(env,env.rows,env.recovery_positions)
    _,_,_,done,info=env.step(action)
    assert done.all() and not info['episode_metrics']['success'].any()
    # Latest permitted contact still has a complete 2 second recovery window.
    env.hit[:] = True
    env.valid_hit[:] = True
    env.impact_step[:] = env.max_episode_length-env.recovery_steps
    env.progress_buf[:] = env.max_episode_length-2
    env.stable_steps[:] = env.hold_steps
    put(env,env.rows,env.recovery_positions)
    _,_,_,done,_=env.step(action)
    assert not done.any()
    _,_,_,done,info=env.step(action)
    assert done.all() and info['episode_metrics']['success'].all()
    # A no-hit approach deadline is a true terminal task failure.
    env.reset()
    env.progress_buf[:] = env.max_episode_length-env.recovery_steps-1
    put(env,env.rows,env.recovery_positions)
    _,_,_,done,info=env.step(action)
    assert done.all() and info['episode_metrics']['timeout'].all()
    assert not info['time_outs'].any()
    # A ground crash after impact terminates as failure.
    env.hit[:] = True
    env.valid_hit[:] = True
    env.progress_buf.zero_()
    env.impact_step.zero_()
    pos=env.recovery_positions.clone()
    pos[:,2]=0.22
    put(env,env.rows,pos)
    _,_,_,done,info=env.step(action)
    assert done.all() and info['episode_metrics']['crash'].all()
    assert not info['episode_metrics']['success'].any()
    print('PASS physical contacts:',results)
    print('PASS vector partial reset, force-required hit, full 2 s deadline, speed qualification, crash failure, reset observations')


if __name__=='__main__':
    args=get_args()
    args.task='armor_impact'
    env,_=task_registry.make_env('armor_impact',args=args)
    try:
        check(env)
    finally:
        if env.viewer:
            env.gym.destroy_viewer(env.viewer)
        env.gym.destroy_sim(env.sim)

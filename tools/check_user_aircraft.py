"""GPU checks for imported CAD, physical mass, propulsion and CTBR signs."""
import json
from pathlib import Path
import isaacgym
from isaacgym import gymtorch
import numpy as np
import torch
import pytorch3d.transforms as T
from airgym.envs import *
from airgym.utils import task_registry
from airgym.utils.helpers import get_args
from rlPx4Controller.pyParallelControl import ParallelRateControl


def place(env, angular=None):
    env.reset()
    env.root_states[:,:3]=env.target_positions+2*env.normals
    env.root_states[:,3:7]=0;env.root_states[:,6]=1
    env.root_states[:,7:13]=0
    if angular is not None:env.root_states[:,10:13]=angular
    env._check_actor_ids=env.actor_ids.contiguous()
    env.gym.set_actor_root_state_tensor_indexed(env.sim,env.root_tensor,
        gymtorch.unwrap_tensor(env._check_actor_ids),env.num_envs)


def body_rate(env):
    r=T.quaternion_to_matrix(env.root_quats[:,[3,0,1,2]])
    return (r.transpose(1,2)@env.root_angvels.unsqueeze(-1)).squeeze(-1)


def main():
    args=get_args();args.task='armor_impact'
    env,_=task_registry.make_env('armor_impact',args=args)
    try:
        actor=env.gym.get_actor_handle(env.envs[0],0)
        names=env.gym.get_actor_rigid_body_names(env.envs[0],actor)
        assert names==['base_link','prop_1','prop_2','prop_3','prop_4'],names
        props=env.gym.get_actor_rigid_body_properties(env.envs[0],actor)
        mass=sum(p.mass for p in props)
        assert abs(mass-.160)<1e-6,mass
        shape_count=len(env.gym.get_actor_rigid_shape_properties(env.envs[0],actor))
        assert shape_count>20,'Imported frame unexpectedly reduced to a simple enclosing shape'
        # Batch behavior must equal independent controllers under different
        # attitudes and rates, including after a selected controller reset.
        rng=np.random.RandomState(3);q=rng.normal(size=(4,4));q/=np.linalg.norm(q,axis=1,keepdims=True)
        cmd=rng.normal(size=(4,4))*.03;cmd[:,3]=.3;omega=rng.normal(size=(4,3))*.1
        batch=ParallelRateControl(4);singles=[ParallelRateControl(1) for _ in range(4)]
        gains=[np.array(getattr(env.cfg.actuation,'rate_pid_'+s),dtype=np.float64) for s in ['p','i','d']]
        batch.set_pid_params(*gains);batch.set_q_world(q)
        for i,c in enumerate(singles):c.set_pid_params(*gains);c.set_q_world(q[i:i+1])
        for _ in range(5):
            got=np.array(batch.update(cmd,omega,.01))
            expected=np.concatenate([c.update(cmd[i:i+1],omega[i:i+1],.01) for i,c in enumerate(singles)])
            assert np.allclose(got,expected,atol=1e-12),'Vector controller differs from independent controllers'
        batch.reset_idx([2]);singles[2].reset_idx([0]);batch.set_q_world(q)
        singles[2].set_q_world(q[2:3])
        got=np.array(batch.update(cmd,omega,.01))
        expected=np.concatenate([c.update(cmd[i:i+1],omega[i:i+1],.01) for i,c in enumerate(singles)])
        assert np.allclose(got,expected,atol=1e-12),'Configured gains did not survive indexed reset'
        action=torch.zeros(env.num_envs,4,device=env.device)
        place(env)
        env.pre_physics_step(action.clone())
        assert torch.allclose(env.thrusts[:,:,2],torch.full_like(env.thrusts[:,:,2],.066*9.80665),atol=1e-6)
        action[:,-1]=1
        env.pre_physics_step(action.clone())
        assert torch.allclose(env.thrusts[:,:,2],torch.full_like(env.thrusts[:,:,2],.127*9.80665),atol=1e-6)
        hover_command=mass*abs(env.cfg.sim.gravity[2])/4/(2*env.cfg.actuation.half_command_thrust_n)
        action[:]=0;action[:,-1]=2*hover_command-1
        place(env);start=env.root_positions.clone()
        for step in range(100):
            _,_,_,done,info=env.step(action)
            assert not done.any(),('Free-flight hover terminated',step,
                info['terminal_observation'][done].tolist(),
                {k:v[done].tolist() for k,v in info['episode_metrics'].items()})
        height_error=(env.root_positions[:,2]-start[:,2]).abs().max().item()
        hover_omega=body_rate(env).norm(dim=-1).max().item()
        assert height_error<.2,(height_error,hover_omega)
        axes=[]
        for axis in range(3):
            place(env);command=action.clone();command[:,axis]=.5/env.settings.max_body_rate
            for _ in range(20):
                _,_,_,done,_=env.step(command)
                assert not done.any(),'Rate step terminated'
            rate=body_rate(env).mean(0)
            assert rate[axis]>.1,(axis,rate)
            axes.append(rate.tolist())
        # This tests the same low-level rate loop used throughout an episode;
        # no attitude recovery policy is inserted at a contact event.
        place(env,torch.tensor([0.,0.,2.],device=env.device))
        for _ in range(100):
            _,_,_,done,_=env.step(action)
            assert not done.any(),'Rate damping test terminated'
        yaw_after=body_rate(env)[:,2].abs().max().item()
        assert yaw_after<.2,yaw_after
        # Require actual contact to perturb angular momentum; this records the
        # outcome of flight forces + solver contact, without a scripted impulse.
        place(env)
        env.root_states[:,:3]=env.target_positions+.5*env.normals
        env.root_states[:,7:10]=-5*env.normals
        env.gym.set_actor_root_state_tensor_indexed(env.sim,env.root_tensor,
            gymtorch.unwrap_tensor(env._check_actor_ids),env.num_envs)
        contact_sample=None
        for _ in range(15):
            before=env.root_angvels.clone()
            _,_,_,done,info=env.step(action)
            m=info['episode_metrics'];hit=m['hit']
            if hit.any():
                after=info['terminal_observation'][:,15:18]
                change=(after-before).norm(dim=-1)
                assert (change[hit]>.01).all(),change
                contact_sample={'hit_speed_m_s':m['impact_speed'][hit].tolist(),
                    'angular_velocity_change_rad_s':change[hit].tolist(),
                    'angular_speed_after_rad_s':after[hit].norm(dim=-1).tolist()}
                break
        assert contact_sample is not None,'No CAD frame contact'
        report={'profile':'user160_cad_v1','loaded_mass_kg':mass,'rigid_body_names':names,
            'convex_collision_shapes':shape_count,'half_command_thrust_n':.066*9.80665,
            'full_command_thrust_n':.127*9.80665,'static_hover_motor_command':hover_command,
            'one_second_height_error_m':height_error,'one_second_body_rate_norm':hover_omega,
            'positive_rate_step_body_rates':axes,'yaw_rate_after_one_second_from_2rad_s':yaw_after,
            'physical_contact_angular_response':contact_sample,
            'note':'Dynamics checks only; component masses, inertia and rate gains are estimates, no trained recovery policy evaluated'}
        path=Path('docs/user_aircraft_dynamics_validation.json');path.write_text(json.dumps(report,indent=2)+'\n')
        print('PASS user160 mass, thrust points, free flight, positive CTBR axes and rate damping')
        print(json.dumps(report,indent=2))
    finally:
        if env.viewer:env.gym.destroy_viewer(env.viewer)
        env.gym.destroy_sim(env.sim)


if __name__=='__main__':main()

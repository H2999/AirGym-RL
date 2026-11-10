"""Static Isaac Gym preview of the imported aircraft; no policy is needed.

Run without arguments for an interactive viewer. --headless checks loading.
"""
import sys
import time
import numpy as np
import isaacgym
from isaacgym import gymapi
import torch
from airgym.envs import *
from airgym.utils import task_registry
from airgym.utils.helpers import get_args


def main():
    scene='--scene' in sys.argv
    if scene:sys.argv.remove('--scene')
    if not any(a=='--ctl_mode' or a.startswith('--ctl_mode=') for a in sys.argv):
        sys.argv.extend(['--ctl_mode','rate'])
    args=get_args();args.task='armor_impact';args.num_envs=1
    env,_=task_registry.make_env('armor_impact',args=args)
    try:
        # Consume the initial indexed reset once, then pause physics. The user
        # can inspect geometry without an untrained aircraft falling away.
        command=torch.zeros(1,4,device=env.device)
        command[:,-1]=2*(.160*9.81/4/(2*env.cfg.actuation.half_command_thrust_n))-1
        env.step(command)
        print('Loaded user160 CAD aircraft. Physics paused for geometry inspection.')
        if args.headless:
            print('Headless model-loading check passed.')
            return
        center=env.root_positions[0].cpu().numpy()
        if scene:
            target=env.target_positions[0].cpu().numpy()
            normal=env.normals[0].cpu().numpy()
            side=np.array([-normal[1],normal[0],0.])
            eye=target+3.2*normal+2.2*side+np.array([0.,0.,1.6])
            center=target+.6*normal
        else:
            eye=center+[.28,-.32,.20]
        env.gym.viewer_camera_look_at(env.viewer,None,gymapi.Vec3(*eye),gymapi.Vec3(*center))
        print('Close the window or press Esc to exit.')
        while not env.gym.query_viewer_has_closed(env.viewer):
            if any(e.action=='QUIT' and e.value>0 for e in env.gym.query_viewer_action_events(env.viewer)):
                break
            env.gym.step_graphics(env.sim)
            env.gym.draw_viewer(env.viewer,env.sim,True)
            time.sleep(1/60)
    finally:
        if env.viewer:env.gym.destroy_viewer(env.viewer)
        env.gym.destroy_sim(env.sim)


if __name__=='__main__':main()

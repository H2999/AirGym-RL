"""Physical armor contact, speed-qualified impact and closed-loop recovery.

Observations use simulator state (as Balloon does), not a vision detector.
No scripted impulse, teleport or controller takeover is applied at impact.
"""
import math
import numpy as np
import torch
import pytorch3d.transforms as T
from isaacgym import gymapi, gymtorch
from airgym.envs.base.hovering import Hovering
from .armor_impact_config import TARGETS


class ArmorImpact(Hovering):
    def __init__(self, cfg, *args, **kwargs):
        if cfg.env.ctl_mode != 'rate':
            raise ValueError('armor_impact uses CTBR: --ctl_mode rate')
        super().__init__(cfg, *args, **kwargs)
        if not hasattr(self.parallel_rate_control, 'reset_idx'):
            raise RuntimeError('Run: python tools/patch_controller_reset.py (inside airgym_dev)')
        self.settings = cfg.impact
        self.rows = torch.arange(self.num_envs, device=self.device)
        centers, normals, half_sizes = [], [], []
        for target in TARGETS:
            bounds = np.asarray(target['panel']['local_bounds'])
            local = bounds.mean(axis=0)
            # Front face of this specific armor mesh, not the building center.
            local[0] = bounds[1, 0]
            yaw = target['yaw']
            c, s = math.cos(yaw), math.sin(yaw)
            rot = np.array([[c,-s,0], [s,c,0], [0,0,1]])
            centers.append(rot @ local + target['position'])
            normals.append([c, s, 0])
            half_sizes.append((bounds[1] - bounds[0])[1:] / 2)
        self.plate_centers = torch.tensor(np.asarray(centers), dtype=torch.float32, device=self.device)
        self.plate_normals = torch.tensor(normals, dtype=torch.float32, device=self.device)
        self.plate_half_sizes = torch.tensor(np.asarray(half_sizes), dtype=torch.float32, device=self.device)
        robot_bodies = self.asset_manager.get_robot_num_bodies()
        self.robot_bodies = robot_bodies
        self.plate_body_ids = torch.tensor([robot_bodies+2*i+1 for i in range(len(TARGETS))], device=self.device)
        self.contacts = gymtorch.wrap_tensor(self.gym.acquire_net_contact_force_tensor(self.sim)).view(self.num_envs, -1, 3)
        self.target_ids = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.target_positions = torch.zeros(self.num_envs, 3, device=self.device)
        self.normals = torch.zeros_like(self.target_positions)
        self.hit = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.valid_hit = self.hit.clone()
        self.impact_step = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.stable_steps = self.impact_step.clone()
        self.impact_speed = torch.zeros(self.num_envs, device=self.device)
        self.peak_force = self.impact_speed.clone()
        self.initial_distance = torch.ones(self.num_envs, device=self.device)
        self.total_completed = self.total_hits = self.total_valid_hits = self.total_successes = 0
        self.recovery_steps = int(round(self.settings.recovery_seconds / self.dt))
        self.hold_steps = int(math.ceil(self.settings.stable_hold_seconds / self.dt))
        self.actor_ids = (self.rows * self.vec_root_tensor.shape[1]).to(torch.int32)
        self.reset_idx(self.rows)
        self.compute_observations()

    def create_sim(self):
        if self.headless:
            # This task has no cameras: do not allocate a Vulkan renderer.
            self.graphics_device_id = -1
        super().create_sim()

    def _create_envs(self):
        # Flat floor intentionally avoids global triangle meshes overlapping
        # neighboring vector environments. Structures retain source elevations.
        super()._create_envs()
        rng = np.random.RandomState(self.cfg.seed if self.cfg.seed >= 0 else 42)
        plate_actors = {2+2*i for i in range(len(TARGETS))}
        for env in self.envs:
            restitution_sample, friction_sample = rng.uniform(size=2)
            for actor in range(self.gym.get_actor_count(env)):
                material = self.cfg.steel_plate if actor in plate_actors else self.cfg.impact
                restitution = material.restitution_range[0]+restitution_sample*(material.restitution_range[1]-material.restitution_range[0])
                friction = material.friction_range[0]+friction_sample*(material.friction_range[1]-material.friction_range[0])
                props = self.gym.get_actor_rigid_shape_properties(env, actor)
                for shape in props:
                    shape.restitution = restitution
                    shape.friction = friction
                self.gym.set_actor_rigid_shape_properties(env, actor, props)
                if actor in plate_actors:
                    self.gym.set_rigid_body_color(env,actor,0,gymapi.MESH_VISUAL,
                        gymapi.Vec3(*self.cfg.steel_plate.color))

    def reset_idx(self, env_ids):
        if len(env_ids) == 0:
            return
        n = len(env_ids)
        self.target_ids[env_ids] = torch.randint(len(TARGETS), (n,), device=self.device)
        self.target_positions[env_ids] = self.plate_centers[self.target_ids[env_ids]]
        self.normals[env_ids] = self.plate_normals[self.target_ids[env_ids]]
        normal = self.normals[env_ids]
        tangent = torch.stack((-normal[:,1], normal[:,0], normal[:,2]), -1)
        distance = self._uniform(self.settings.spawn_distance, n)
        warm = torch.rand(n, device=self.device) < self.settings.warm_start_probability
        distance = torch.where(warm, self._uniform(self.settings.warm_start_distance, n), distance)
        pos = self.target_positions[env_ids] + distance[:,None]*normal
        pos += (torch.rand(n,1,device=self.device)*2-1) * self.settings.spawn_lateral_noise * tangent
        pos[:,2] += (torch.rand(n,device=self.device)*2-1) * self.settings.spawn_vertical_noise
        self.root_states[env_ids] = 0
        self.root_states[env_ids,:3] = pos
        yaw = torch.atan2(-normal[:,1], -normal[:,0])
        self.root_states[env_ids,5] = torch.sin(yaw/2)
        self.root_states[env_ids,6] = torch.cos(yaw/2)
        self.root_states[env_ids,7:10] = -normal * (warm.float()*self.settings.speed)[:,None]
        self.initial_distance[env_ids] = distance
        self.hit[env_ids] = False
        self.valid_hit[env_ids] = False
        self.impact_speed[env_ids] = 0
        self.impact_step[env_ids] = 0
        self.stable_steps[env_ids] = 0
        self.peak_force[env_ids] = 0
        self.progress_buf[env_ids] = 0
        self.reset_buf[env_ids] = 0
        self.pre_actions[env_ids] = 0
        self.parallel_rate_control.reset_idx(env_ids.cpu().tolist())
        if self.viewer and self.num_envs == 1:
            center = self.target_positions[0].cpu().numpy()
            outward = self.normals[0].cpu().numpy()
            side = np.array([-outward[1], outward[0], 0])
            eye = center + 3*outward + 3*side + np.array([0, 0, 1.5])
            look = center + 0.8*outward
            self.gym.viewer_camera_look_at(self.viewer, None, gymapi.Vec3(*eye), gymapi.Vec3(*look))
        # Gym defers consumption of index tensors until simulate(). Keep the
        # backing allocation alive across return and policy inference.
        self._reset_actor_ids = self.actor_ids[env_ids].contiguous()
        self.gym.set_actor_root_state_tensor_indexed(self.sim, self.root_tensor, gymtorch.unwrap_tensor(self._reset_actor_ids), len(self._reset_actor_ids))

    def _uniform(self, interval, n):
        return interval[0] + torch.rand(n, device=self.device)*(interval[1]-interval[0])

    def reset(self):
        self.reset_idx(self.rows)
        self.compute_observations()
        return self.obs_buf, self.privileged_obs_buf

    def compute_observations(self):
        rotation = T.quaternion_to_matrix(self.root_quats[:,[3,0,1,2]])
        o = self.obs_buf
        o[:,0:9] = rotation.reshape(self.num_envs, 9)
        o[:,9:12] = self.target_positions - self.root_positions
        o[:,12:15] = self.root_linvels
        o[:,15:18] = self.root_angvels
        o[:,18:21] = self.normals
        o[:,21] = self.hit.float()
        o[:,22] = torch.where(self.hit, ((self.progress_buf-self.impact_step)*self.dt/self.settings.recovery_seconds).clamp(0,1), 0)
        o[:,23] = self.settings.speed
        o[:,24:28] = self.pre_actions
        o[:,28] = self.valid_hit.float()
        o[:,29] = self.stable_steps.float()/self.hold_steps
        return o

    def step(self, actions):
        # Parent mutates the thrust component in place: protect PPO's action tensor.
        before_pos = self.root_positions.clone()
        before_vel = self.root_linvels.clone()
        old_hit = self.hit.clone()
        previous_actions = self.pre_actions.clone()
        self.reset_buf.zero_()
        commands = actions.clone()
        commands[:,:3] *= self.settings.max_body_rate
        self.pre_physics_step(commands)
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_net_contact_force_tensor(self.sim)
        self.progress_buf += 1
        plate_force = self.contacts[self.rows, self.plate_body_ids[self.target_ids]].norm(dim=-1)
        self.peak_force = torch.maximum(self.peak_force, plate_force)
        relative = self.root_positions - self.target_positions
        before_relative = before_pos - self.target_positions
        normal_distance = (before_relative*self.normals).sum(-1)
        tangent = torch.stack((-self.normals[:,1], self.normals[:,0], self.normals[:,2]), -1)
        lateral = (relative*tangent).sum(-1).abs()
        half = self.plate_half_sizes[self.target_ids]
        corridor = (lateral < half[:,0]+self.settings.contact_center_margin) & (relative[:,2].abs() < half[:,1]+self.settings.contact_center_margin)
        approach_speed = -(before_vel*self.normals).sum(-1)
        new_hit = ~old_hit & (plate_force > self.settings.contact_force_threshold) & corridor & (normal_distance>0) & (approach_speed>0.2)
        tangential_speed = (before_vel + approach_speed[:,None]*self.normals).norm(dim=-1)
        self.hit |= new_hit
        self.impact_step[new_hit] = self.progress_buf[new_hit]
        self.impact_speed[new_hit] = approach_speed[new_hit]
        qualified = (approach_speed-self.settings.speed).abs() <= self.settings.speed_tolerance
        qualified &= tangential_speed <= self.settings.max_tangential_speed
        self.valid_hit[new_hit] = qualified[new_hit]
        rotation = T.quaternion_to_matrix(self.root_quats[:,[3,0,1,2]])
        up = rotation[:,2,2]
        speed = self.root_linvels.norm(dim=-1)
        angular_speed = self.root_angvels.norm(dim=-1)
        stable = self.hit & (up > math.cos(math.radians(self.settings.max_tilt_deg)))
        stable &= angular_speed < self.settings.max_recovery_angvel
        if self.settings.require_low_speed:
            stable &= speed < self.settings.max_recovery_speed
        self.stable_steps = torch.where(stable, self.stable_steps+1, torch.zeros_like(self.stable_steps))
        elapsed = self.progress_buf - self.impact_step
        finished = self.hit & (elapsed >= self.recovery_steps)
        finite = torch.isfinite(self.root_states).all(-1)
        crash = ~finite | (self.root_positions[:,2] < self.settings.min_height) | (self.root_positions[:,2] > self.settings.max_height)
        crash |= angular_speed > self.settings.max_angular_speed
        crash |= relative.norm(dim=-1) > self.settings.max_target_distance
        # Unexpected structure/ground contact before the intended plate is failure.
        robot_contact = self.contacts[:,:self.robot_bodies].norm(dim=-1).amax(-1) > self.settings.contact_force_threshold
        crash |= robot_contact & ~self.hit
        # Crossing behind the plate without registered physical contact is a miss.
        miss = ~self.hit & ((relative*self.normals).sum(-1) < -0.3)
        success = finished & self.valid_hit & (self.stable_steps >= self.hold_steps) & ~crash
        # Reserve the entire recovery window even for a late valid contact.
        timeout = ~self.hit & (self.progress_buf >= self.max_episode_length-self.recovery_steps) & ~crash & ~miss
        done = crash | miss | finished | timeout
        distance_before = before_relative.norm(dim=-1)
        distance = relative.norm(dim=-1)
        alignment = torch.exp(-8*(lateral.square()+relative[:,2].square()))
        speed_tracking = torch.exp(-((approach_speed-self.settings.speed)/1.2).square())
        # Progress is paid once per traveled distance; no unlimited hover reward.
        forward_progress = ((before_pos-self.root_positions)*self.normals).sum(-1)
        approach_reward = 10*(distance_before-distance) + 5*forward_progress
        approach_reward += 20*forward_progress.clamp(min=0)*speed_tracking*alignment
        # A smooth penalty supplies an acceleration gradient even at zero
        # speed, where the Gaussian speed bonus alone is almost flat.
        approach_reward -= self.dt*(0.1*(approach_speed-self.settings.speed).square()
                                    + 0.5*(lateral.square()+relative[:,2].square()))
        # Score the resulting attitude, not a prescribed recovery trajectory or
        # location. The policy may retreat, brake or keep flying as it learns.
        recovery_reward = self.dt*(
            4*up.clamp(0,1).square()
            + 3*torch.exp(-(angular_speed/self.settings.max_recovery_angvel).square())
            + torch.exp(-self.root_linvels[:,2].square())
            - 0.2*speed.square())
        reward = torch.where(old_hit, recovery_reward, approach_reward)
        reward += new_hit.float()*(10 + 40*qualified.float())
        reward += 150*success.float() - 60*crash.float() - 20*miss.float() - 20*timeout.float()
        reward -= 0.01*(self.actions-previous_actions).square().sum(-1)
        # Invalid-speed hits get recovery experience but no task-success bonus.
        reward -= finished.float()*(~success).float()*20
        self.rew_buf[:] = torch.nan_to_num(reward, nan=-60.0, posinf=-60.0, neginf=-60.0)
        self.pre_actions[:] = self.actions
        self.compute_observations()
        terminal_obs = self.obs_buf.clone()
        ids = done.nonzero(as_tuple=False).flatten()
        episode = {
            'hit': self.hit.clone(), 'valid_hit': self.valid_hit.clone(),
            'impact_speed': self.impact_speed.clone(), 'success': success.clone(),
            'crash': crash.clone(), 'timeout': timeout.clone(),
            'peak_plate_force': self.peak_force.clone(),
            'post_impact_seconds': torch.where(self.hit, elapsed*self.dt, torch.zeros_like(speed)),
            'recovery_speed': speed.clone(), 'recovery_angvel': angular_speed.clone(),
            'upright_cos': up.clone(), 'target_id': self.target_ids.clone(),
            'stable_at_deadline': (self.stable_steps >= self.hold_steps).clone(),
        }
        self.total_completed += len(ids)
        self.total_hits += int(self.hit[ids].sum())
        self.total_valid_hits += int(self.valid_hit[ids].sum())
        self.total_successes += int(success[ids].sum())
        extras = {
            # Missing the task deadline is terminal failure, not a bootstrapped
            # truncation of an otherwise continuing episode.
            'time_outs': torch.zeros_like(timeout), 'terminal_observation': terminal_obs,
            'episode_metrics': episode,
            'item_reward_info': {'reward': self.rew_buf.clone(), 'hit_event': new_hit.float(), 'success_event': success.float()},
            'impact/completed_episodes': self.total_completed,
            'impact/hit_rate': self.total_hits/max(1,self.total_completed),
            'impact/valid_hit_rate': self.total_valid_hits/max(1,self.total_completed),
            'impact/success_rate': self.total_successes/max(1,self.total_completed),
        }
        self.reset_idx(ids)
        self.compute_observations()  # reset observations and terminal dones must agree
        self.reset_buf[:] = done.long()
        if self.viewer:
            self.render(sync_frame_time=True)
        return self.obs_buf, self.privileged_obs_buf, self.rew_buf, self.reset_buf, extras

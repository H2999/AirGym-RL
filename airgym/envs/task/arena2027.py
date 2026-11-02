"""AirGym flight environment containing static 2027 bases and outposts."""
import math
import numpy as np
import torch
from isaacgym import gymapi
from airgym.envs.base.hovering import Hovering
from .arena2027_config import STRUCTURES, LAYOUT

class Arena2027(Hovering):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if self.ctl_mode == 'pos':
            self.action_lower_limits = torch.tensor([-14, -7.5, 0, -6], device=self.device)
            self.action_upper_limits = torch.tensor([14, 7.5, 6, 6], device=self.device)

    def _create_envs(self):
        super()._create_envs()
        for env in self.envs:
            # Source map has a 1.5 degree symmetric floor slope. Keep the
            # source elevations without leaving the structures floating.
            origin = self.gym.get_env_origin(env)
            w, h = LAYOUT['arena_size']
            ridge = h / 2 * math.tan(math.radians(LAYOUT['floor_slope_deg']))
            vertices = np.array([[-w/2,-h/2,0],[w/2,-h/2,0],
                                 [-w/2,0,ridge],[w/2,0,ridge],
                                 [-w/2,h/2,0],[w/2,h/2,0]], dtype=np.float32)
            triangles = np.array([[0,1,2],[1,3,2],[2,3,4],[3,5,4]], dtype=np.uint32)
            params = gymapi.TriangleMeshParams()
            params.nb_vertices, params.nb_triangles = 6, 4
            params.transform.p = origin
            self.gym.add_triangle_mesh(self.sim, vertices.flatten(), triangles.flatten(), params)

    def reset_idx(self, env_ids):
        # Fixed buildings keep their initial transforms across resets.
        self.root_states[env_ids] = 0
        self.root_states[env_ids, :3] = torch.tensor(self.cfg.env.spawn_position, device=self.device)
        self.root_states[env_ids, 6] = 1
        self.gym.set_actor_root_state_tensor(self.sim, self.root_tensor)
        self.reset_buf[env_ids] = 0
        self.progress_buf[env_ids] = 0
        self.pre_actions[env_ids] = 0

    def compute_quadcopter_reward(self):
        # Basic stable hover objective; no game rules or damage model are implied.
        error = torch.linalg.vector_norm(self.root_positions - torch.tensor(self.cfg.env.spawn_position, device=self.device), dim=-1)
        reward = 1 / (1 + error.square())
        reset = (self.progress_buf >= self.max_episode_length - 1).long()
        return reward, reset, {'position_reward': reward}

"""Geometric reference used ONLY for optional imitation warm-start data.

Neither ArmorImpact.step nor evaluation calls this controller. A trained policy
must issue every CTBR command itself, including the post-contact recovery.
"""
import torch
import torch.nn.functional as F
import pytorch3d.transforms as T


def reference_action(env):
    rotation = T.quaternion_to_matrix(env.root_quats[:, [3, 0, 1, 2]])
    delta = env.target_positions - env.root_positions
    normal = env.normals
    lateral = delta - (delta * normal).sum(-1, keepdim=True) * normal
    # Compensate finite attitude response; measured acceptance remains 5 +/-0.5.
    approach_velocity = -normal * (0.92 * env.settings.speed) + 3 * lateral
    recovery_velocity = (0.5 * (env.recovery_positions - env.root_positions)).clamp(-1, 1)
    velocity = torch.where(env.hit[:, None], recovery_velocity, approach_velocity)
    gain = torch.where(env.hit, torch.full_like(env.impact_speed, 2.5),
                       torch.full_like(env.impact_speed, 3.0))
    acceleration = (gain[:, None] * (velocity - env.root_linvels)).clamp(-12, 12)
    acceleration[:, 2] = (6 * (env.target_positions[:, 2] - env.root_positions[:, 2])
                          - 4 * env.root_linvels[:, 2]).clamp(-6, 8)
    force_direction = acceleration.clone()
    force_direction[:, 2] += 9.81
    desired_z = F.normalize(force_direction, dim=-1)
    desired_y = F.normalize(torch.cross(desired_z, -normal, dim=-1), dim=-1)
    desired_x = torch.cross(desired_y, desired_z, dim=-1)
    desired_rotation = torch.stack([desired_x, desired_y, desired_z], dim=-1)
    error = 0.5 * (desired_rotation.transpose(1, 2) @ rotation
                   - rotation.transpose(1, 2) @ desired_rotation)
    error_vector = torch.stack([error[:, 2, 1], error[:, 0, 2], error[:, 1, 0]], dim=-1)
    body_rates = (-6 * error_vector).clamp(-3, 3) / env.settings.max_body_rate
    # Reference is calibrated only for the supplied 0.601 kg X152b and the
    # existing 4 * 9.59 N thrust model, not the user's uncharacterized aircraft.
    collective = (0.601 * (9.81 + acceleration[:, 2])
                  / (rotation[:, 2, 2].clamp(min=0.2) * 38.36)).clamp(0, 1)
    return torch.cat([body_rates, (2 * collective - 1)[:, None]], dim=-1)

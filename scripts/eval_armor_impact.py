"""Evaluate a checkpoint from hover; no training warm starts or controller takeover.

Omit --headless to watch. Writes per-episode measurements and aggregate rates.
"""
import argparse
import json
import hashlib
import sys
from pathlib import Path
import isaacgym
import torch
import yaml
from airgym.utils.helpers import get_args
from scripts.runner import update_config
from lib.torch_runner import Runner
from lib.agent.players import A2CPlayer


def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--episodes',type=int,default=100)
    parser.add_argument('--report',default='runs/armor_impact_evaluation.json')
    extra,remaining=parser.parse_known_args()
    sys.argv=[sys.argv[0]]+remaining
    args=vars(get_args())
    args['task']='armor_impact'
    if '--num_envs' not in remaining and not any(x.startswith('--num_envs=') for x in remaining):
        args['num_envs']=16
    if extra.episodes<=0:
        raise ValueError('--episodes must be positive')
    if not args['checkpoint'] or not Path(args['checkpoint']).is_file():
        raise FileNotFoundError('Pass an existing --checkpoint path')
    with open(Path(__file__).parent/'config/ppo_armor_impact.yaml') as f:
        config=update_config(yaml.safe_load(f),args)
    runner=Runner()
    runner.load(config)
    player=A2CPlayer(runner.params)
    raw=player.env.env.env
    records=[]
    try:
        # Strict evaluation: every trial starts close to stationary hover.
        raw.settings.warm_start_probability=0.0
        weights=torch.load(args['checkpoint'],map_location=player.device)
        player.model.load_state_dict(weights['model'],strict=True)
        saved_rate=weights.get('task_max_body_rate',raw.settings.max_body_rate)
        if saved_rate!=raw.settings.max_body_rate:
            raise ValueError('Checkpoint uses a different CTBR action scale')
        player.has_batch_dimension=True
        obs=player.env.reset()
        # A finite budget prevents a broken termination rule from hanging eval.
        budget=(extra.episodes//raw.num_envs+2)*(raw.max_episode_length+1)
        for _ in range(budget):
            action=player.get_action(obs,is_deterministic=True)
            obs,reward,done,info=player.env.step(action)
            for i in done.nonzero(as_tuple=False).flatten().tolist():
                m=info['episode_metrics']
                records.append({k:v[i].item() for k,v in m.items()})
                if len(records)>=extra.episodes:
                    break
            if len(records)>=extra.episodes:
                break
        if len(records)<extra.episodes:
            raise RuntimeError('Evaluation did not complete its requested episodes')
        hits=[r for r in records if r['hit']]
        valid=[r for r in records if r['valid_hit']]
        summary={
            'checkpoint':str(Path(args['checkpoint']).resolve()),
            'checkpoint_sha256':hashlib.sha256(Path(args['checkpoint']).read_bytes()).hexdigest(),
            'seed':args['seed'], 'episodes':len(records), 'warm_start_probability':0.0,
            'criteria': {k:getattr(raw.settings,k) for k in [
                'max_tangential_speed','stable_hold_seconds','max_tilt_deg',
                'max_recovery_speed','max_recovery_angvel','max_recovery_position_error',
                'recovery_standoff','min_height','max_height']},
            'required_speed_mps':raw.settings.speed,
            'speed_tolerance_mps':raw.settings.speed_tolerance,
            'recovery_deadline_seconds':raw.settings.recovery_seconds,
            'hit_rate':len(hits)/len(records), 'valid_hit_rate':len(valid)/len(records),
            'success_rate':sum(r['success'] for r in records)/len(records),
            'crash_rate':sum(r['crash'] for r in records)/len(records),
            'mean_hit_speed_mps':sum(r['impact_speed'] for r in hits)/max(1,len(hits)),
            'successful_episodes':sum(r['success'] for r in records),
            'normal_speed_min_mps':min((r['impact_speed'] for r in hits),default=None),
            'normal_speed_max_mps':max((r['impact_speed'] for r in hits),default=None),
            'by_target':{},
        }
        from airgym.envs.task.armor_impact_config import TARGETS
        for i,t in enumerate(TARGETS):
            group=[r for r in records if r['target_id']==i]
            summary['by_target'][t['kind']]={
                'episodes':len(group),
                'success_rate':sum(r['success'] for r in group)/max(1,len(group)),
            }
        report=Path(extra.report)
        report.parent.mkdir(parents=True,exist_ok=True)
        report.write_text(json.dumps({'summary':summary,'episodes':records},indent=2))
        print(json.dumps(summary,indent=2))
        print('Report:',report.resolve())
    finally:
        if raw.viewer:
            raw.gym.destroy_viewer(raw.viewer)
        raw.gym.destroy_sim(raw.sim)


if __name__=='__main__':
    main()

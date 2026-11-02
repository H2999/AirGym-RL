"""Train armor-impact PPO; --epochs overrides YAML without editing it."""
import argparse
import sys
from pathlib import Path
import isaacgym
import yaml
from airgym.utils.helpers import get_args
from scripts.runner import update_config
from lib.torch_runner import Runner
from lib.utils.isaacgym_utils import RLGPUAlgoObserver

if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--epochs',type=int,default=None)
    extra,remaining=parser.parse_known_args()
    sys.argv=[sys.argv[0]]+remaining
    args=vars(get_args())
    args['task']='armor_impact'
    if '--num_envs' not in remaining and not any(x.startswith('--num_envs=') for x in remaining):
        args['num_envs']=256
    if args['ctl_mode']!='rate':
        raise ValueError('Use --ctl_mode rate')
    if args['checkpoint'] and not Path(args['checkpoint']).is_file():
        raise FileNotFoundError(args['checkpoint'])
    with open(Path(__file__).parent/'config/ppo_armor_impact.yaml') as f:
        config=yaml.safe_load(f)
    config=update_config(config,args)
    if extra.epochs is not None:
        config['params']['config']['max_epochs']=extra.epochs
    r=config['params']['config']
    if args['checkpoint']:
        r['freeze_input_normalization']=True
    if r['num_actors']*r['horizon_length']%r['minibatch_size']:
        raise ValueError('num_envs * horizon_length must divide into whole minibatches; use 64 or 256 envs')
    runner=Runner(RLGPUAlgoObserver())
    runner.load(config)
    runner.run(args)

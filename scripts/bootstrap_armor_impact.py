"""Optional behavior cloning + DAgger initialization before PPO training."""
import argparse
import json
import sys
from pathlib import Path
import isaacgym
import torch
import yaml
from airgym.utils.helpers import get_args
from airgym.utils.armor_reference import reference_action
from scripts.runner import update_config
from lib.torch_runner import Runner
from lib.agent.players import A2CPlayer


def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--output',default='runs/armor_impact_bootstrap.pth')
    parser.add_argument('--collect_steps',type=int,default=800)
    parser.add_argument('--updates',type=int,default=2500)
    parser.add_argument('--rounds',type=int,default=3)
    extra,remaining=parser.parse_known_args()
    sys.argv=[sys.argv[0]]+remaining
    args=vars(get_args());args['task']='armor_impact'
    if '--num_envs' not in remaining and not any(x.startswith('--num_envs=') for x in remaining):
        args['num_envs']=64
    if min(extra.collect_steps,extra.updates,extra.rounds)<=0:
        raise ValueError('collection, update and round counts must be positive')
    with open(Path(__file__).parent/'config/ppo_armor_impact.yaml') as f:
        config=update_config(yaml.safe_load(f),args)
    runner=Runner();runner.load(config)
    player=A2CPlayer(runner.params);player.has_batch_dimension=True
    env=player.env.env.env
    model=player.model
    data_obs=[];data_actions=[];data_returns=[];round_info=[]
    try:
        if args['checkpoint']:
            if not Path(args['checkpoint']).is_file():
                raise FileNotFoundError(args['checkpoint'])
            # Allow migrating a shared actor/critic initialization to separate
            # networks without changing the actor or observation normalization.
            weights=torch.load(args['checkpoint'],map_location=env.device)
            missing,unexpected=model.load_state_dict(weights['model'],strict=False)
            if unexpected or any(not k.startswith('critic_mlp.') for k in missing):
                raise ValueError('Incompatible initialization: '+str((missing,unexpected)))
            if missing:
                model.critic_mlp.load_state_dict(model.actor_mlp.state_dict())
        env.settings.warm_start_probability=0.0
        optimizer=torch.optim.Adam(model.parameters(),lr=1e-3,eps=1e-8)
        for rnd in range(extra.rounds):
            obs=player.env.reset()
            beta=1.0 if rnd==0 else (0.5 if rnd==1 else 0.0)
            completed=hits=successes=0
            rollout_rewards=[];rollout_dones=[]
            for _ in range(extra.collect_steps):
                with torch.no_grad():
                    target=reference_action(env)
                    data_obs.append(obs.clone())
                    data_actions.append(target.clone())
                    if beta==1:
                        action=target
                    else:
                        predicted=player.get_action(obs,is_deterministic=True)
                        choose=torch.rand(env.num_envs,1,device=env.device)<beta
                        action=torch.where(choose,target,predicted)
                    obs,reward,done,info=player.env.step(action)
                    rollout_rewards.append(reward.clone()*config['params']['config']['reward_shaper']['scale_value'])
                    rollout_dones.append(done.clone())
                    ids=done.nonzero().flatten()
                    completed+=len(ids)
                    hits+=int(info['episode_metrics']['hit'][ids].sum())
                    successes+=int(info['episode_metrics']['success'][ids].sum())
            returns=torch.zeros(env.num_envs,device=env.device)
            round_returns=[]
            for reward,done in zip(reversed(rollout_rewards),reversed(rollout_dones)):
                returns=reward+config['params']['config']['gamma']*returns*(1-done.float())
                round_returns.append(returns.clone())
            data_returns.extend(reversed(round_returns))
            x=torch.cat(data_obs);y=torch.cat(data_actions)
            if rnd==0:
                with torch.no_grad():
                    model.running_mean_std.running_mean.copy_(x.mean(0))
                    model.running_mean_std.running_var.copy_(x.var(0).clamp(min=1e-4))
                    model.running_mean_std.count.fill_(len(x))
            model.eval() # fixed normalization while fitting the actor
            for update in range(extra.updates):
                idx=torch.randint(len(x),(min(2048,len(x)),),device=env.device)
                normalized=model.norm_obs(x[idx])
                prediction=model.mu(model.actor_mlp(normalized))
                loss=(prediction-y[idx]).square().mean()
                optimizer.zero_grad();loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(),2.0)
                optimizer.step()
            metrics={'round':rnd,'expert_probability':beta,'completed':completed,'hits':hits,'successes':successes,'mse':float(loss),'samples':len(x)}
            round_info.append(metrics)
            print(json.dumps(metrics),flush=True)
        # Fit the independent critic to discounted demonstration returns before
        # PPO, avoiding random initial advantages destroying the cloned actor.
        returns=torch.cat(data_returns)[:,None]
        with torch.no_grad():
            model.value_mean_std.running_mean.copy_(returns.mean(0))
            model.value_mean_std.running_var.copy_(returns.var(0).clamp(min=1e-4))
            model.value_mean_std.count.fill_(len(returns))
            target_values=(returns-model.value_mean_std.running_mean)/(model.value_mean_std.running_var+1e-5).sqrt()
        critic_optimizer=torch.optim.Adam(list(model.critic_mlp.parameters())+list(model.value_head.parameters()),lr=1e-3)
        for _ in range(extra.updates):
            idx=torch.randint(len(x),(min(2048,len(x)),),device=env.device)
            prediction=model.value_head(model.critic_mlp(model.norm_obs(x[idx])))
            value_loss=(prediction-target_values[idx]).square().mean()
            critic_optimizer.zero_grad();value_loss.backward();critic_optimizer.step()
        print('Critic prefit MSE:',float(value_loss),flush=True)
        with torch.no_grad():
            model.logstd.fill_(-3.0)
        # Reset optimizer momentum and PPO learning rate after imitation fitting.
        ppo_optimizer=torch.optim.Adam(model.parameters(),lr=config['params']['config']['learning_rate'],eps=1e-8)
        output=Path(extra.output);output.parent.mkdir(parents=True,exist_ok=True)
        torch.save({'model':model.state_dict(),'optimizer':ppo_optimizer.state_dict(),
                    'epoch':0,'frame':0,'last_mean_rewards':-1e9,'env_state':None,
                    'bootstrap':round_info,'task_max_body_rate':env.settings.max_body_rate},output)
        output.with_suffix('.json').write_text(json.dumps(round_info,indent=2))
        print('Saved neural policy initialization:',output.resolve())
    finally:
        if env.viewer:env.gym.destroy_viewer(env.viewer)
        env.gym.destroy_sim(env.sim)


if __name__=='__main__':
    main()

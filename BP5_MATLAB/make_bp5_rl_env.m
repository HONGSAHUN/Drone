function env=make_bp5_rl_env(P)
% Optional adapter. Requires Reinforcement Learning Toolbox.
if nargin<1, P=bp5_params(); end
assert(exist('rlFunctionEnv','file')~=0, ...
    'Reinforcement Learning Toolbox is required for this adapter. Core demo needs only MATLAB.');
obs=rlNumericSpec([23 1]); obs.Name='error_velocity_rotation_rates_motors_time';
act=rlNumericSpec([3 1],'LowerLimit',-ones(3,1),'UpperLimit',ones(3,1));
act.Name='residual_acceleration';
env=rlFunctionEnv(obs,act,@bp5_rl_step,@() bp5_rl_reset(P));
end

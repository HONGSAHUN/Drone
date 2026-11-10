function [obs,reward,done,info]=bp5_rl_step(action,info)
% Agent supplies bounded acceleration correction; classical inner loops remain.
action=action(:);
assert(numel(action)==3 && all(isfinite(action)),'Action must contain 3 finite values.');
action=max(-1,min(1,action));
ref=struct('p',info.target,'v',zeros(3,1),'a',zeros(3,1),'yaw',0);
u=bp5_control(info.s,ref,info.P,2*action); % up to +/-2 m/s^2 per axis
info.s=bp5_step(info.s,u,info.P,info.dt); info.step=info.step+1;
s=info.s; e=info.target-s.p;
reward=info.dt*(1-2*(e'*e)-.2*(s.v'*s.v)-.02*(action'*action));
failed=norm(e)>2 || s.R(3,3)<cos(pi/3) || s.p(3)<=info.P.groundHeight;
done=failed || info.step>=info.maxSteps;
if failed, reward=reward-10; end
obs=bp5_rl_observation(info);
end

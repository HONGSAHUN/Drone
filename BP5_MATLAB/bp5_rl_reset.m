function [obs,info]=bp5_rl_reset(P)
% Hover residual-control task. No toolbox required for reset/step themselves.
if nargin<1, P=bp5_params(); end
target=[0;0;P.groundHeight+1];
s=bp5_initial_state(P,target+[.15*randn(2,1);.08*randn]);
s.v=.03*randn(3,1); hover=P.B\[P.mass*P.g;0;0;0];
s.omega=sqrt(hover/P.kThrust);
info=struct('s',s,'P',P,'target',target,'step',0,'maxSteps',400,'dt',.02);
obs=bp5_rl_observation(info);
end

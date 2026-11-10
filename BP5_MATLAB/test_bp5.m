function report=test_bp5()
P=bp5_params(); assert(all(eig(P.J)>0)); assert(rank(P.B)==4);
assert(abs(P.mass-P.measuredTotalMass)<1e-10,'Measured total mass mismatch.');
% Hover equilibrium: correct thrust, no spurious attitude or lateral drift.
s=bp5_initial_state(P,[0;0;2]); hover=P.B\[P.mass*P.g;0;0;0];
s.omega=sqrt(hover/P.kThrust); s0=s;
for k=1:100, s=bp5_step(s,hover,P,.01); end
assert(norm(s.p-s0.p)<1e-9 && norm(s.w)<1e-9,'Hover equilibrium failed.');
% Ballistic limit with drag off: acceleration must be exactly -g.
Q=P; Q.linearDrag=zeros(3,1); s=bp5_initial_state(Q,[0;0;10]);
s=bp5_step(s,zeros(4,1),Q,.1);
assert(abs(s.p(3)-(10-.5*Q.g*.1^2))<1e-9 && abs(s.v(3)+Q.g*.1)<1e-9);
% Pure wrench allocation signs and reconstruction.
for k=2:4
    wrench=[P.mass*P.g;zeros(3,1)]; wrench(k)=.005;
    assert(norm(P.B*(P.B\wrench)-wrench)<1e-10);
end
L=bp5_simulate(P);
assert(all(isfinite(L.p(:))) && all(isfinite(L.omega(:))));
assert(max(L.omega(:))<=P.omegaMax+1e-8 && min(L.omega(:))>=0);
assert(L.rmsPosition<.20 && L.finalHorizontalError<.10,'Baseline tracking failed.');
assert(max(abs(L.p(1,:)))<.02,'Unexpected forward drift.');
for k=1:100:numel(L.t)
    R=L.R(:,:,k); assert(norm(R'*R-eye(3),'fro')<1e-8 && abs(det(R)-1)<1e-8);
end
assert(L.landed && abs(L.p(3,end)-P.groundHeight)<1e-9,'Landing contact failed.');
assert(max(L.omega(:,end))<1e-6,'Landing motor disarm failed.');
% The untrained zero-residual policy must preserve the classical hover loop.
rng(42); [obs,info]=bp5_rl_reset(P); assert(isequal(size(obs),[23 1]));
for k=1:info.maxSteps
    [obs,reward,done,info]=bp5_rl_step(zeros(3,1),info);
    assert(all(isfinite(obs)) && isfinite(reward));
    if done, break; end
end
assert(k==400 && norm(info.s.p-info.target)<.03,'Zero-residual RL baseline failed.');
report=struct('passed',true,'rmsPosition',L.rmsPosition, ...
    'finalHorizontalError',L.finalHorizontalError,'userReportedMass',P.mass);
disp(report);
end

function s=bp5_step(s,command,P,dt)
% Newton-Euler rigid body; motor speed lag; flat, inelastic ground clamp.
% No aerodynamic duct model, prop wash, sensor estimator or real contact solver.
n=ceil(dt/P.substep); h=dt/n;
omegaCmd=sqrt(max(0,min(P.maxMotorThrust,command(:)))/P.kThrust);
for k=1:n
    % Midpoint rotor speed improves force integration during motor transients.
    omMid=omegaCmd+(s.omega-omegaCmd)*exp(-h/(2*P.motorTau));
    s.omega=omegaCmd+(s.omega-omegaCmd)*exp(-h/P.motorTau);
    wrench=P.B*(P.kThrust*omMid.^2);
    dw=P.J\(wrench(2:4)-cross(s.w,P.J*s.w)-P.angularDrag.*s.w);
    wm=s.w+.5*h*dw;
    Rmid=s.R*rotvec(.5*h*wm);
    acc=(Rmid(:,3)*wrench(1)-P.linearDrag.*s.v)/P.mass-[0;0;P.g];
    s.p=s.p+h*s.v+.5*h*h*acc; s.v=s.v+h*acc;
    s.R=s.R*rotvec(h*wm); s.w=s.w+h*dw;
    if s.p(3)<P.groundHeight
        s.p(3)=P.groundHeight;
        s.v(3)=max(0,s.v(3));
        % Crude lateral friction at ground, not a multicontact rigid-body model.
        s.v(1:2)=s.v(1:2)*exp(-15*h);
    end
end
% Remove accumulated floating-point drift from the rotation representation.
[U,~,V]=svd(s.R); s.R=U*diag([1 1 det(U*V')])*V';
end

function R=rotvec(v)
theta=norm(v); K=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
if theta<1e-8
    R=eye(3)+K+.5*K*K;
else
    R=eye(3)+sin(theta)/theta*K+(1-cos(theta))/theta^2*K*K;
end
end

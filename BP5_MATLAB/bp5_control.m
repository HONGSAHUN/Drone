function [command,diagOut]=bp5_control(s,ref,P,residualAccel)
% Position PD + acceleration feedforward -> geometric attitude PD -> mixer.
if nargin<4, residualAccel=zeros(3,1); end
a=ref.a+P.kp.*(ref.p-s.p)+P.kd.*(ref.v-s.v)+residualAccel;
a(3)=max(-P.maxAccelZ,min(P.maxAccelZ,a(3)));
vertical=P.g+a(3); maxXY=vertical*tan(P.maxTilt);
if norm(a(1:2))>maxXY, a(1:2)=a(1:2)*maxXY/norm(a(1:2)); end
F=P.mass*(a+[0;0;P.g]); b3=F/norm(F);
b1c=[cos(ref.yaw);sin(ref.yaw);0]; b2=cross(b3,b1c); b2=b2/norm(b2);
Rd=[cross(b2,b3),b2,b3]; E=.5*(Rd'*s.R-s.R'*Rd);
eR=[E(3,2);E(1,3);E(2,1)];
tau=-P.kR.*eR-P.kw.*s.w+cross(s.w,P.J*s.w);
T=max(0,dot(F,s.R(:,3)));
raw=P.B\[T;tau];
command=max(0,min(P.maxMotorThrust,raw));
diagOut=struct('Rdesired',Rd,'requestedWrench',[T;tau], ...
    'saturated',any(abs(raw-command)>1e-8));
end

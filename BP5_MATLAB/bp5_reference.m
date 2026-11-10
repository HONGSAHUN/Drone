function ref=bp5_reference(t,P)
% Smooth rest-to-rest path: takeoff, right, left, center, land.
times=[0 1 4 5 8 11 14 15 18];
pts=[0 0 0 0 0 0 0 0 0; ...
     0 0 0 0 -1 1 0 0 0; ...
     0 0 1 1 1 1 1 1 0];
pts(3,:)=pts(3,:)+P.groundHeight;
i=find(t>=times,1,'last');
if isempty(i), i=1; end
if i>=numel(times)
    ref=struct('p',pts(:,end),'v',zeros(3,1),'a',zeros(3,1),'yaw',0); return;
end
T=times(i+1)-times(i); u=max(0,min(1,(t-times(i))/T));
h=10*u^3-15*u^4+6*u^5;
hd=(30*u^2-60*u^3+30*u^4)/T;
hdd=(60*u-180*u^2+120*u^3)/T^2;
d=pts(:,i+1)-pts(:,i);
ref=struct('p',pts(:,i)+h*d,'v',hd*d,'a',hdd*d,'yaw',0);
end

function s=bp5_initial_state(P,position)
if nargin<2, position=[0;0;P.groundHeight]; end
s=struct('p',position,'v',zeros(3,1),'R',eye(3),'w',zeros(3,1), ...
    'omega',zeros(4,1));
end

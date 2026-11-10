function L=bp5_simulate(P)
if nargin<1, P=bp5_params(); end
L.t=0:P.dt:P.duration; N=numel(L.t);
L.p=zeros(3,N); L.v=L.p; L.ref=L.p; L.w=L.p;
L.R=zeros(3,3,N); L.omega=zeros(4,N); L.command=L.omega;
L.saturated=false(1,N); s=bp5_initial_state(P);
landed=false;
for k=1:N
    ref=bp5_reference(L.t(k),P); [u,d]=bp5_control(s,ref,P);
    % Simulation-only landing/disarm supervisor after the descent segment.
    if L.t(k)>=18 && s.p(3)-P.groundHeight<.015 && abs(s.v(3))<.15
        landed=true;
    end
    if landed, u=zeros(4,1); end
    L.p(:,k)=s.p; L.v(:,k)=s.v; L.R(:,:,k)=s.R; L.w(:,k)=s.w;
    L.ref(:,k)=ref.p; L.omega(:,k)=s.omega; L.command(:,k)=u;
    L.saturated(k)=d.saturated;
    if k<N, s=bp5_step(s,u,P,P.dt); end
end
L.rmsPosition=sqrt(mean(sum((L.p-L.ref).^2,1)));
L.finalHorizontalError=norm(L.p(1:2,end)-L.ref(1:2,end));
L.landed=landed;
L.P=P;
end

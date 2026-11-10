function bp5_animate(L,videoFile)
% Optional videoFile: full path ending .avi (Motion JPEG).
if nargin<2, videoFile=''; end
P=L.P; fig=figure('Name','BP5 | provisional physical model','Color',[.97 .98 1], ...
    'Position',[100 100 1200 720]);
layout=tiledlayout(fig,2,2,'TileSpacing','compact');
ax=nexttile(layout,1,[2 1]); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
xlabel(ax,'forward x (m)'); ylabel(ax,'left y (m)'); zlabel(ax,'height (m)');
xlim(ax,[-.6 .6]); ylim(ax,[-1.5 1.5]); zlim(ax,[0 1.5]); view(ax,55,24);
patch(ax,[-.5 .5 .5 -.5],[-1.5 -1.5 1.5 1.5],[0 0 0 0], ...
    [.87 .90 .87],'FaceAlpha',.35,'EdgeColor','none');
plot3(ax,L.ref(1,:),L.ref(2,:),L.ref(3,:),'--','Color',[.5 .5 .5]);
trail=plot3(ax,NaN,NaN,NaN,'Color',[.10 .4 .9],'LineWidth',1.5);
H=bp5_scene(ax,P);
a2=nexttile(layout); hold(a2,'on'); grid(a2,'on');
plot(a2,L.t,L.ref(3,:)-P.groundHeight,'--','Color',[.55 .55 .55]);
plot(a2,L.t,L.p(3,:)-P.groundHeight,'Color',[.1 .4 .9],'LineWidth',1.5);
zmark=plot(a2,0,0,'o','MarkerFaceColor',[.1 .4 .9]);
ylabel(a2,'altitude above landed COM (m)'); xlabel(a2,'time (s)');
legend(a2,'target','actual','Location','southeast');
a3=nexttile(layout); hold(a3,'on'); grid(a3,'on');
plot(a3,L.t,L.ref(2,:),'--','Color',[.55 .55 .55]);
plot(a3,L.t,L.p(2,:),'Color',[.8 .18 .48],'LineWidth',1.5);
ymark=plot(a3,0,0,'o','MarkerFaceColor',[.8 .18 .48]);
ylabel(a3,'lateral y (m, left positive)'); xlabel(a3,'time (s)');
title(layout,sprintf('BP5 | MASS %.3f kg | INERTIA / PROPULSION UNCALIBRATED',P.mass));
writer=[];
if ~isempty(videoFile)
    writer=VideoWriter(videoFile,'Motion JPEG AVI'); writer.FrameRate=25; open(writer);
    cleanup=onCleanup(@() close(writer));
end
stride=max(1,round(.04/P.dt)); spin=zeros(4,1); last=1;
indices=unique([1:stride:numel(L.t),numel(L.t)]);
for k=indices
    if ~isgraphics(fig), break; end
    H.body.Matrix=[L.R(:,:,k),L.p(:,k);0 0 0 1];
    spin=spin-P.yawSign.*L.omega(:,k)*(L.t(k)-L.t(last)); last=k;
    for j=1:4
        a=spin(j); Q=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
        H.prop(j).Matrix=[Q,H.propCenter(:,j);0 0 0 1];
    end
    set(trail,'XData',L.p(1,1:k),'YData',L.p(2,1:k),'ZData',L.p(3,1:k));
    set(zmark,'XData',L.t(k),'YData',L.p(3,k)-P.groundHeight);
    set(ymark,'XData',L.t(k),'YData',L.p(2,k));
    title(ax,sprintf('t = %.2f s | user-reported mass %.3f kg',L.t(k),P.mass));
    drawnow;
    if ~isempty(writer), writeVideo(writer,getframe(fig)); end
end
end

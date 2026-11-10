function L=run_bp5_demo(animate)
% Run from this folder: run_bp5_demo
if nargin<1, animate=true; end
P=bp5_params(); L=bp5_simulate(P);
out=fullfile(fileparts(mfilename('fullpath')),'results');
if ~exist(out,'dir'), mkdir(out); end
save(fullfile(out,'baseline.mat'),'L');
data=array2table([L.t',L.p',L.ref',L.omega'], ...
    'VariableNames',{'t','x','y','z','ref_x','ref_y','ref_z','omega1','omega2','omega3','omega4'});
writetable(data,fullfile(out,'baseline.csv'));
fprintf('UNCALIBRATED MODEL | user-reported total mass %.3f kg\n',P.mass);
fprintf('3-D tracking RMS: %.4f m | final horizontal error: %.4f m\n', ...
    L.rmsPosition,L.finalHorizontalError);
fig=figure('Visible','off','Color','w','Position',[100 100 1250 820]);
tl=tiledlayout(fig,2,2,'TileSpacing','compact');
ax=nexttile(tl); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,45,27);
H=bp5_scene(ax,P); H.body.Matrix=[eye(3),[0;0;P.groundHeight];0 0 0 1];
xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); zlabel(ax,'z (m)');
title(ax,'Photo-based geometry (approximate)');
ax=nexttile(tl); plot(ax,L.t,L.ref(3,:)-P.groundHeight,'k--', ...
    L.t,L.p(3,:)-P.groundHeight,'b','LineWidth',1.3); grid(ax,'on');
xlabel(ax,'time (s)'); ylabel(ax,'altitude (m)'); legend(ax,'target','actual');
ax=nexttile(tl); plot(ax,L.t,L.ref(2,:),'k--',L.t,L.p(2,:),'m','LineWidth',1.3);
grid(ax,'on'); xlabel(ax,'time (s)'); ylabel(ax,'lateral y (m)');
ax=nexttile(tl); plot(ax,L.t,L.omega'*60/(2*pi)); grid(ax,'on');
xlabel(ax,'time (s)'); ylabel(ax,'rotor speed (RPM, assumed motor model)');
title(tl,sprintf('BP5 | mass %.3f kg (user) | inertia / thrust UNCALIBRATED | RMS %.3f m', ...
    P.mass,L.rmsPosition));
exportgraphics(fig,fullfile(out,'baseline.png'),'Resolution',150); close(fig);
if animate, bp5_animate(L); end
end

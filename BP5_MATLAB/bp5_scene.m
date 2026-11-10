function H=bp5_scene(ax,P)
% Procedural visual approximation, not manufacturing CAD.
H.body=hgtransform('Parent',ax);
dark=[.10 .13 .15]; pink=[1 .14 .55];
th=linspace(0,2*pi,70);
for k=1:4
    xy=P.rotorXY(:,k);
    for z=[-.0175 .0175]
        line('XData',xy(1)+P.guardRadius*cos(th)-P.com(1), ...
            'YData',xy(2)+P.guardRadius*sin(th)-P.com(2), ...
            'ZData',z*ones(size(th))-P.com(3),'Color',dark, ...
            'LineWidth',2,'Parent',H.body);
    end
    for ang=[0 pi/2 pi 3*pi/2]
        endpt=[xy+P.guardRadius*[cos(ang);sin(ang)];.0175];
        segment([xy;.0175],endpt,dark,2,H.body,P.com);
        segment(endpt,endpt-[0;0;.035],[.8 .12 .12],2,H.body,P.com);
    end
    segment([0;0;0],[xy;0],dark,6,H.body,P.com);
    boxpart([xy;-.005],[.025;.025;.025],dark,H.body,P.com);
    H.prop(k)=hgtransform('Parent',H.body);
    center=[xy;.01]-P.com;
    H.prop(k).Matrix=[eye(3),center;0 0 0 1];
    H.propCenter(:,k)=center;
    for ang=0:2*pi/3:4*pi/3
        r=P.maxPropDiameter*.46;
        v=[.008 r r*.93 .008;-.005 -.008 .008 .005;0 0 0 0];
        Q=[cos(ang) -sin(ang) 0;sin(ang) cos(ang) 0;0 0 1]; v=Q*v;
        patch('Vertices',v','Faces',1:4,'FaceColor',pink, ...
            'EdgeColor','none','Parent',H.prop(k));
    end
end
boxpart([0;0;.030],[.070;.060;.035],[.08 .48 .26],H.body,P.com);
boxpart([.015;0;.080],[.030;.150;.030],[.84 .86 .89],H.body,P.com);
% Camera face points along +body x. Lens circles approximate the supplied photo.
for y=[-.052 0 .052]
    line('XData',.031*ones(size(th))-P.com(1), ...
        'YData',y+.008*cos(th)-P.com(2), ...
        'ZData',.080+.008*sin(th)-P.com(3), ...
        'Color',dark,'LineWidth',3,'Parent',H.body);
end
boxpart([-.010;0;-.020],[.075;.035;.030],[.18 .19 .24],H.body,P.com);
for x=[-.055 .055]
    segment([x;-.040;-.020],[x;-.040;-.050],[.5 .5 .5],2,H.body,P.com);
    segment([x;.040;-.020],[x;.040;-.050],[.5 .5 .5],2,H.body,P.com);
end
end

function segment(a,b,col,width,parent,com)
v=[a-com,b-com]; line('XData',v(1,:),'YData',v(2,:), ...
    'ZData',v(3,:),'Color',col,'LineWidth',width,'Parent',parent);
end

function boxpart(c,sz,col,parent,com)
v=[-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1; ...
   -1 -1 1;1 -1 1;1 1 1;-1 1 1].*sz'/2+c'-com';
patch('Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8], ...
    'FaceColor',col,'EdgeColor',col*.7,'Parent',parent);
end

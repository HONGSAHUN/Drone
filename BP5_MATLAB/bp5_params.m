function P = bp5_params()
% BP5 photographic approximation. SI units throughout.
% Product screenshot values are NOT measurements of the assembled aircraft.
P.g = 9.81;
P.frameSize = [0.285; 0.292; 0.035]; % supplied product screenshot
P.maxPropDiameter = 0.127;           % maximum supported, not measured prop
P.frameMass = 0.120;                % kit-only catalog value, approximate
P.catalogDryMass = 0.363;           % reference only; NEVER used as total mass
P.measuredTotalMass = 1.300;        % user-reported flight-ready mass, kg
% User-reported installed hardware. Labels do not identify thrust/inertia.
P.hardware.motor = 'RS2205-2300KV';
P.hardware.motorKV = 2300; % RPM/V, not a thrust coefficient or loaded RPM
P.hardware.propeller = 'HQProp DP 5x4.5x3 V3';
P.hardware.propPitch = .1143; % 4.5 inch nominal pitch, user-reported
P.hardware.propBlades = 3;
P.hardware.battery = 'UCELL LiPo 4S 14.8V 1550mAh 150C';
P.hardware.nominalVoltage = 14.8;
P.hardware.capacityAh = 1.550;
P.hardware.FC = 'MicoAirH743v2';
P.hardware.firmware = 'UNKNOWN';
P.guardRadius = 0.070;              % ASSUMPTION, photo approximation
P.rotorXY = [0.0725 0.076; -0.0725 0.076; ...
             -0.0725 -0.076; 0.0725 -0.076]'; % ASSUMPTION
% Body axes: x forward, y left, z up. Motor numbering follows rotorXY.
% Positive yaw reaction signs; not verified physical wiring/spin directions.
P.yawSign = [1; -1; 1; -1];
% ALL masses below except frame kit mass are provisional, not identified.
% Part positions are relative to the frame center; xyz box sizes in metres.
P.parts = struct('name',{},'mass',{},'xyz',{},'size',{});
P.parts(1) = part('frame',.120,[0;0;0],P.frameSize);
for k=1:4
    P.parts(end+1) = part(sprintf('motor%d',k),.032, ...
        [P.rotorXY(:,k);0],[.026;.026;.025]);
end
P.parts(end+1) = part('electronics',.130,[0;0;.035],[.070;.060;.040]);
P.parts(end+1) = part('camera',.090,[.015;0;.080],[.030;.150;.030]);
P.parts(end+1) = part('battery',.230,[-.010;0;-.020],[.075;.035;.030]);
P.parts(end+1) = part('mounts_cables',.052,[0;0;.015],[.100;.080;.060]);
% Reconcile measured total without pretending to know component weights.
% Remaining 0.550 kg is an UNLOCATED mass with provisional position/box size.
% This preserves measured total mass, but COM and inertia remain assumptions.
remaining=P.measuredTotalMass-sum([P.parts.mass]);
assert(remaining>=0,'Component estimates exceed measured total; revise part masses.');
if remaining>0
    P.parts(end+1)=part('unlocated_mass',remaining,[0;0;.025],[.160;.160;.100]);
end
P.mass = sum([P.parts.mass]);
P.com = zeros(3,1);
for k=1:numel(P.parts)
    P.com = P.com + P.parts(k).mass * P.parts(k).xyz / P.mass;
end
% Uniform boxes + parallel-axis theorem: coarse inertia estimate only.
P.J = zeros(3);
for k=1:numel(P.parts)
    c=P.parts(k); d=c.xyz-P.com; a=c.size;
    Jc=c.mass/12*diag([a(2)^2+a(3)^2,a(1)^2+a(3)^2,a(1)^2+a(2)^2]);
    P.J=P.J+Jc+c.mass*((d'*d)*eye(3)-d*d');
end
P.rotorPosition = [P.rotorXY;zeros(1,4)] - P.com;
P.kThrust = 6/(2200^2); % ASSUMPTION N/(rad/s)^2, not a BP5 motor spec
P.omegaMax = 2200;     % ASSUMPTION rad/s
P.maxMotorThrust = P.kThrust*P.omegaMax^2;
P.yawMomentPerThrust = .012; % ASSUMPTION metres, kQ/kT
P.motorTau = .035;          % ASSUMPTION first-order rotor-speed lag, s
P.linearDrag = [.12;.12;.18]; % ASSUMPTION N/(m/s)
P.angularDrag = [.0002;.0002;.0003]; % ASSUMPTION N m/(rad/s)
P.B = [ones(1,4);P.rotorPosition(2,:);-P.rotorPosition(1,:); ...
       P.yawMomentPerThrust*P.yawSign'];
P.groundHeight = P.com(3)+.050; % flat-ground COM height, simplified contact
P.dt = .01; P.substep = .002; P.duration = 20;
P.kp = [3.0;3.0;5.0]; P.kd = [3.5;3.5;4.0];
% Torque gains scale with estimated inertia. Baseline controller, not flight gains.
P.kR = 90*diag(P.J); P.kw = 18*diag(P.J);
P.maxTilt = 25*pi/180; P.maxAccelZ = 4;
P.assumptions = 'Total mass 1.3 kg user-reported; geometry approximate; component mass distribution, inertia and propulsion uncalibrated.';
end

function c=part(name,mass,xyz,sz)
c=struct('name',name,'mass',mass,'xyz',xyz,'size',sz);
end

function obs=bp5_rl_observation(info)
s=info.s; P=info.P;
obs=[info.target-s.p;-s.v;s.R(:);s.w; ...
    s.omega/P.omegaMax;1-info.step/info.maxSteps]; % 23 x 1
end

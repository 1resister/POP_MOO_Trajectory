function epsilon = dynamic_tolerance(d_corner, cfg)
%DYNAMIC_TOLERANCE Smooth 0.05-to-0.10 mm corner tolerance transition.
% xi=clip(1-d/L,0,1), h=3*xi^2-2*xi^3.
L = cfg.geometry.transition_length;
xi = min(1,max(0,1-d_corner./L));
h = 3*xi.^2 - 2*xi.^3;
epsilon = cfg.geometry.line_tolerance + ...
    (cfg.geometry.corner_tolerance-cfg.geometry.line_tolerance).*h;
end

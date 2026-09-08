function [A, B, info] = build_resonance_model(mode, acceleration_scale)
%BUILD_RESONANCE_MODEL Continuous normalized second-order mechanical mode.
% qdd+2*zeta*wn*qd+wn^2*q=K*wn^2*(a/Amax).
wn = 2*pi*mode.frequency;
A = [0 1; -wn^2 -2*mode.zeta*wn];
B = [0; mode.gain*wn^2/acceleration_scale];
info.frequency = mode.frequency;
info.wn = wn;
info.zeta = mode.zeta;
info.gain = mode.gain;
end

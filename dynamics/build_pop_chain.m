function [A, B] = build_pop_chain()
%BUILD_POP_CHAIN Continuous p/v/a/j/snap/crackle chain driven by POP.
% xdot=A*x+B*u, where u=p^(6) [mm/s^6].
A = zeros(6);
A(1:5,2:6) = eye(5);
B = [0;0;0;0;0;1];
end

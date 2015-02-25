function SV = calc_vel_a(SV,R)
%
% -------------------------------------------
% | Basic Robotics Derived (Matlab toolbox) |
% -------------------------------------------
% | Kinematics |
% --------------
%
% calc_vel_a
%
% Updates SV.L(2:SP.n+1).w and SV.L(2:SP.n+1).v using the matrix R (see calc_hh_a.m)
%
% Note: This function requires updated positions SV = calc_pos(SP,SV);
%
% Syntax:
% -------
% SV = calc_vel_a(SV,R)
%
% Input:
% ------
% SV    - structure containing system variables (see struct_def.m)
% R     - Augmented Jacobian (link centroid) see calc_hh_a.m
%
% Output:
% -------
% SV         
%
% SV members changed:
% --------------------
% SV.L(2:SP.n+1).w
% SV.L(2:SP.n+1).v
% 

Y = R(7:end,:)*[SV.L(1).v;SV.L(1).w;SV.dq];

for i = 1:length(SV.L)-1
  SV.L(i+1).v = Y(6*i-5:6*i-3);
  SV.L(i+1).w = Y(6*i-2:6*i);
end

%%%EOF
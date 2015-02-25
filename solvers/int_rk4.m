function  SV = int_rk4(SP,SV,d_time,Gravity)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Solvers |
% -------------
%
% "abused" Runge–Kutta 4th order method (see the Note below)
%
% Syntax:
% -------
% SV = int_rk4(SP,SV,d_time,Gravity)
%
% Input:
% ------
% SP          - structure containing system parameters (see struct_def.m)
% SV          - structure containing system variables (see struct_def.m)
% d_time      - fixed integration step
% Gravity     - [3x1] gravity vector
%
% Output:
% -------
% SV
%
% SV members changed:
% --------------------
% L(1).p, L(1).R, L(1).Q, L(1).v, L(1).w, q, dq
%
% Note: 
% the change in SV.tau and the external forces and torques acting on the system during
% the interval d_time is not accounted for. This is more convenient and proves to be a "reasonable"
% approximation in most cases (if the system is not stiff)
%

[SV,dy1,y] = f_dyn_Q(SP,SV,Gravity, []);
[SV,dy2]   = f_dyn_Q(SP,SV,Gravity, y + d_time*dy1/2);
[SV,dy3]   = f_dyn_Q(SP,SV,Gravity, y + d_time*dy2/2);
[SV,dy4]   = f_dyn_Q(SP,SV,Gravity, y + d_time*dy3);

y = y+d_time*(dy1+2*dy2+2*dy3+dy4)/6;

% update the state
SV.L(1).p = y(1:3);
SV.L(1).Q = y(4:7)/norm(y(4:7)); SV.L(1).R = q2R(SV.L(1).Q);
SV.L(1).v = y(8:10);
SV.L(1).w = y(11:13);
SV.q = y(14:13+SP.n);
SV.dq = y(14+SP.n:13+2*SP.n);

%%%EOF
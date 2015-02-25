function [c,f] = r_ne1_a(SP,SV,R,Gravity,ddq,dv,dw)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Dynamics |
% ------------
%
% r_ne1_a
%
% recursive Newton-Euler method (inverse dynamics)
%
% Syntax:
% -------
% c = r_ne1_a(SP,SV,R,Gravity,ddq,dv,dw);
%
% Input:
% ------
% SP         - structure containing system parameters (see struct_def.m)
% SV         - structure containing system variables (see struct_def.m) 
% R          - Augmented Jacobian matrix (link centroid) (see calc_hh_a.m)
% Gravity    - [3x1] gravity vector
% ddq        - desired joint acceleration 
% dv         - desired linear acceleration of the CoM of the base 
% dw         - desired linear acceleration of the base
%
%
% Output:
% -------
% c          - joint forces/torques and base forces/torques that have to be applied in order for
%              the system to have a desired acceleration specified by ddq,dv,dw 
%
% SV members changed:
% --------------------
%

SV.ddq = ddq;
SV.L(1).dv = dv;
SV.L(1).dw = dw;

SV = calc_vel(SP,SV);
SV = calc_acc(SP,SV);

f = zeros(6*(SP.n+1),1);
for iL = 1:SP.n+1
  Ii = SV.L(iL).R*SP.L(iL).I*SV.L(iL).R';
  f(6*iL-5:6*iL) = [SP.L(iL).m*SV.L(iL).dv - (SV.L(iL).F + SP.L(iL).m*Gravity); 
                    Ii*SV.L(iL).dw + cross(SV.L(iL).w, Ii*SV.L(iL).w) - SV.L(iL).T];
end

c = R'*f; % backward recursion

%%%EOF
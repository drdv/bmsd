function c = r_ne_a(SP,SV,R,Gravity)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Dynamics |
% ------------
%
% r_ne_a
%
% recursive Newton-Euler method (inverse dynamics)
% 
% Syntax:
% -------
% c = r_ne_a(SP,SV,R,Gravity);
%
% Input:
% ------
% SP         - structure containing system parameters (see struct_def.m)
% SV         - structure containing system variables (see struct_def.m) 
% R          - Augmented Jacobian matrix (link centroid) (see calc_hh_a.m)
% Gravity    - [3x1] gravity vector
%
% Output:
% -------
% c          - nonlinear term of the equations of motion (includes external forces + gravity)
%
% SV members changed:
% --------------------
%

SV=forward_recursion(SP,SV);

f = zeros(6*(SP.n+1),1);
for iL = 1:SP.n+1
  Ii = SV.L(iL).R*SP.L(iL).I*SV.L(iL).R';
  f(6*iL-5:6*iL) = [SP.L(iL).m*SV.L(iL).dv - (SV.L(iL).F + SP.L(iL).m*Gravity); 
                    Ii*SV.L(iL).dw + cross(SV.L(iL).w, Ii*SV.L(iL).w) - SV.L(iL).T];
end

c = R'*f; % backward recursion

%%%EOF
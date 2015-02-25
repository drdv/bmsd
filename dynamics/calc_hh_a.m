function [HH,R,M] = calc_hh_a(SP,SV)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Dynamics |
% ------------
%
% calc_hh_a
%
% Calculation of the inertia matrix HH,
% augmented Jacobian matrix (with respect to the link centroids) R and
% augmented Mass/Inertia matrix M
% 
% Syntax:
% -------
% [HH, R, M] = calc_hh_a(SP,SV);
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% SV    - structure containing system variables (see struct_def.m)
%
% Output:
% -------
%
% HH [(6+SP.n)x(6+SP.n)]     - Global inertia matrix.
% R  [(6+6*SP.n)x(6+SP.n)]   - Augmented Jacobian matrix (link centroid).
% M  [(6+6*SP.n)x(6+6*SP.n)] - Augmented Mass/Inertia matrix.
%
% ("6" is the DOF of the base)
%
% Note: R = [Jbb, Jbm; 
%            Jmb, Jmm];
%
% Jbb = eye(6)        - d(base motion)/d(base motion)
% Jbm = zeros(6,SP.n) - d(base motion)/d(manipulator motion)
% Jmb                 - d(maniopulator motion)/d(base motion)
% Jmm                 - d(maniopulator motion)/d(maniopulator motion)
%
% SV members changed:
% --------------------
%

% memory allocation
M  = zeros(6*(SP.n+1),6*(SP.n+1));
R  = zeros(6+6*SP.n,6+SP.n);

R(7:end,7:end) = calc_Jmm(SP,SV); % Jmm

% Forming M, Jbb and Jmb
for iL = 1:SP.n+1
  ind = 6*iL-5:6*iL;  
  M(ind,ind) = [ eye(3)*SP.L(iL).m zeros(3,3); zeros(3,3) SV.L(iL).R*SP.L(iL).I*SV.L(iL).R' ];
  % Jbb abd Jmb
  R(ind,1:6) = [eye(3) tilde(SV.L(1).p-SV.L(iL).p);zeros(3,3) eye(3)];
end

% Forming HH (not efficient, but clear :))
HH=R'*M*R;

%%%EOF
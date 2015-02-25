function G = calc_G(SP,R,Gravity)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Dynamics |
% ------------
%
% calc_G
%
% Computes the generalized forces/torques as a result of Gravity
% H(q)*ddq + C(q,dq) + G(q) = tau
%
% Note that G is included in C when using r_ne_a
%
% Note: This function requires updated positions SV = calc_pos(SP,SV);
% 
% Syntax:
% -------
% G = calc_G(SP,SV,Gravity);
%
% Input:
% ------
% SP         - structure containing system parameters (see struct_def.m)
% R          - augmented Jacobian matrix (see calc_hh_a.m)
% Gravity    - [3x1] gravity vector
%
% Output:
% -------
% G          - G(q), generalized forces/torques as a result of Gravity
%
% SV members changed:
% --------------------
%

f = zeros(6*(SP.n+1),1);
for iL = 1:SP.n+1
  f(6*iL-5:6*iL) = [-SP.L(iL).m*Gravity; 
                    zeros(3,1)];
end

G = R'*f; 

%%%EOF
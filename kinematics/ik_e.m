function [q, iter] = ik_e(SP,SV,bN,bP,bR,dP,dR)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% ik_e
%
% Iterative algorithm for inverse kinematics computation
%
% Note: This function requires updated positions SV = calc_pos(SP,SV);
%
% Syntax:
% -------
% [q, iter] = ik_e(SP,SV,bN,bP,bR,dP,dR);
% [q, iter] = ik_e(SP,SV,bN,bP,bR,dP);
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% SV    - structure containing system variables (see struct_def.m)
% bN  [1x1]  - body number (on which is the end-effector) 
% bP  [3x1]  - position of the end-effector in the CF of body bN
% bR  [3x3]  - rotation matrix (a rotational offset from the CF of body bN)
% dP  [3x1]  - desired position of the end-effector in the WF
% dR  [3x3]  - rotation matrix representing the desired orientation 
%              of the end-effector w.r.t the WF
% 
% In dR is not specified, only the desired position of the end-effector is considered
%
% Output:
% -------
% q  [SP.nx1]   - joint angles that produce dP dR.
% iter          - number of iteration until convergence
%
% SV members changed:
% --------------------
%

if nargin < 7
  case_flag = 0; % only position
else
  case_flag = 1; % position + orientation
end
  
norm_err = 1;
iter = 0;
max_iter = 100;
gain = 1.0; % 0.99; 
tol = 0.00001;

while norm_err > tol && iter <= max_iter

  SV = calc_pos(SP,SV); 
  [pE,RE] = fk_e(SP,SV,bN,bP,bR);
  Je = calc_Je(SP,SV,SP.bN,SP.bP); 
   
  %==============================      
  err(1:3) = gain*(dP - pE);
  
  if case_flag 
    err_R = R_err(RE,dR);
    err(4:6) = gain*(err_R);
  end
  %==============================      

  norm_err = norm(err);

  if case_flag
    q_dir = pinv(Je)*err(:);
  else
    q_dir = pinv(Je(1:3,:))*err(:); % only position
  end
  
  SV.q =  SV.q + q_dir;
  
  iter = iter + 1;
end

if iter >= max_iter
  disp('Warning: The inverse kinematics routine did not converge.')
  q = zeros(SP.n,1);
else
  q = SV.q;
end

%%%EOF
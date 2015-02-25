function [p,R] = fk_e(SP,SV,bN,bP,bR)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% fk_e
%
% Calculates the position and orientation of an end-effector point
%
% Note: This function requires updated positions SV = calc_pos(SP,SV);
%
% Syntax:
% -------
% [p,R]=fk_e(SP,SV,bN,bP);
% [p,R]=fk_e(SP,SV,bN,bP,bR);
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% SV    - structure containing system variables (see struct_def.m)
% bN  [1x1]  - body number (on which is the end-effector) 
% bP  [3x1]  - position of the end-effector in the CF of body bN
% bR  [3x3]  - rotation matrix (a rotational offset from the CF of body bN)
%
% Output:
% -------
% p  [3x1]   - position of the end-effector
% R  [3x3]   - orientation of the end-effector
%
% SV members changed:
% --------------------
%
     
if bN <=0 || bN > SP.n+1 
  disp('ERROR: bN is out of range');
  return
end

p = SV.L(bN).p + SV.L(bN).R*bP;

if nargin < 5
    R = SV.L(bN).R;
else
    R = SV.L(bN).R*bR;
end



%%%EOF
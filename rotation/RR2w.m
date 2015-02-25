function w = RR2w(R1,R2,d_time)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% RR2w
%
% Compute angular velocity w given two rotation matrices R1,R2 and 
% time d_time for the transition from R1 to R2 
%
% Syntax:
% -------
% w = RR2w(R1,R2,d_time)
% 
% Input: 
% -------
% R1,R2 - rotation matrices
%
% Output:
% --------
% w - (3x1) body angular velocity (expressed in the world frame) 
%

% Algorithm:
% ----------
% Find axis and angle from the transition rotation matrix Rt = R2*R1'
% and form w as (an/d_time)*wa

Rt = R2*R1'; % transition rotation matrix
[an,wa] = R2aa(Rt);
w = (an/d_time)*wa;

%%%EOF
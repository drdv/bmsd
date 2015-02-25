function [an,ax] = R2aa(R)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% R2aa
%
% Converts a rotation matrix to axis-angle representation
% (the axis is of unit norm)
% 
% Syntax:
% -------
% [an,ax] = R2aa(R)
%
% Input:
% ------
% R  - 3x3 rotation matrix 
%
% Output: 
% -------
% an [1x1]  - angle [rad]
% ax [3x1]  - axis ||ax||=1
%

% Algorithm:
% ----------
% ... to include
%

an = acos((trace(R)-1)/2);

ax = [R(3,2) - R(2,3);
      R(1,3) - R(3,1);
      R(2,1) - R(1,2)];

ax = ax/norm(ax);

% That is the standard formula
% ax = 1/(2*sin(an))*[R(3,2) - R(2,3);
%                     R(1,3) - R(3,1);
%                     R(2,1) - R(1,2)];

%%%EOF
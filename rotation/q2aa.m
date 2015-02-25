function [an,ax] = q2aa(q)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% q2aa
%
% Converts a unit quaternion to axis-angle representation
% (the axis is of unit norm)
%
% Syntax:
% -------
% [an,ax] = q2aa(q)
%
% Input:
% ------
% q = [s x y z] - unit quaternion 
%     [s]       - scalar part
%     [x y z]   - vector part
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

an = 2*acos(q(1));
s = sqrt(1-q(1)^2);

if s<eps
  ax = [0;0;1];
else
  ax = q(2:4)'/s;
end
%%%EOF
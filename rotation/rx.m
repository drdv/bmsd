function Rx = rx(A)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% rx
%
% Returns a rotation matrix (X axis)
%
% Syntax:
% -------
% Rx = rx(A)
%
% Input:
% ------
% A  [1x1]   - angule [rad] 
%
% Output:
% -------
% Rx  [3x3]  - rotation matrix (X axis)
%

% Algorithm:
% ----------
% see http://en.wikipedia.org/wiki/Rotation_matrix

Rx = [1        0         0 ; 
      0   cos(A)   -sin(A) ;
      0   sin(A)    cos(A)];

%%%EOF
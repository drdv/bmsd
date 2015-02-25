function Rz = rz(A)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% rz
%
% Returns a rotation matrix (Z axis)
%
% Syntax:
% -------
% Rz = rz(A)
%
% Input:
% ------
% A  [1x1]   - angule [rad] 
%
% Output:
% -------
% Rz  [3x3]  - rotation matrix (Z axis)
%

% Algorithm:
% ----------
% see http://en.wikipedia.org/wiki/Rotation_matrix

Rz = [ cos(A)  -sin(A)   0 ;
       sin(A)   cos(A)   0 ;
            0        0   1 ];

%%%EOF
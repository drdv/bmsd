function R = rpy2R(r)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% rpy2R
%
% Convert a X-Y-Z rotation sequence (around the new axis) to rotation matrix 
% (the final result is equivalent to Z-Y-X around fixed axis)
%
% Syntax:
% -------
% R = rpy2R(r)
%
% Input:
% ------
% r  -  a 3D vector representing X-Y-Z rotation sequence (around the new axis)
%
% Output:
% -------
% R  - 3x3 rotation matrix
%

% Algorithm:
% -----------
% R = rx(X) * ry(Y) * rz(Z) -> X-Y-Z rotation sequence (around the new axis)
% rx, ry and rz are rotation matrices 
% for details see files rx.m, ry.m and rz.m

X = r(1);
Y = r(2);
Z = r(3);

R = [                      cos(Y)*cos(Z),                      -cos(Y)*sin(Z),         sin(Y);
      sin(X)*sin(Y)*cos(Z)+cos(X)*sin(Z), -sin(X)*sin(Y)*sin(Z)+cos(X)*cos(Z), -sin(X)*cos(Y);
     -cos(X)*sin(Y)*cos(Z)+sin(X)*sin(Z),  cos(X)*sin(Y)*sin(Z)+sin(X)*cos(Z),  cos(X)*cos(Y)];

%%%EOF
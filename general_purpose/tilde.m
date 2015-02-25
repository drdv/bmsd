function S = tilde(w)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | General purpose |
% -------------------
%
% tilde
%
% Returns a skew-symmetric matrix from a 3D vector
% if u and v are 3D vectors then cross(u,v) = tilde(u)*v 
% 
% Syntax:
% -------
% S = tilde(w);
%
% Input:
% ------
% w  [3x1]   - a 3D vector
%
% Output:
% -------
% S  [3x3]   - skew-symmetric matrix
%

S = [  0    -w(3)    w(2);
      w(3)    0     -w(1);
     -w(2)   w(1)     0   ];

%%%EOF
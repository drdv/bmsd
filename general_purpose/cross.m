function n = cross(u,v)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | General purpose |
% -------------------
%
% cross
%
% Cross product of two vectors
% 
% Syntax:
% -------
% n = cross(u,v);
%
% Input:
% ------
% u  [3x1]   -  3D vector
% v  [3x1]   -  3D vector
%
% Output:
% -------
% n  [3x1]   -  cross product of u and v
%

%n = zeros(3,1); % initialize the output

n(1) = u(2)*v(3) - u(3)*v(2);
n(2) = u(3)*v(1) - u(1)*v(3);
n(3) = u(1)*v(2) - u(2)*v(1);

n = transpose(n);

%%%EOF
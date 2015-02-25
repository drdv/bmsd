function q = rpy2q(r)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% Convert a X-Y-Z rotation sequence (around the new axis) to unit quaternion 
%
% Syntax:
% -------
% q = rpy2q(r)
%
% Input:
% ------
% rpy  -  a 3D vector representing X-Y-Z rotation sequence (around the new axis)
%
% Output:
% -------
% q = [s x y z] - unit quaternion 
%     [s]       - scalar part
%     [x y z]   - vector part
%

% Algorithm: 
% ----------
%  1. Form three quaternions coresponding to the three rotations around 
%     the X, Y and Z axis (using aa2q.m)
%  2. Multiply the three quaternions q = qx*qy*qz
%
%  for details see rpy2R.m and R2rpy.m

% first step
qx = aa2q(r(1), [1 0 0]);
qy = aa2q(r(2), [0 1 0]);
qz = aa2q(r(3), [0 0 1]);

% second step
q = zeros(1,4);
% tmp = quat_mult(qx, qy);
% q1 = quat_mult(tmp, qz);
% the product of [s1 x1 0 0]*[s2 0 y2 0]*[s3 0 0 z3] gives:
q(1) = qz(1)*qx(1)*qy(1)-qz(4)*qx(2)*qy(3);
q(2) = qz(1)*qx(2)*qy(1)+qz(4)*qx(1)*qy(3);
q(3) =-qz(4)*qx(2)*qy(1)+qz(1)*qx(1)*qy(3);  
q(4) = qz(4)*qx(1)*qy(1)+qz(1)*qx(2)*qy(3);

%%%EOF
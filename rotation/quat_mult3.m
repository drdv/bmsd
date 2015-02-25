function qm = quat_mult3(q1, q2, q3)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% quat_mult3
%
% Multiplication of three quaternions
%
% Syntax:
% -------
% qm = quat_mult(q1,q2,q3)
%
% Input:
% ------
% quaternions q1, q2 and q3 of the form
% 
% q = [s x y z] - unit quaternion 
%     [s]       - scalar part
%     [x y z]   - vector part
%
% Output: 
% -------
% qm = q1*q2*q3 
%

% Algorithm:
% ----------
% q1 = [s1 x1 y1 z1]
% q2 = [s2 x2 y2 z2]
% q3 = [s3 x3 y3 z3]
%
% qm = q1*q2*q3 = (q1*q2)*q3 = (q1*RM(q2))*q3 = q1*RM(q2)*RM(q3)
% where RM is an orthogonal matrix defined in quat_mult.m

% comment out the next line if you want to perform a symbolic computation
qm = zeros(1,4);

qm(1)=(q1(1)*q2(1)-q1(2)*q2(2)-q1(3)*q2(3)-q1(4)*q2(4))*q3(1)- ...
      (q1(1)*q2(2)+q1(2)*q2(1)+q1(3)*q2(4)-q1(4)*q2(3))*q3(2)- ...
      (q1(1)*q2(3)-q1(2)*q2(4)+q1(3)*q2(1)+q1(4)*q2(2))*q3(3)- ...
      (q1(1)*q2(4)+q1(2)*q2(3)-q1(3)*q2(2)+q1(4)*q2(1))*q3(4);
qm(2)=(q1(1)*q2(1)-q1(2)*q2(2)-q1(3)*q2(3)-q1(4)*q2(4))*q3(2)+ ...
      (q1(1)*q2(2)+q1(2)*q2(1)+q1(3)*q2(4)-q1(4)*q2(3))*q3(1)+ ...
      (q1(1)*q2(3)-q1(2)*q2(4)+q1(3)*q2(1)+q1(4)*q2(2))*q3(4)- ...
      (q1(1)*q2(4)+q1(2)*q2(3)-q1(3)*q2(2)+q1(4)*q2(1))*q3(3);
qm(3)=(q1(1)*q2(1)-q1(2)*q2(2)-q1(3)*q2(3)-q1(4)*q2(4))*q3(3)- ...
      (q1(1)*q2(2)+q1(2)*q2(1)+q1(3)*q2(4)-q1(4)*q2(3))*q3(4)+ ...
      (q1(1)*q2(3)-q1(2)*q2(4)+q1(3)*q2(1)+q1(4)*q2(2))*q3(1)+ ...
      (q1(1)*q2(4)+q1(2)*q2(3)-q1(3)*q2(2)+q1(4)*q2(1))*q3(2);
qm(4)=(q1(1)*q2(1)-q1(2)*q2(2)-q1(3)*q2(3)-q1(4)*q2(4))*q3(4)+ ...
      (q1(1)*q2(2)+q1(2)*q2(1)+q1(3)*q2(4)-q1(4)*q2(3))*q3(3)- ...
      (q1(1)*q2(3)-q1(2)*q2(4)+q1(3)*q2(1)+q1(4)*q2(2))*q3(2)+ ...
      (q1(1)*q2(4)+q1(2)*q2(3)-q1(3)*q2(2)+q1(4)*q2(1))*q3(1);

%%%EOF
function qm = quat_mult(q1,q2)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% quat_mult
%
% Multiplication of two quaternions
%
% Syntax:
% -------
% qm = quat_mult(q1,q2)
%
% Input:
% ------
% quaternions q1 and q2 of the form
% 
% q = [s x y z] - unit quaternion 
%     [s]       - scalar part
%     [x y z]   - vector part
%
% Output: 
% -------
% qm = q1*q2 
%

% Algorithm:
% ----------
% Let q1 and q2 be two quaternions of the form
% q1 = [s1 x1 y1 z1] = [s1 v1] = s1 + x1*i + y1*j + z1*k;
% q2 = [s2 x2 y2 z2] = [s2 v2] = s2 + x2*i + y2*j + z2*k;
%
% where
% i*j=k, j*k=i, k*i=j, j*i=-k, k*j=-i, i*k=-j, i^2 = j^2 = k^2 = i*j*k = -1
%
% q1*q2 = (s1 + x1*i + y1*j + z1*k)*(s2 + x2*i + y2*j + z2*k)
%       = s1*(s2 + x2*i + y2*j + z2*k) + x1*i*(s2 + x2*i + y2*j + z2*k) + ...
%         y1*j*(s2 + x2*i + y2*j + z2*k) + z1*k*(s2 + x2*i + y2*j + z2*k)
%       = s1*s2 + s1*x2*i + s1*y2*j + s1*z2*k + x1*s2*i - x1*x2 + x1*y2*k - x1*z2*j + ...
%         y1*s2*j - y1*x2*k - y1*y2 + y1*z2*i + z1*s2*k + z1*x2*j - z1*y2*i - z1*z2
%       = (s1*s2 - x1*x2 - y1*y2 - z1*z2 )   + ...
%         (s1*x2 + x1*s2 + y1*z2 - z1*y2 )*i + ... 
%         (s1*y2 - x1*z2 + y1*s2 + z1*x2 )*j + ...
%         (s1*z2 + x1*y2 - y1*x2 + z1*s2 )*k   .... (Eq. 1)
%
% The above formula can be expressed in a compact way as follows:
% q1*q2 = (s1*s2 - v1'*v2) + (s1*v2 + s2*v1 + cross(v1,v2))
% where v1 = [x1 y1 z1] and v2 = [x2 y2 z2] are the vector parts of q1 and q2
%
% Note: (Eq. 1) is a linear combination of the coeficients of q1. The same 
% thing holds for the coeficients of q2. Hece, we can write:
%
%                                          i   j   k
% q1*q2 = q1*RM = [s1, x1, y1, z1] * [ s2  x2  y2  z2;
%                                     -x2  s2 -z2  y2;
%                                     -y2  z2  s2 -x2;
%                                     -z2 -y2  x2  s2]; 
%
% where the matrix RM is orthogonal (and is a function of q2) 
%
% The product q1*q2 can be expressed in a "reversed way" as well.
% Rearanging (Eq. 1) we obtain:
%
% q1*q2 = (s1*s2 - x1*x2 - y1*y2 - z1*z2 )   + ...
%         (x1*s2 + s1*x2 - z1*y2 + y1*z2 )*i + ... 
%         (y1*s2 + z1*x2 + s1*y2 - x1*z2 )*j + ...
%         (z1*s2 - y1*x2 + x1*y2 + s1*z2 )*k 
%
% and can be expressed as:
%                                          i   j   k
% q1*q2 = q2*LM = [s2, x2, y2, z2] * [ s1  x1  y1  z1;
%                                     -x1  s1  z1 -y1;
%                                     -y1 -z1  s1  x1;
%                                     -z1  y1 -x1  s1]; 
%
% where the matrix LM is orthogonal (and is a function of q1),
% consequently q1*RM = q2*LM 
%
% The representation of quaternion multiplication using matrices LM and RM
% is important, because their product forms a rotation matrix (see q2R.m) 

%qm = q1 * R_matrix(q2);
%qm = q2 * L_matrix(q1);

qm(1) = q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
qm(2) = q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
qm(3) = q1(1)*q2(3) + q1(3)*q2(1) + q1(4)*q2(2) - q1(2)*q2(4);
qm(4) = q1(1)*q2(4) + q1(4)*q2(1) + q1(2)*q2(3) - q1(3)*q2(2);

%%%EOF
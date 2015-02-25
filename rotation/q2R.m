function R = q2R(s,x,y,z)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% q2R
%
% Converts a unit quaternion to rotation matrix 
%
% Syntax:
% -------
% q2R([1,0,0,0]);
% q2R( 1,0,0,0 );
%
% Input:
% ------
% q = [s x y z] - unit quaternion 
%     [s]       - scalar part
%     [x y z]   - vector part
%
% Output:
% -------
% R  - 3x3 rotation matrix 
%

% Algorithm:
% -----------
% Quaternions can be used to rotate a vector as follows
% (http://mathworld.wolfram.com/Quaternion.html)
%
% p1 = q*p*q^(-1)  ... (Eq. 1)
%
% where 
% p = [0 px py pz]        - is a vecotr represented as a quaternion 
% p1                      - is the rotated vector
% q      = [s x y z]      - is the rotation quaternion
% q^(-1) = [s -x -y -z]   - is the conjugate of q
% 
% Expanding the product in (Eq. 1) using the two matrices RM and LM derived
% in quat_mult.m leads to forming a rotation matrix
%
% As we noted in quat_mult.m the product q1*q2 can be expressed as follows
% q1*q2 = q2*LM(q1)  ... (Eq. 2)
% q1*q2 = q1*RM(q2)  ... (Eq. 3)
% 
% Applying (Eq. 2) to q*p leads to
% q*p = p*LM(q)
%
% Next applying (Eq. 3) for p*LM(q) and q^(-1) we obtain
%
% p1 = p*LM(q)*RM(q^(-1)) = p*DCM  ... (Eq. 4)
% where DCM is a Direction Cosine Matrix of the form
%
% DCM =
% [ s1^2+x1^2+y1^2+z1^2,                   0,                   0,                   0]
% [                   0, x1^2+s1^2-z1^2-y1^2,     2*x1*y1+2*s1*z1,     2*x1*z1-2*s1*y1]
% [                   0,     2*x1*y1-2*s1*z1, y1^2-z1^2+s1^2-x1^2,     2*y1*z1+2*s1*x1]
% [                   0,     2*x1*z1+2*s1*y1,     2*y1*z1-2*s1*x1, z1^2-y1^2-x1^2+s1^2];
%
% Not taking the transpose of equation (Eq. 4) leads to:
% p1' = R*p'
%
% where R is the rotation matrix = transpose of (Direction Cosine Matrix) 
% and p1' and p' are column vectors expressed in a quaternion form
%
% Since we are working with a unit quaternion s1^2+x1^2+y1^2+z1^2 = 1
% and we can represent the diagonal elements as:
% x1^2+s1^2-z1^2-y1^2 = ... 
% x1^2+s1^2-z1^2-y1^2 + y1^2+z1^2-y1^2-z1^2 = ...
% 1-2*y1^2-2*z1^2
% and so on ...
% 

if nargin == 1 && length(s) == 4
  x = s(2);
  y = s(3);
  z = s(4);
  s = s(1);
elseif nargin == 4 && length([s,x,y,z]) == 4
  % another way to input the parameters
else
  disp('Error in the input parameters')
  R = [];
  return
end

R = [1-2*z^2-2*y^2,   2*x*y-2*s*z,   2*x*z+2*s*y;
       2*x*y+2*s*z, 1-2*z^2-2*x^2,   2*y*z-2*s*x;
       2*x*z-2*s*y,   2*y*z+2*s*x, 1-2*y^2-2*x^2];

%%%EOF
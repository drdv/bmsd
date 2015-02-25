function rpy = q2rpy(q)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% q2rpy
%
% Convert a unit quaternion to X-Y-Z rotation sequence (around the new axis) 
%
% Of course, we can simply use q2R.m followed by R2rpy.m however, roll-pitch-yaw
% angle can be derived directly from a quaternion
%
% Syntax:
% -------
% rpy = q2rpy(q)
% 
% Input:
% ------
% q = [s x y z] - unit quaternion 
%     [s]       - scalar part
%     [x y z]   - vector part
%
% Output:
% -------
% rpy  :  a 3D vector representing X-Y-Z rotation sequence (around the new axis)
%

% Algorithm: 
% ----------
%     http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
%
%  1. From q2R.m we know that the rotation matrix corresponding to 
%     a quaternion q = [s x y z] is
%
%     R = [1-2*z^2-2*y^2,   2*x*y-2*s*z,   2*x*z+2*s*y;
%            2*x*y+2*s*z, 1-2*z^2-2*x^2,   2*y*z-2*s*x;
%            2*x*z-2*s*y,   2*y*z+2*s*x, 1-2*y^2-2*x^2];
%     
%     and from rpy2R.m
% 
%     R = [                      cos(Y)*cos(Z),                      -cos(Y)*sin(Z),         sin(Y);
%           sin(X)*sin(Y)*cos(Z)+cos(X)*sin(Z), -sin(X)*sin(Y)*sin(Z)+cos(X)*cos(Z), -sin(X)*cos(Y);
%          -cos(X)*sin(Y)*cos(Z)+sin(X)*sin(Z),  cos(X)*sin(Y)*sin(Z)+sin(X)*cos(Z),  cos(X)*cos(Y)];
%
%  2. From R2rpy.m we know that for the general case
%
%     X = -atan2(R(2,3),R(3,3));
%     Y =  atan2(R(1,3),sqrt(R(2,3)^2 + R(3,3)^2));
%     Z = -atan2(R(1,2),R(1,1));
%
%  3. Substituting the entries of R we obtain
% 
%     X = -atan2(2*y*z-2*s*x, 1-2*y^2-2*x^2);
%     Y =  asin(2*x*z+2*s*y); % for the computation of Y we use directly R(1,3)
%     Z = -atan2(2*x*y-2*s*z, 1-2*z^2-2*y^2); 
%
%  4. From the discussion in R2rpy.m we know that the X-Y-Z convention has a 
%     singular point at rpy(2) = +-pi/2. Next, we have to express this singularity
%     in terms of quaternions.
%
%     4.1 First we notice that the singularity happens at sin(pi/2) = 2*x*z+2*s*y = 1 
%         so x*z+s*y = 0.5
%     4.2 From rpy2q.m we know that q = qx*qy*qz and 
%         qy = aa2q([Y, 0 1 0]), hence by the definition of quaternion 
%         qy = [cos(Y/2) 0  sin(Y/2) 0], when Y = pi/2 we have
%         qy = [0.7071   0  0.7071   0], substituting this in the expression for
%                                        q in rpy2q.m and rearranging we obtain:
%
%         q(1) = 0.7071*( qz(1)*qx(1)-qz(4)*qx(2) );
%         q(2) = 0.7071*( qz(1)*qx(2)+qz(4)*qx(1) );
%         q(3) = 0.7071*( qz(1)*qx(1)-qz(4)*qx(2) );  
%         q(4) = 0.7071*( qz(1)*qx(2)+qz(4)*qx(1) );
%         
%         (the approximate numerical value 0.7071 of cos(pi/4) is used here for illustration purpose)
%
%         we note that q(1) = q(3) and q(2) = q(4)
%
%         using the definition of quaternion the above four equations can be
%         written in the following form:
%
%         q(1) = 0.7071*( cos(Z/2)*cos(X/2)-sin(Z/2)*sin(X/2) );
%         q(2) = 0.7071*( cos(Z/2)*sin(X/2)+sin(Z/2)*cos(X/2) );
%         q(3) = 0.7071*( cos(Z/2)*cos(X/2)-sin(Z/2)*sin(X/2) );
%         q(4) = 0.7071*( cos(Z/2)*sin(X/2)+sin(Z/2)*cos(X/2) );
%
%         Using the trigonometric formulas:
%         sin(A+B) = sin(A)*cos(B) + cos(A)*sin(B)
%         cos(A+B) = cos(A)*cos(B) - sin(A)*sin(B)
%
%         we obtain:
%
%         q(1) = q(3) = 0.7071*( cos(X/2+Z/2) );      ... (*)
%         q(2) = q(4) = 0.7071*( sin(X/2+Z/2) );      ... (**)
%
%         Of course, one of the above equations is redundant, so (as in R2rpy.m)
%         we have to fix X and determine Y (or vise versa)
%
%         X+Z = 2*acos(q(1)/0.7071);
%         X+Z = 2*asin(q(2)/0.7071); => 
%         acos(q(1)/0.7071) = asin(q(2)/0.7071) (note that 0.7071 is only an approximation of pi/4)
%
%         One can use any of the above two equation to determine Z (if X=0 is assumed)
%         however asin and acos give a 180 degree range and we can obtain 360, by 
%         dividing (**)/(*):
%
%         q(2)/q(1) = q(4)/q(3) = sin(X/2+Z/2)/cos(X/2+Z/2) = tan(X/2+Z/2) =>
%
%         X+Z = 2*atan2(q(2), q(1));
%
%         so for X = 0, Z = 2*atan2(q(2), q(1));
%
%     4.1 Next, if Y = -pi/2 the singularity happens at sin(-pi/2) = 2*x*z+2*s*y = -1 
%         so x*z+s*y = -0.5
%     4.3 If Y = -pi/2, qy = [0.7071   0  -0.7071   0], substituting this in the expression for
%                                                       q in rpy2q.m and rearranging we obtain: 
%
%         q(1) = 0.7071*( qz(1)*qx(1)+qz(4)*qx(2) );
%         q(2) = 0.7071*( qz(1)*qx(2)-qz(4)*qx(1) );
%         q(3) = 0.7071*(-qz(1)*qx(1)-qz(4)*qx(2) );  
%         q(4) = 0.7071*(-qz(1)*qx(2)+qz(4)*qx(1) );
%
%         this time q(1) = -q(3) and q(2) = -q(4)
%
%         using the definition of quaternion the above four equations can be
%         written in the following form:
%
%         q(1) = 0.7071*( cos(Z/2)*cos(X/2)+sin(Z/2)*sin(X/2) );
%         q(2) = 0.7071*( cos(Z/2)*sin(X/2)-sin(Z/2)*cos(X/2) );
%         q(3) = -0.7071*(cos(Z/2)*cos(X/2)+sin(Z/2)*sin(X/2) );
%         q(4) = -0.7071*(cos(Z/2)*sin(X/2)-sin(Z/2)*cos(X/2) );
%
%         Using the trigonometric formulas:
%         sin(A-B) = sin(A)*cos(B) - cos(A)*sin(B)
%         cos(A-B) = cos(A)*cos(B) + sin(A)*sin(B)
%
%         we obtain
%
%         q(1) = -q(3) = 0.7071*( cos(X/2-Z/2) );      ... (*1)
%         q(2) = -q(4) = 0.7071*( sin(X/2-Z/2) );      ... (**1)
%
%         q(2)/q(1) = q(4)/q(3) = tan(X/2-Z/2), hence
%
%         X-Z = 2*atan2(q(2), q(1));
%
%         if X = 0, Z = -2*atan2(q(2), q(1));

s = q(1);
x = q(2);
y = q(3);
z = q(4);

rpy = zeros(3,1);

% error tolerance
eps = 1e-14; % with some insurance

singular_test = x*z+s*y;
if singular_test < 0.5+eps && singular_test > 0.5-eps  % singularity Y = pi/2

  rpy(1) = 0; % set rpy(1)=0 in order to compute rpy(3)
  rpy(2) = pi/2;
  rpy(3) = 2*atan2(x, s);
  
elseif singular_test < -0.5+eps && singular_test > -0.5-eps % singularity Y = -pi/2
    
  rpy(1) = 0; % set rpy(1)=0 in order to compute rpy(3)
  rpy(2) = -pi/2;
  rpy(3) = -2*atan2(x, s);
  
else
  
  rpy(1) = -atan2(2*y*z-2*s*x, 1-2*y^2-2*x^2);
  rpy(2) =  asin(2*x*z+2*s*y);
  rpy(3) = -atan2(2*x*y-2*s*z, 1-2*z^2-2*y^2); 

end

%%%EOF
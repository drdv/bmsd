function rpy = R2rpy(R)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% R2rpy
%
% Convert a rotation matrix to X-Y-Z rotation sequence (around the new axis) 
%
% Syntax:
% -------
% rpy = R2rpy(R)
%
% Input:
% ------
% R  - 3x3 rotation matrix
%
% Output:
% -------
% rpy  :  a 3D vector representing X-Y-Z rotation sequence (around the new axis)
%

% Algorithm:
% ----------
% In rpy2R.m the reotational matrix corresponding to a X-Y-Z rotation sequence (around the new axis)
% is derive as
%
% R = [                      cos(Y)*cos(Z),                      -cos(Y)*sin(Z),         sin(Y);
%       sin(X)*sin(Y)*cos(Z)+cos(X)*sin(Z), -sin(X)*sin(Y)*sin(Z)+cos(X)*cos(Z), -sin(X)*cos(Y);
%      -cos(X)*sin(Y)*cos(Z)+sin(X)*sin(Z),  cos(X)*sin(Y)*sin(Z)+sin(X)*cos(Z),  cos(X)*cos(Y)];
%
% Note: At Y = pi/2 and Y = -pi/2 the angles X and Z can not be determined uniquely.  
% If Y = pi/2, R becomes
%
% [ 0, 0, 1]
% [ a, b, 0]
% [-b, a  0]
%
% where 
%
% a = sin(X)*cos(Z) + cos(X)*sin(Z)
% b = cos(X)*cos(Z) - sin(X)*sin(Z)
%
% if we divide a/b (try to separate and compute X or Z) we obtain
%
% a/b = sin(X+Z)/cos(X+Z)
%
% clearly X and Z can not be computed separately.
% This situation is referred to as singularity of the X-Y-Z notation. 
% Some of the difficulties that arise are:
%  -- we can have infinity many combinations of X and Z that lead to the same rotation matrix
%  -- infinite rotational rates [dX/dt, dY/dt, dZ/dt] are needed in order to produce a finit rigid 
%     body angular velocity. For details see
%
%      Singularities of Euler and Roll-Pitch-Yaw Representations
%      Ang, M.H.; Tourassis, V.D.
%      Aerospace and Electronic Systems, IEEE Transactions on
%      Volume AES-23, Issue 3, May 1987 Page(s):317 - 324
%
%      The same paper but published as a report can be downloaded at:
%      https://urresearch.rochester.edu/retrieve/2374/TM-55.pdf
%
% If Y ~= +-pi/2 computing X, Y and Z is straightforward:
%
%  R(1,2)/R(1,1) = -sin(Z)/cos(Z) = -tan(Z) 
%  R(2,3)/R(3,3) = -sin(X)/cos(X) = -tan(X)
%  R(1,3)/sqrt(R(2,3)^2 + R(3,3)^2) = sin(Y)/cos(Y)  ... because:
%
%  sqrt(R(2,3)^2 + R(3,3)^2) = sqrt(sin(X)^2*cos(Y)^2 + cos(X)^2*cos(Y)^2) = 
%                            = sqrt((sin(X)^2 + cos(X)^2)*cos(Y)^2) = 
%                            = sqrt(cos(Y)^2) = cos(Y)
%
%                            where sin(X)^2 + cos(X)^2 = 1 was used
% 
%  hence:
%
%  X = -atan2(R(2,3),R(3,3)) %not defined for Y = +-pi/2 because R(2,3) = R(3,3) = 0
%  Y =  atan2(R(1,3),sqrt(R(2,3)^2 + R(3,3)^2))
%  Z = -atan2(R(1,2),R(1,1)) %not defined for Y = +-pi/2 because R(1,2) = R(1,1) = 0 
%
%  see the definition of atan2 below
% 
%  Note: In order to detect whether Y = +-pi/2 we can check whether R(1,1) == 0 && R(1,2) == 0
%        from which it follows that R(2,3) == 0 && R(3,3) == 0 as well because the four elements
%        contain cos(Y).
%
%  Note on atan2:
%
%  if X > 0
%    atan2(y,x) = F*sign(y)
%  elseif X == 0
%    atan2(y,x) = pi/2*sign(y)
%  elseif X < 0
%    atan2(y,x) = (pi-F)*sign(y)
%  end
%  
%  when y = 0
%  
%  if X > 0
%    atan2(0,x) = 0
%  elseif X == 0
%    atan2(0,x) -> undefined  !!!!! (although Matlab returns 0)
%  elseif X < 0
%    atan2(0,x) = pi
%  end
%  
%  where F is the angle in [0, pi/2) such that tan(F) = abs(y/x)
%  for more details see http://en.wikipedia.org/wiki/Atan2

rpy = zeros(3,1);

% first consider the singular case Y = +-pi/2
if abs(R(1,1)) < eps && abs(R(1,2)) < eps

  rpy(2) = atan2( R(1,3) , R(1,1) ); % either atan2(1,0) = pi/2 or atan2(-1,0) = -pi/2
  
  % here we have to assign a value for rpy(1) in order to compute rpy(3) or vise versa. 
  % The easiest choice is rpy(1) = 0
  rpy(1) = 0;
  
  %hence, R(2,1)/R(2,2) = sin(X+Z)/cos(X+Z) = sin(Z)/cos(Z) = tan(Z) 
  rpy(3) = atan2( R(2,1) , R(2,2) );

else
   
  rpy(1) = -atan2(R(2,3),R(3,3));
  rpy(2) =  atan2(R(1,3),sqrt(R(2,3)^2 + R(3,3)^2));
  rpy(3) = -atan2(R(1,2),R(1,1)); 
  
end
  
%%%EOF
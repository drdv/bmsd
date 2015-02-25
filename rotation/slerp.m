function qm = slerp(qi,qn,t,eps)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% slerp
%
% Spherical Linear Interpolation (Slerp) using quaternions
%
% Syntax:
% -------
% qm = slerp(qi, qn, t, eps)
%
% Input:
% -------
% qi=[w1 x1 y1 z1] - start unit quaternions
% qn=[w2 x2 y2 z2] - end unit quaternions
% t=[0 to 1]       - t = 0 -> qm = qi ; t = 1 -> qm = qn  
% eps              - threshold value
%
% Output:
% -------
% qm               - unit quaternion which lies between qi and qn
%


% Algorithm:
% ----------
% This routine calculates a unit quaternion, which lies between two known 
% unit quaternions - q1 and q2, using a spherical linear interpolation - Slerp.
% Slerp follows the shortest great arc on a unit sphere,
% hence, the shortest possible interpolation path.
% Consequently, Slerp has constant angular velocity, 
% so it is the optimal interpolation curve between two rotations.
% (first published by Sheomake K., 1985 - Animating Rotation with Quaternion Curves)
%
% in general:
%    slerp(q1, q2, t) = q1*(sin(1-t)*theta)/sin(t) + q2*(sin(t*theta))/sin(theta)
% where theta is the angle between the two unit quaternions, and t is between [0,1]
%
% two border cases are distinguished:
%    1: where q1 = q2 (or close by eps)
%    2: where q1 = -q2 (angle between unit quaternions is 180 degrees).
%
% For mode details 
% see http://en.wikipedia.org/wiki/Slerp
% 
% I borrowed a code writted by Sagi Dalyot 
% http://www.mathworks.com/matlabcentral/fileexchange/loadFile.do?objectId=11827&objectType=File


if t==0 % saving calculation time -> where qm=qi
  qm=qi;
elseif t==1 % saving calculation time -> where qm=qn
  qm=qn;
else
  
  % Calculating the angle beteen the unit quaternions by dot product
  C=dot(qi,qn);                  
  theta=acos(C);
  
  % if angle theta is close by epsilon to 0 degrees -> calculate by linear interpolation
  if (1 - C) <= eps 
    qm=qi*(1-t)+qn*t; % avoiding divisions by number close to 0

    % when theta is close by epsilon to 180 degrees the result is undefined -> no shortest direction to rotate
  elseif (1 + C) <= eps  % this appears only when qi = -qn
    % rotating one of the unit quaternions by 90 degrees -> q2
    q2(1) = qi(4); q2(2) = -qi(3); q2(3)= qi(2); q2(4) = -qi(1); 
    qm=qi*(sin((1-t)*(pi/2)))+q2*sin(t*(pi/2));
    disp('theta = pi')
  else
    qm= (qi*sin((1-t)*theta) + qn*sin(t*theta))/sin(theta);
  end
end

%%%EOF
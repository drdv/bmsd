function RM = qRM(q)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% qRM
%
% Returns an orthogonal matrix RM for quaternion multiplication (see quat_mult.m)
% q1*q2 = q1*RM(q2)
% 
% Syntax:
% -------
% RM = qRM(q)
%
% Input:
% -------
% q = [s x y z] - unit quaternion 
%     [s]       - scalar part
%     [x y z]   - vector part
%
% Output:
% -------
% RM - 4x4 orthogonal matrix so that q1*q2 = q1*RM(q2)

RM = [ q(1)  q(2)  q(3)  q(4);
      -q(2)  q(1) -q(4)  q(3);
      -q(3)  q(4)  q(1) -q(2);
      -q(4) -q(3)  q(2)  q(1)];
 
%%%EOF
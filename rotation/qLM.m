function LM = qLM(q)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% qLM
%
% Returns an orthogonal matrix LM for quaternion multiplication (see quat_mult.m)
% q1*q2 = q2*LM(q1)
% 
% Syntax:
% -------
% LM = qLM(q)
%
% Input:
% -------
% q = [s x y z] - unit quaternion 
%     [s]       - scalar part
%     [x y z]   - vector part
%
% Output:
% -------
% LM - 4x4 orthogonal matrix so that q1*q2 = q2*LM(q1)

LM = [ q(1)  q(2)  q(3)  q(4);
      -q(2)  q(1)  q(4) -q(3);
      -q(3) -q(4)  q(1)  q(2);
      -q(4)  q(3) -q(2)  q(1)];
 
%%%EOF
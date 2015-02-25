function e = R_err(R1,R2)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% R_err
%
% Forms an orientation error between a current orientation 
% specified by R1 and a desired orientation specified by R2
%
% Syntax:
% -------
% e = R_err(R1,R2)
%
% Input:
% ------
% R1  -  current rotation matrix
% R2  -  desired rotation matrix
%
% Output:
% -------
% e  - 3x1 orientation error 
%

% Algorithm:
% ----------- 
% R = R2*R1'; % R*R1 = R2
% [an,ax] = R2aa(R);
% e = sin(an)*ax
%

e = 0.5*[cross(R1(:,1), R2(:,1)) + ... 
         cross(R1(:,2), R2(:,2)) + ...
         cross(R1(:,3), R2(:,3))];

%%%EOF
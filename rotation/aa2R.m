function R = aa2R(an,ax)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% aa2R
%
% Converts axis angle representation to rotation matrix
% This is reffered to as Rodrigues rotation formula.
% see
% http://mathworld.wolfram.com/RodriguesRotationFormula.html
% http://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
% 
% Syntax:
% -------
% R = aa2R(an,ax)
%
% Input:
% ------
% an [1x1]  - angle [rad]
% ax [3x1]  - axis
%
% Output: 
% -------
% R  - 3x3 rotation matrix 
%
% Note: The axis angle representation can be stored in only three 
% parameters if norm(ax) = 1, then a = ax*an whould contain all the 
% necessary information (an = norm(a), ax = a/norm(a)). This however,
% is not used here.
%

% Algorithm:
% ----------
% Using Matlab symblic toolbox:
% syms ux uy uz   % define the axis components
% syms an         % define the angle
% u = [ux uy uz]; % axis of rotation (unit norm is assumed)
% q = [cos(an/2)  u*sin(an/2)]; % quaternion from axis angle (see aa2q.m)
% R = simple(q2R(q)); 
%
% R = [1-uz^2+uz^2*cos(an)-uy^2+uy^2*cos(an),        ux*uy-ux*uy*cos(an)-uz*sin(an),        ux*uz-ux*uz*cos(an)+uy*sin(an)
%             ux*uy-ux*uy*cos(an)+uz*sin(an), 1-uz^2+uz^2*cos(an)-ux^2+ux^2*cos(an),        uy*uz-uy*uz*cos(an)-ux*sin(an)
%             ux*uz-ux*uz*cos(an)-uy*sin(an),        uy*uz-uy*uz*cos(an)+ux*sin(an), 1-uy^2+uy^2*cos(an)-ux^2+ux^2*cos(an)]
% 
% The main diagonal terms can be simplified as follows:
%
% 1-uz^2+uz^2*cos(an)-uy^2+uy^2*cos(an) = 
% 1+(cos(an)-1)*uz^2 + (cos(an)-1)*uy^2 =
% 1+(uz^2 + uy^2)*(cos(an)-1) =
% 1+(1 - ux^2)*(cos(an)-1) =          % note: ux^2 + uy^2 + uz^2 = 1
% 1+cos(an)-1-ux^2*(cos(an)-1) = 
% cos(an)-ux^2*(cos(an)-1) = 
% cos(an)+ux^2*(1-cos(an))
%
% ... and so forth for the remaining two terms
%
% So R becomes:
%
% R = [    cos(an)+ux^2*(1-cos(an)), (1-cos(an))*ux*uy-uz*sin(an), (1-cos(an)*ux*uz+uy*sin(an)
%      (1-cos(an))*ux*uy+uz*sin(an),     cos(an)+uy^2*(1-cos(an)), (1-cos(an)*uy*uz-ux*sin(an)
%      (1-cos(an))*ux*uz-uy*sin(an), (1-cos(an))*uy*uz+ux*sin(an),    cos(an)+uz^2*(1-cos(an))]
%
% for more information see:
% http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/index.htm
%
% The above expression for R can be written in a compact form as:
% R = eye(3) + tilde(ax)*sin(an) + tilde(ax)^2*(1-cos(an));
%
% where tilde(ax) is a skew-symmetric representation of ax (see tilde.m)

% normalize the axis
ax = ax/norm(ax);

if ( an == 0 )
    R = eye(3);
else
    R = eye(3) + tilde(ax)*sin(an) + tilde(ax)^2*(1-cos(an));
end

%%%EOF
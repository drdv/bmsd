function q = aa2q(an,ax)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% aa2q
%
% Converts axis angle representation to unit quaternion
%
% Syntax:
% -------
% q = aa2q(an,ax)
%
% Input:
% ------
% an [1x1]  - angle [rad]
% ax [3x1]  - axis
%
% Output: 
% -------
% q = [s x y z] - unit quaternion 
%     [s]       - scalar part
%     [x y z]   - vector part
%
% Note: The axis angle representation can be stored in only three 
% parameters if norm(ax) = 1, then a = ax*an whould contain all the 
% necessary information (an = norm(a), ax = a/norm(a)). This however,
% is not used here.
%

% Algorithm:
% ----------
% Follows directly from the definition of quaternion
% q = [cos(an/2) ax*sin(an/2)]

% normalize the axis
ax = ax/norm(ax);

% by the definition of quaternion 
q(1) = cos( an/2 );
q(2:4) = ax * sin( an/2 );

% normalize the quaternion
q = q/norm(q);

%%%EOF
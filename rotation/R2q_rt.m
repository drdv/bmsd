function q = R2q_rt(R)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% R2q_rt
%
% Converts a rotation matrix to unit quaternion
% (The result is the same as when using Q2r.m, only the implementation
% is a bit different)
%
% Syntax:
% -------
% q = R2q_rt(R)
%
% Input:
% ------
% R  - 3x3 rotation matrix 
%
% Output:
% -------
% q = [s x y z] - unit quaternion 
%     [s]       - scalar part
%     [x y z]   - vector part
%

% Algorithm:
% -----------
% I coppied this function from the Robotics toolbox 
% written by Peter I. Corke, for more information see
% http://www.petercorke.com/Robotics%20Toolbox.html
%
% Note: A small modification is made 
%       I check whether trace(R)+1 < eps because for some cases the
%       function was returning complex numbers

% added this for stability reasons 
tmp0 = trace(R)+1;
if tmp0 < eps tmp0 = 0; end

qs = sqrt(tmp0)/2.0;
kx = R(3,2) - R(2,3);	% Oz - Ay
ky = R(1,3) - R(3,1);	% Ax - Nz
kz = R(2,1) - R(1,2);	% Ny - Ox

if (R(1,1) >= R(2,2)) && (R(1,1) >= R(3,3)) 
  kx1 = R(1,1) - R(2,2) - R(3,3) + 1;	        % Nx - Oy - Az + 1
  ky1 = R(2,1) + R(1,2);			% Ny + Ox
  kz1 = R(3,1) + R(1,3);			% Nz + Ax
  add = (kx >= 0);
elseif (R(2,2) >= R(3,3))
  kx1 = R(2,1) + R(1,2);			% Ny + Ox
  ky1 = R(2,2) - R(1,1) - R(3,3) + 1;	        % Oy - Nx - Az + 1
  kz1 = R(3,2) + R(2,3);			% Oz + Ay
  add = (ky >= 0);
else
  kx1 = R(3,1) + R(1,3);			% Nz + Ax
  ky1 = R(3,2) + R(2,3);			% Oz + Ay
  kz1 = R(3,3) - R(1,1) - R(2,2) + 1;	        % Az - Nx - Oy + 1
  add = (kz >= 0);
end

if add
  kx = kx + kx1;
  ky = ky + ky1;
  kz = kz + kz1;
else
  kx = kx - kx1;
  ky = ky - ky1;
  kz = kz - kz1;
end
nm = norm([kx ky kz]);
if nm == 0,
  q = [1 0 0 0];
else
  s = sqrt(1 - qs^2) / nm;
  qv = s*[kx ky kz];
  
  q = [qs qv];
end

%%%EOF
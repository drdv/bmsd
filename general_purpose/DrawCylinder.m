function DrawCylinder(pos, az, radius, len, color)
% 
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | General purpose |
% -------------------
%
% DrawCylinder
%
% Draw a cylinder (used to display the manipulator joints)
%
% Input:
% ------
% pos     - 3D position of the center point
% az      - orientation axis [x,y,z]
% radius  - radius 
% len     - length 
% color   - rgb color (e.g., for red use [1 0 0])
%
% Example:
% --------
% DrawCylinder([0;0;0], [1;1;1], 0.3, 1, [1 0 0]);
%
% Note:
% ------
% I copied this function from Prof. Kajita' tolls
% (to add a reference to them!)
%

%******** rotation matrix
az0 = [0;0;1];
ax  = cross(az0,az);
ax_n = norm(ax);
if ax_n < eps 
  rot = eye(3);
else
  ax = ax/ax_n;
  ay = cross(az,ax);
  ay = ay/norm(ay);
  rot = [ax ay az];
end

%********** make cylinder
a = 20;    % number of side faces
theta = (0:a)/a * 2*pi;

x = [radius; radius]* cos(theta);
y = [radius; radius] * sin(theta);
z = [len/2; -len/2] * ones(1,a+1);

for n=1:size(x,1)
  xyz = [x(n,:);y(n,:);z(n,:)];
  xyz2 = rot * xyz;
  x2(n,:) = xyz2(1,:);
  y2(n,:) = xyz2(2,:);
  z2(n,:) = xyz2(3,:);
end

%************* draw
% cylinder
hSurface = surf(x2+pos(1),y2+pos(2),z2+pos(3));
set(hSurface,'FaceColor',color);

% top and bottom
% set to 1 to close the cylinder
if 0
  for n=1:2
    patch(x2(n,:)+pos(1),y2(n,:)+pos(2),z2(n,:)+pos(3),cc(n,:));
  end	
end

%%%EOF
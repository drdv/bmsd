function draw_frame(varargin)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | General purpose |
% -------------------
%
% draw_frame
%
% Draws a coordinate frame with orientation R at position p
%
% Syntax:
% -------
% draw_frame(R, p, color)
% draw_frame(R, color)
% draw_frame(R, p)
% draw_frame(R)
% draw_frame
%
% Input:
% ------
% R     - rotation matrix OR unit quaternion [s x y z]
% p     - position of the origin
% color - color of the axis
%
% if color == 'rgb' then: 
% X axis - blue
% Y axis - green
% Z axis - red
%

if nargin == 0
  R = eye(3);
  p = [0,0,0]';
  color = 'b';
elseif nargin == 1
  R = varargin{1};
  p = [0,0,0]';
  color = 'b';
elseif nargin == 2
  if ischar(varargin{2})
    R = varargin{1};
    p = [0,0,0]';
    color = varargin{2};
  else
    R = varargin{1};
    p = varargin{2};
    p = p(:);
    color = 'b';
  end
elseif nargin == 3
  R = varargin{1};
  p = varargin{2};
  p = p(:);
  color = varargin{3};
else
  disp('incorrect input')
end

% if R is a unit quaternion convert it to rotation matrix
if length(R) == 4 
  R = q2R(R);
end

if strcmp(color, 'rgb')
  color_flag = 0;
else
  color_flag = 1;
end

cc = {'b', 'g', 'r'};
str = ['X' 'Y' 'Z']';
for i = 1:3
  U(:,1) = p;
  U(:,2) = p + R(:,i);
  v = 0.2*R(:,i);
  if color_flag
    hold on;plot3(U(1,:),U(2,:),U(3,:),color,'LineWidth',2);
  else
    hold on;plot3(U(1,:),U(2,:),U(3,:),cc{i},'LineWidth',2);
  end
  text(U(1,2)+v(1),U(2,2)+v(2),U(3,2)+v(3),str(i),'FontWeight','bold','FontSize',8)
end

axis([p(1)-1 p(1)+1 p(2)-1 p(2)+1 p(3)-1 p(3)+1])
cameratoolbar('NoReset')
cameratoolbar('SetMode','orbit')
grid on

%%%EOF
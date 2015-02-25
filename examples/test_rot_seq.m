%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Examples |
% ------------
%
% test_rot_seq
%
% Demonstrate alternative sequences of Euler angles
%

% casees to test
% case_flag = 1  ->  X-Y-Z (FIXED axis)
% case_flag = 2  ->  X-Y-Z (NEW axis)
% case_flag = 3  ->  Z-Y-X (NEW axis)
case_flag = 2;

% a frame
F = eye(3);

% Some angles
A = 90; % rotation around the X axis
B = 90; % rotation around the Y axis
C = 90; % rotation around the Z axis

% Roatation matrices coresponding to the above angles.
% Positive direction is counter clockwise when looking 
% towards the center (from the top of the axis)
Rz = rz(A*pi/180);
Ry = ry(B*pi/180);
Rx = rx(C*pi/180);

% X-Y-Z (FIXED axis)
% The rotation is performed around the X-Y-Z inertial axis
if case_flag == 1 
  figure(1);
  subplot(2,2,1)
  draw_frame(F, 'rgb');title('Initial orientation')
 
  set(gca,'CameraPosition',[10.3227 -12.3021 6.48838])
  set(gca,'CameraViewAngle',6.60861)
  
  subplot(2,2,2);
  draw_frame(Rx*F, 'rgb');title('Rotated around fixed X')
  
  set(gca,'CameraPosition',[10.3227 -12.3021 6.48838])
  set(gca,'CameraViewAngle',6.60861)
  
  %hold on; draw_frame(F, 'k:') % draw fixed frame
  subplot(2,2,3);
  draw_frame(Ry*Rx*F, 'rgb');title('Rotated around fixed Y')
  
  set(gca,'CameraPosition',[10.3227 -12.3021 6.48838])
  set(gca,'CameraViewAngle',6.60861)
  
  %hold on; draw_frame(F, 'k:') % draw fixed frame
  subplot(2,2,4);title('Rotated around fixed Z')
  draw_frame(Rz*Ry*Rx*F, 'rgb');

  set(gca,'CameraPosition',[10.3227 -12.3021 6.48838])
  set(gca,'CameraViewAngle',6.60861)
end
        
% X-Y-Z (NEW axis)
% The rotation is performed around the axis of the rotated frame
if case_flag == 2 
  figure(2)
  subplot(2,2,1)
  draw_frame(F, 'rgb');title('Initial orientation')
  
  set(gca,'CameraPosition',[10.3227 -12.3021 6.48838])
  set(gca,'CameraViewAngle',6.60861)
   
  subplot(2,2,2);
  draw_frame(Rx*F, 'rgb');title('Rotated around new X')
  
  set(gca,'CameraPosition',[10.3227 -12.3021 6.48838])
  set(gca,'CameraViewAngle',6.60861)
  
  subplot(2,2,3);
  draw_frame(Rx*Ry*F, 'rgb');title('Rotated around new Y')
  
  set(gca,'CameraPosition',[10.3227 -12.3021 6.48838])
  set(gca,'CameraViewAngle',6.60861)
  
  subplot(2,2,4);title('Rotated around new Z')
  draw_frame(Rx*Ry*Rz*F, 'rgb');
  
  set(gca,'CameraPosition',[10.3227 -12.3021 6.48838])
  set(gca,'CameraViewAngle',6.60861)

end

% Z-Y-X (NEW axis)
% === equivalent to === X-Y-Z (FIXED axis) (I mean the final result)
% This is not very intuitive result!
% The rotation is performed around the axis of the rotated frame
if case_flag == 3
  figure(3)
  subplot(2,2,1)
  draw_frame(F, 'rgb');title('Initial orientation')
 
  set(gca,'CameraPosition',[10.3227 -12.3021 6.48838])
  set(gca,'CameraViewAngle',6.60861)
  
  subplot(2,2,2);
  draw_frame(Rz*F, 'rgb');title('Rotated around new Z')
  
  set(gca,'CameraPosition',[10.3227 -12.3021 6.48838])
  set(gca,'CameraViewAngle',6.60861)
  
  subplot(2,2,3);
  draw_frame(Rz*Ry*F, 'rgb');title('Rotated around new Y')
  
  set(gca,'CameraPosition',[10.3227 -12.3021 6.48838])
  set(gca,'CameraViewAngle',6.60861)
  
  subplot(2,2,4);title('Rotated around new X')
  draw_frame(Rz*Ry*Rx*F, 'rgb');
  
  set(gca,'CameraPosition',[10.3227 -12.3021 6.48838])
  set(gca,'CameraViewAngle',6.60861)
end

%%%EOF
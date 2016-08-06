function Draw_System(SP, SV, bN, bP, frame, draw_flag)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | General purpose |
% -------------------
%
% Draw_System
%
% Plots the manipulator system
% IMPORTANT: very slow (do not use during simulation)
%
% Syntax:
% -------
% Draw_System(SP, SV);
% Draw_System(SP, SV, [1], [0;0;0.5], [1:SP.n]);
% Draw_System(SP, SV, [], [], [1:SP.n],  0);
%
% Input:
% ------
% SP          - structure containing system parameters (see struct_def.m)
% SV          - structure containing system variables (see struct_def.m)
% bN  [mx1]   - body numbers (on which are the end-effectors)
% bP  [3xm]   - bP(:,i) is the position of the i-th end-effector in the CF of body bN(i)
% frame       - Displays the CF associated with the joints
%               frame = [1 3 9 ] means that the CF of the 1 3 and 9th joints will be displayed.
% draw_flag   - if 1 use drawnow in the file
%
% Notes:
%
%  The 'R' joints are plotted like circles ('o')
%  The 'P' joints are plotted like diamonds ('d')
%

if nargin < 6
  draw_flag = 1;
end

if nargin < 5
  draw_flag = 1;
  frame = [];
end

if nargin < 3
  draw_flag = 1;
  frame = [];
  bN = [];
end

%=====================
% Visualization flags (if set to zero the primitives are not visualized)
% ----------------------------------------------------------------------

C_flag      = 1; % Centroid of links
J_flag      = 1; % Joints
Tesaki_flag = 1; % End-effector points
Text_flag   = 0; % Joint number is visualized

J2C_flag    = 1; % Line between a joint and centroid of link
C2J_flag    = 1; % Line between centroid of a link and an output joint
C2E_flag    = 1; % Line between link centroid and end-effector point

%=====================

fact = 25;       % Scaling factor for the axis of the CF
line_w = 0.8;    % width of lines
CoM_size = 8;    % size of the CoM
EE_size = 15;    % size of the end-effectors
base_CoM = 12;   % size of the CoM of the base
base_frame = 1;  % plot the base frame

% sometimes I use the settings below
if 0
  CoM_size = 1;
  base_CoM = 1;
  base_frame = 0;
end
%=====================

SV = calc_pos(SP,SV);
pJ = fk_j(SP, SV, 1:SP.n );

%cla
hold on

for iJ = 1:SP.n % iterate over all joints

  iL = iJ+1;         % joint iJ is the "input" joint for link iL
  iS = SP.C(iL);     % link iS is the link that "supports" link iL

  % Plots the position of joint iJ
  if SP.J(iJ).type == 'R'
    J(iJ) = plot3(pJ(1,iJ),pJ(2,iJ),pJ(3,iJ),'ro','MarkerSize',10);
  else
    J(iJ) = plot3(pJ(1,iJ),pJ(2,iJ),pJ(3,iJ),'rd','MarkerSize',7.5);
  end

  % Plots a vector from joint iJ to the CoM of link iL
  J2C(iJ) = plot3([pJ(1,iJ) SV.L(iL).p(1)], [pJ(2,iJ) SV.L(iL).p(2)], [pJ(3,iJ) SV.L(iL).p(3)],'b','LineWidth',line_w);

  % Plots a vector from joint iJ to the CoM of link iS
  C2J(iJ) = plot3([pJ(1,iJ) SV.L(iS).p(1)], [pJ(2,iJ) SV.L(iS).p(2)], [pJ(3,iJ) SV.L(iS).p(3)],'b','LineWidth',line_w);

  % Plot the number of each joint
  Text(iJ) = text(pJ(1,iJ) + 0.1, pJ(2,iJ) + 0.1 , pJ(3,iJ),num2str(iJ),'FontWeight','bold','FontSize',10);

end

try
  for iJ = 1:length(frame) % iterate over all frames to be plotted

    Ri = SV.L(frame(iJ)+1).R;
    PlotFrames_my_in(pJ(:,frame(iJ)), Ri, fact);

    % plot cylinders for the joints
    DrawCylinder(pJ(:,frame(iJ)), Ri(:,3), 0.01, 0.07, 'g')
  end
catch
  sprintf('The values of the entries in the vector "frame" must be >= 1 and <= %d',SP.n)
end

% The base is handled separately because of the different color
C(1) = plot3(SV.L(1).p(1),SV.L(1).p(2),SV.L(1).p(3),'s','MarkerEdgeColor','k', 'MarkerFaceColor','g','MarkerSize',base_CoM);

for iL = 2:SP.n+1 % iterate over all links (except for the base)

  % Plots the position of the CoM of link iL
  C(iL) = plot3(SV.L(iL).p(1),SV.L(iL).p(2),SV.L(iL).p(3),'s','MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',CoM_size);

end

% Plots the end-links
cntr = 1;
for iE = 1:length(bN)  % iterate over all end-effectors

  pE = fk_e( SP, SV, bN(iE), bP(:,iE) );
  C2E(cntr) = plot3([SV.L(bN(iE)).p(1) pE(1)], [SV.L(bN(iE)).p(2) pE(2)], [SV.L(bN(iE)).p(3) pE(3)],'b','LineWidth',line_w);
  Tesaki(cntr) = plot3(pE(1), pE(2), pE(3),'r.','MarkerSize',EE_size);
  cntr = cntr + 1;
end

if base_frame
  PlotFrames_my_in(SV.L(1).p, SV.L(1).R, fact, 'r');
end

%========== Flag management =========
if J2C_flag == 0
  set(J2C,'Visible','off');
end

if C2J_flag == 0
  set(C2J,'Visible','off');
end

if C_flag == 0
  set(C,'Visible','off');
end

if J_flag == 0
  set(J,'Visible','off');
end

if C2E_flag == 0
  set(C2E,'Visible','off');
end

if Tesaki_flag == 0
  set(Tesaki,'Visible','off');
end

if Text_flag == 0 && SP.n > 0
  set(Text,'Visible','off');
end
%=====================================

% draws the plots
if draw_flag
  drawnow
end

function PlotFrames_my_in(R,A,fact,color)
%
%  plots the CF associated with the joints
%

if nargin<4
  cc = {'k','k','r'};
else
  cc = {color,color,color};
end

str = ['X' 'Y' 'Z']';
for i = 1:3
  U(:,1) = R;
  U(:,2) = R + A(:,i)/fact;
  v = 0.3*A(:,i)/fact;
  plot3(U(1,:),U(2,:),U(3,:),'Color',cc{i},'LineWidth',2);
  text(U(1,2)+v(1),U(2,2)+v(2),U(3,2)+v(3),str(i),'FontWeight','bold','FontSize',8)
end

%%%EOF

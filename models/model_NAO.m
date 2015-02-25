function SP = model_NAO()
%
% model of
% NAO humanoid robot
%
% The notation is NOT based of the D-H convention as given in the
% documentation. 
%
% SP.J(i).limits(1) = lower limit for joint i;
% SP.J(i).limits(2) = upper limit for joint i;
% SP.J(i).limits(3) = maximum change per cycle for joint i;
%
% Note 1: LHipYawPitch and RHipYawPitch should be coupled 1:1.
%         (LHipYawPitch is the important one). In the model this is not
%         accounted for.
%
% Note 2: LHand and RHand are implemented as end-effectors
%         (no joint variable is assigned). If I need to include joint
%         variables for the Hands may be I should define them as J(25:26)
%
% Note 3: The parameters J(i).t are first defined assuming that the CoM of
%         the parent link of J(i).t coinsides with the input joint of the
%         parent link. Afterwards the real CoM position is considered.
%         For joints that are connected to the base, the parameter J().t is
%         changed to be from the CoM of the Trunk.
%         In order to keep track of the motion of the Torso, it is defined
%         as an end-effector as well (which hapens to be inside the trunk
%         of NAO)
%
% Note 4: The same (as in Note 3) is done for the end-effectors. First
%         they are defined with respect to the input joint of the link
%         and then an offset is applied to account for the position of the
%         CoM
%
% Note 5: TODO: The inertia parameters of the links are considered to be 
%         trivial (to change this)
%
% Note 6: In the documantation only the position of the CoM for the right
%         leg and arm are given, when assigning the positions for the left 
%         leg and left arm, I made them to be symmetric to the right ones, 
%         so somethimes the "y" coordinate is with oposite sign
%
% 20/Jul/2010
%

% definition of some constants
% ----------------------------------------

NeckOffsetZ     = 126.50/1000;
ShoulderOffsetY =  98.00/1000;
ShoulderOffsetZ = 100.00/1000;
UpperArmLength  =  90.00/1000;
LowerArmLength  =  50.55/1000;
HipOffsetY      =  50.00/1000;
HipOffsetZ      =  85.00/1000;
ThighLength     = 100.00/1000;
TibiaLength     = 102.74/1000;
%FootHeight      =  45.11/1000; % from the documentation
FootHeight      =  45.111/1000; % from Choregraphe (testing)
HandOffsetX     =  58.00/1000;
HandOffsetZ     =  15.90/1000;

% single support polygon
SP.FootX = 145/1000;
SP.FootY = 78/1000;
SP.FootHeight = FootHeight;

SP.Torso2CoMofTrunk = [-4.80, 0.06, 42.27]'/1000;

% definition of system structure
% ----------------------------------------

% Left Leg
LLeg_C = [0 1 2 3 4 5 6];

% Right Leg
RLeg_C = [1 8 9 10 11 12];

% Left Arm
LArm_C = [1 14 15 16 17];

% Right Arm
RArm_C = [1 19 20 21 22];

% Head
Head_C = [1 24];

SP.C = [LLeg_C, RLeg_C, LArm_C, RArm_C, Head_C];
SP.n = length(SP.C)-1;
SP.mode = 0; % Trunk is free floating

% definition of joints
% ----------------------------------------

% Left Leg
% ----------
%LHipYawPitch
SP.J(1).t    = [  0.0   HipOffsetY  -HipOffsetZ  ]';
SP.J(1).f    = rx(3*pi/4)*[ -7.17     -11.87     27.05 ]'/1000;
SP.J(1).rpy  = [ -3/4*pi     0.0      0.0 ]';
SP.J(1).limits = [-1.145303,  0.740810,  0.083078]';
SP.J(1).type = 'R';

%LHipRoll
SP.J(2).t    = [  0.0     0.0      0.0 ]';
SP.J(2).f    = rx(pi/2)*ry(pi/4)*rz(pi/2)*[  -16.49     0.29      -4.75 ]'/1000;
SP.J(2).rpy  = [  0.0    pi/2      0.0 ]';
SP.J(2).limits = [-0.379472,  0.790477,  0.083078]';
SP.J(2).type = 'R';

%LHipPitch
SP.J(3).t    = [  0.0     0.0      0.0 ]';
SP.J(3).f    = [-53.86   1.31      2.01]'/1000;
SP.J(3).rpy  = [ pi/2    -pi/4      0.0 ]';
SP.J(3).limits = [-1.773912,  0.484090,  0.127933]';
SP.J(3).type = 'R';

%LKneePitch
SP.J(4).t    = [ -ThighLength    0.0      0.0 ]';
SP.J(4).f    = [  -48.91    4.71    2.10]'/1000;
SP.J(4).rpy  = [  0.0     0.0      0.0 ]';
SP.J(4).limits = [-0.092346,  2.112528,  0.127933]';
SP.J(4).type = 'R';

%LAnklePitch
SP.J(5).t    = [ -TibiaLength     0.0      0.0 ]';
SP.J(5).f    = [  6.38     1.42      0.28 ]'/1000;
SP.J(5).rpy  = [  0.0     0.0      0.0 ]';
SP.J(5).limits = [-1.189516,  0.922747,  0.127933]';
SP.J(5).type = 'R';

%LAnkleRoll
SP.J(6).t    = [  0.0     0.0      0.0 ]';
SP.J(6).f    = [  -32.08    -3.30     24.89]'/1000;
SP.J(6).rpy  = [ -pi/2    0.0      0.0 ]';
SP.J(6).limits = [-0.769001,  0.397880,  0.083078]';
SP.J(6).type = 'R';

% Right Leg
% ----------
%RHipYawPitch
SP.J(7).t    = [  0.0   -HipOffsetY  -HipOffsetZ  ]';
SP.J(7).f    = rx(pi/4)*[ -7.17     11.87     27.05 ]'/1000;
SP.J(7).rpy  = [ -1/4*pi     0.0      0.0 ]';
SP.J(7).limits = [-1.145303,  0.740810,  0.083078]';
SP.J(7).type = 'R';

%RHipRoll
SP.J(8).t    = [  0.0     0.0      0.0 ]';
SP.J(8).f    = rx(pi/2)*ry(-pi/4)*rz(pi/2)*[  -16.49     -0.29      -4.75 ]'/1000;
SP.J(8).rpy  = [  0.0    pi/2      0.0 ]';
SP.J(8).limits = [-0.738321,  0.414754,  0.083078]';
SP.J(8).type = 'R';

%RHipPitch
SP.J(9).t    = [  0.0     0.0      0.0 ]';
SP.J(9).f    = [53.86   -1.31    -2.01]'/1000;
SP.J(9).rpy  = [ -pi/2    -pi/4      0.0 ]';
SP.J(9).limits = [-1.772308,  0.485624,  0.127933]';
SP.J(9).type = 'R';

%RKneePitch
SP.J(10).t    = [  ThighLength    0.0      0.0 ]';
SP.J(10).f    = [48.91   -4.71    -2.10]'/1000;
SP.J(10).rpy  = [  0.0     0.0      0.0 ]';
SP.J(10).limits = [-0.103083,  2.120198,  0.127933]';
SP.J(10).type = 'R';

%RAnklePitch
SP.J(11).t    = [  TibiaLength     0.0      0.0 ]';
SP.J(11).f    = [  -6.38     -1.42      -0.28 ]'/1000;
SP.J(11).rpy  = [  0.0     0.0      0.0 ]';
SP.J(11).limits = [-1.186448,  0.932056,  0.127933]';
SP.J(11).type = 'R';

%RAnkleRoll
SP.J(12).t    = [  0.0     0.0      0.0 ]';
SP.J(12).f    = [  32.08     -3.30      24.89 ]'/1000;
SP.J(12).rpy  = [ pi/2    0.0      0.0 ]';
SP.J(12).limits = [-0.388676,  0.785875,  0.083078]';
SP.J(12).type = 'R';

% Left Arm
% ----------
%LShoulderPitch
SP.J(13).t    = [  0.0   ShoulderOffsetY  ShoulderOffsetZ  ]';
SP.J(13).f    = [ -1.78      -0.19       -25.07 ]'/1000;
SP.J(13).rpy  = [ -pi/2     0.0      0.0 ]';
SP.J(13).limits = [-2.085600,  2.085600,  0.165283]';
SP.J(13).type = 'R';

%LShoulderRoll
SP.J(14).t    = [  0.0     0.0      0.0  ]';
SP.J(14).f    = [  20.67     3.88      3.62 ]'/1000;
SP.J(14).rpy  = [  pi/2     0.0      0.0 ]';
SP.J(14).limits = [ 0.008700,  1.649400,  0.143815]';
SP.J(14).type = 'R';

%LElbowYaw
SP.J(15).t    = [  UpperArmLength     0.0      0.0  ]';
SP.J(15).f    = [  0.20     -0.01      -25.73 ]'/1000;
SP.J(15).rpy  = [  0.0     pi/2      0.0 ]';
SP.J(15).limits = [-2.085600,  2.085600,  0.165283]';
SP.J(15).type = 'R';

%LElbowRoll
SP.J(16).t    = [  0.0     0.0      0.0  ]';
SP.J(16).f    = [  19.40     3.04      2.50 ]'/1000;
SP.J(16).rpy  = [  0.0    -pi/2      0.0 ]';
SP.J(16).limits = [-1.562100, -0.008700,  0.143815]';
SP.J(16).type = 'R';

%LWristYaw
SP.J(17).t    = [  LowerArmLength     0.0      0.0  ]';
SP.J(17).f    = [2.72 2.68 32.41]'/1000;
SP.J(17).rpy  = [  0.0    pi/2      0.0 ]';
SP.J(17).limits = [-1.823800,  1.823800,  0.143815]';
SP.J(17).type = 'R';

% Right Arm
% ----------
%RShoulderPitch
SP.J(18).t    = [  0.0   -ShoulderOffsetY  ShoulderOffsetZ  ]';
SP.J(18).f    = [ -1.78      -0.19       25.07 ]'/1000;
SP.J(18).rpy  = [ -pi/2     0.0      0.0 ]';
SP.J(18).limits = [-2.085600,  2.085600,  0.165283]';
SP.J(18).type = 'R';

%RShoulderRoll
SP.J(19).t    = [  0.0     0.0      0.0  ]';
SP.J(19).f    = [  20.67     -3.88      3.62 ]'/1000;
SP.J(19).rpy  = [ pi/2     0.0      0.0 ]';
SP.J(19).limits = [-1.649400, -0.008700,  0.143815]';
SP.J(19).type = 'R';

%RElbowYaw
SP.J(20).t    = [  UpperArmLength     0.0      0.0  ]';
SP.J(20).f    = [  0.20     0.01      -25.73 ]'/1000;
SP.J(20).rpy  = [  0.0     pi/2      0.0 ]';
SP.J(20).limits = [-2.085600,  2.085600,  0.165283]';
SP.J(20).type = 'R';

%RElbowRoll
SP.J(21).t    = [  0.0     0.0      0.0  ]';
SP.J(21).f    = [  19.40     -3.04      2.50 ]'/1000;
SP.J(21).rpy  = [  0.0    -pi/2      0.0 ]';
SP.J(21).limits = [ 0.008700,  1.562100,  0.143815]';
SP.J(21).type = 'R';

%RWristYaw
SP.J(22).t    = [  LowerArmLength     0.0      0.0  ]';
SP.J(22).f    = [2.72 -2.68 32.41]'/1000;
SP.J(22).rpy  = [  0.0    pi/2      0.0 ]';
SP.J(22).limits = [-1.823800,  1.823800,  0.143815]';
SP.J(22).type = 'R';

% Head
% ----------
% HeadYaw
SP.J(23).t    = [  0.0     0.0      NeckOffsetZ  ]';
SP.J(23).f    = [  -0.03     0.18      -25.73 ]'/1000;
SP.J(23).rpy  = [  0.0     0.0      0.0 ]';
SP.J(23).limits = [-2.085700,  2.085700,  0.165283]';
SP.J(23).type = 'R';

% HeadPitch
SP.J(24).t    = [  0.0     0.0      0.0  ]';
SP.J(24).f    = [  3.83     -51.56      0.93 ]'/1000;
SP.J(24).rpy  = [ -pi/2     0.0      0.0 ]';
SP.J(24).limits = [-0.672000,  0.514900,  0.143815]';
SP.J(24).type = 'R';

% to remove this when
for i=1:SP.n
    if i == 1
        fprintf('\n\n WARNING: SP.J(i).f and SP.Torso2CoMofTrunk set equal to zeros(3,1)!\n\n');
    end
    SP.J(i).f  = zeros(3,1);
    SP.Torso2CoMofTrunk = zeros(3,1);
end

% account for the position of the CoM
for iJ=SP.n:-1:1
    iL = iJ+1;      % joint iJ is the "input" joint for link iL
    iS = SP.C(iL);  % link iS is the link that "supports" link iL
    iJ_S = iS-1; % The input joint of link iS
    
    if iS ~= 0
        if iS ~= 1
            SP.J(iJ).t = SP.J(iJ).t - SP.J(iJ_S).f;
        else
            SP.J(iJ).t = SP.J(iJ).t - SP.Torso2CoMofTrunk;
        end
    end
end

% definition of links
% ----------------------------------------
for iL = 1:SP.n+1
    % TODO: use realistic inertia parameters
    SP.L(iL).I = eye(3);
end

SP.L(1).m = 1026.28/1000;

SP.L(2).m = 72.44/1000;
SP.L(3).m = 135.30/1000;
SP.L(4).m = 397.98/1000;
SP.L(5).m = 297.06/1000;
SP.L(6).m = 138.92/1000;
SP.L(7).m = 163.04/1000;

SP.L(8).m = SP.L(2).m;
SP.L(9).m = SP.L(3).m;
SP.L(10).m = SP.L(4).m;
SP.L(11).m = SP.L(5).m;
SP.L(12).m = SP.L(6).m;
SP.L(13).m = SP.L(7).m;

SP.L(14).m = 69.84/1000;
SP.L(15).m = 121.66/1000;
SP.L(16).m = 59.59/1000;
SP.L(17).m = 37.70/1000;
SP.L(18).m = 143.14/1000;

SP.L(19).m = SP.L(14).m;
SP.L(20).m = SP.L(15).m;
SP.L(21).m = SP.L(16).m;
SP.L(22).m = SP.L(17).m;
SP.L(23).m = SP.L(18).m;

SP.L(24).m = 59.59/1000;
SP.L(25).m = 476.71/1000;

% definition of end-effectors
% ----------------------------------------
SP.bN = [7, 13, 18, 23, 25, 1];
SP.bP(:,1) = [-FootHeight 0 0]';           % LLeg
SP.bP(:,2) = [FootHeight 0 0]';            % RLeg
SP.bP(:,3) = [HandOffsetZ 0 HandOffsetX]'; % LHand
SP.bP(:,4) = [HandOffsetZ 0 HandOffsetX]'; % RHand
SP.bP(:,5) = [0 0 0]';                     % Head
SP.bP(:,6) = -SP.Torso2CoMofTrunk;         % Torso

for i=1:length(SP.bN);
    iL = SP.bN(i); % link where the end-effector is located
    iJ = iL-1;     % input joint of link iL
    if iL ~= 1
        SP.bP(:,i) = SP.bP(:,i) - SP.J(iJ).f;
    end
end

% orientation offset for the end-effectors
SP.bR{1} = rx(pi/2)*ry(pi/2)*rz(pi/2);
SP.bR{2} = ry(-pi/2);
SP.bR{3} = ry(-pi/2);
SP.bR{4} = ry(-pi/2);
SP.bR{5} = rx(pi/2);
SP.bR{6} = eye(3);

%%%EOF
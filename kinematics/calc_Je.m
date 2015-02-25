function [Je,joints] = calc_Je(SP,SV,bN,bP)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% calc_Je
% 
% Calculates the Jacobian matrix of an end-effector point 
% with respect to the manipulator joints 
%
% Note: This function requires updated positions SV = calc_pos(SP,SV);
%
% Syntax:
% -------
% [Je,joints] = calc_Je(SP,SV,bN,bP);
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% SV    - structure containing system variables (see struct_def.m)
% bN  [1x1]  - body number (on which is the end-effector) 
% bP  [3x1]  - position of the end-effector in the CF of body bN
%
% Output:
% -------
% Je [6xSP.n]   - Jacobian matrix of the end-effector wrt the manipulator joints
% joints [1x?]  - joints that affect the motion of the end-effector
%
% SV members changed:
% --------------------
%

if bN <=0 || bN > SP.n+1 
  disp('ERROR: bN is out of range');
  return
end

joints = joint_tree(SP, bN); % joints from link 1 (the base) to the end-effector
n = length(joints); % number of joints 

pJ = fk_j(SP,SV,joints); % positions of joints to the end-effector
pE = fk_e(SP,SV,bN,bP); % position of the end-effector in inertial frame

Je = zeros(6,SP.n); % initialization of the output

for i = 1:n % iterate over the components of vector joints
  
  iJ = joints(i); % the i-th joint number is iJ
  iL = iJ+1;      % joint iJ is the "input" joint for link iL
  k = SV.L(iL).R(:,3); % axis of rotation/translation of joint iJ (expreessed in the "world" frame)  
  
  if SP.J(iJ).type == 'R' % Revolute joint
    Je(1:3,iJ) = cross( k , ( pE - pJ(:,i) ) );
    Je(4:6,iJ) = k;
  else % Prismatic joint
    Je(1:3,iJ) = k;
    Je(4:6,iJ) = [ 0 0 0 ]';
  end
  
end

%%%EOF

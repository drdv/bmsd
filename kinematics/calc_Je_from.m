function [Je, joints] = calc_Je_from(SP, SV, bN, bP, base_link)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% calc_Je_from
% 
% Calculates the Jacobian matrix of an end-effector assuming that base_link 
% is the "base" link. Usually it would be Link 1, but in certain 
% cases we would like to use a different link as a "base link".
% (see calc_pos_from.m)
%
% Note: This function requires updated positions SV = calc_pos_from(SP,SV,base_link)
%
% Syntax:
% -------
% [Je, joints] = calc_Je_from(SP, SV, bN, bP, base_link);
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% SV    - structure containing system variables (see struct_def.m)
% bN  [1x1]  - body number (on which is the end-effector) 
% bP  [3x1]  - position of the end-effector in the CF of body bN
% base_link  - usually the "base" link is Link 1, but in certain cases we would like to 
%              use a different link as a "base" link
%
% Output:
% -------
% Je [6xSP.n]   - Jacobian matrix of the end-effector wrt the manipulator joints
% joints [1x?]  - joints that affect the motion of the end-effector assumig
%                 that base_link is the "base" link
%
% SV members changed:
% --------------------
%

if bN <=0 || bN > SP.n+1 
  disp('ERROR: bN is out of range');
  return
end

% ----------------------------------------------
% Find the joint between two links (end_link, base_link)
% I assume that base_link is the link that is going to be used as "the new base" 
% Result:
% -------
% joints  - contains the joints between end_link and base_link in ascending order
% base_link_j - contains the joints from base_link to the "old base"
%
% I need base_link_j, because I need to change the sign of these joints 
% ----------------------------------------------
end_link_j = joint_tree(SP,bN);
base_link_j = joint_tree(SP,base_link);
joints_union = union(end_link_j, base_link_j);
joints_intersect = intersect(end_link_j, base_link_j);
joints = setdiff(joints_union, joints_intersect);
% ----------------------------------------------

n = length(joints); % number of joints between end_link and base_link

pJ = fk_j(SP,SV,joints); % positions of joints 
pE = fk_e(SP,SV,bN,bP); % position of the end-effector in inertial frame

Je = zeros(6,SP.n); % initialization of the output

for i = 1:n % iterate over the components of vector joints
  
  iJ = joints(i); % the i-th joint number is iJ
  iL = iJ+1;      % joint iJ is the "input" joint for link iL

  % axis of rotation/translation of joint iJ (expreessed in the "world" frame) 
  if ismember(iJ, base_link_j)
      k = -SV.L(iL).R(:,3);
  else
      k = SV.L(iL).R(:,3);
  end
  
  if SP.J(iJ).type == 'R' % Revolute joint
    Je(1:3,iJ) = cross( k , ( pE - pJ(:,i) ) );
    Je(4:6,iJ) = k;
  else % Prismatic joint
    Je(1:3,iJ) = k;
    Je(4:6,iJ) = [ 0 0 0 ]';
  end
  
end

%%%EOF
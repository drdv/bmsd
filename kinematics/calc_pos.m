function SV = calc_pos(SP,SV)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% calc_pos
%
% Updates SV.L(2:SP.n+1).R and SV.L(2:SP.n+1).p
%
% Syntax:
% -------
% SV=calc_pos(SP,SV);
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% SV    - structure containing system variables (see struct_def.m)
%
% Output:
% -------
% SV
%
% SV members changed:
% --------------------
% SV.L(2:SP.n+1).R
% SV.L(2:SP.n+1).p
%

for iJ = 1:SP.n % iterate over all joints
 
  iL = iJ+1;         % joint iJ is the "input" joint for link iL
  iS = SP.C(iL);     % link iS is the link that "supports" link iL  
  
  % ------------------
  % update orientation
  % ------------------
  
  % Account for the angle of revolute joints
  % Note that: the Z axis of the CF fixed in a joint is assumed to be the joint's axis of rotation
  if SP.J(iJ).type == 'R'
    SP.J(iJ).rpy(3) = SP.J(iJ).rpy(3) + SV.q(iJ); % SP is changed only locally 
  end
  
  SV.L(iL).R = SV.L(iS).R * rpy2R(SP.J(iJ).rpy);
 
  % ----------------------
  % update position of CoM
  % ----------------------
  SV.L(iL).p = SV.L(iS).p + SV.L(iS).R*SP.J(iJ).t + SV.L(iL).R*SP.J(iJ).f;

  % Account for the length of prismatic joints
  % Note that: SV.L(iL).R(:,3) stands for SV.L(iL).R*[0;0;1] (where [0;0;1] is the joint axis of translation)
  if SP.J(iJ).type == 'P' % Prismatic joint 
    SV.L(iL).p = SV.L(iL).p + SV.L(iL).R(:,3)*SV.q(iJ);
  end
    
end

%%%EOF
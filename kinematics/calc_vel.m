function SV = calc_vel(SP,SV)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% calc_vel
%
% Updates SV.L(2:SP.n+1).w and SV.L(2:SP.n+1).v
%
% Note: This function requires updated positions SV = calc_pos(SP,SV);
%
% Syntax:
% -------
% SV=calc_vel(SP,SV);
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
% SV.L(2:SP.n+1).w
% SV.L(2:SP.n+1).v
%

for iJ = 1:SP.n % iterate over all joints

  iL = iJ+1;         % joint iJ is the "input" joint for link iL
  iS = SP.C(iL);     % link iS is the link that "supports" link iL  
  
  k = SV.L(iL).R(:,3); % joint axis or rotation/translation expressed in the "world" frame
  d = SV.L(iL).R*SP.J(iJ).f; % vector from joint iJ to CoM of link iL expressed in the "world"
                             % frame (in case of a revolute joint)
  
  if SP.J(iJ).type == 'R' % Revolute joint      
    
    SV.L(iL).w = SV.L(iS).w + k*SV.dq(iJ);
    SV.L(iL).v = SV.L(iS).v + cross(SV.L(iS).w, SV.L(iL).p-SV.L(iS).p) + cross(k,d)*SV.dq(iJ);
      
  else % Prismatic joint
    
    SV.L(iL).w = SV.L(iS).w;
    SV.L(iL).v = SV.L(iS).v + cross(SV.L(iS).w, SV.L(iL).p-SV.L(iS).p) + k*SV.dq(iJ);
    
  end
  
end


%%%EOF


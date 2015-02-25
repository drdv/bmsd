function SV = calc_acc(SP,SV)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% calc_acc
%
% Updates SV.L(2:SP.n+1).dw and SV.L(2:SP.n+1).dv
%
% Note: This function requires updated positions and velocities 
% SV = calc_pos(SP,SV); SV = calc_vel(SP,SV);
%
% Syntax:
% -------
% SV=calc_acc(SP,SV);
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
% SV.L(2:SP.n+1).dw
% SV.L(2:SP.n+1).dv
%

for iJ = 1:SP.n % iterate over all joints

  iL = iJ+1;         % joint iJ is the "input" joint for link iL
  iS = SP.C(iL);     % link iS is the link that "supports" link iL  
  
  k = SV.L(iL).R(:,3); % joint axis of rotation/translation expressed in the "world" frame
  d = SV.L(iL).R*SP.J(iJ).f; % vector from joint iJ to CoM of link iL expressed in the "world"
                             % frame (in case if a revolute joint)
 
  kXd = cross(k,d);
     
  if SP.J(iJ).type == 'R' % Revolute joint      
    
    SV.L(iL).dw = SV.L(iS).dw + cross(SV.L(iS).w,k)*SV.dq(iJ) + k*SV.ddq(iJ);
    SV.L(iL).dv = SV.L(iS).dv + cross(SV.L(iS).dw, SV.L(iL).p-SV.L(iS).p) + cross(SV.L(iS).w, SV.L(iL).v-SV.L(iS).v) + ...
        cross(SV.L(iS).w+k*SV.dq(iJ), kXd)*SV.dq(iJ) + kXd*SV.ddq(iJ);
    
  else % Prismatic joint
    
    SV.L(iL).dw = SV.L(iS).dw;
    SV.L(iL).dv = SV.L(iS).dv + cross(SV.L(iS).dw, SV.L(iL).p-SV.L(iS).p) + k*SV.ddq(iJ) + ... 
        cross(SV.L(iS).w, SV.L(iL).v-SV.L(iS).v + k*SV.dq(iJ));
    
  end
  
end


%%%EOF


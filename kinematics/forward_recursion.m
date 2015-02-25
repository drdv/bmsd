function SV=forward_recursion(SP,SV)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% forward_recursion
%
% Updates SV.L(2:SP.n+1).w, SV.L(2:SP.n+1).v, SV.L(2:SP.n+1).dw and SV.L(2:SP.n+1).dv
% (implementing calc_vel.m and calc_acc.m in one file)
%
% Note: This function requires updated positions SV = calc_pos(SP,SV);
%
% It is assumed that:
% SV.ddq = zeros(SP.n,1);
% SV.L(1).dv = zeros(3,1);
% SV.L(1).dw = zeros(3,1);
%
% Syntax:
% -------
% SV=forward_recursion(SP,SV);
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
% SV.L(2:SP.n+1).dw
% SV.L(2:SP.n+1).dv
% SV.L(1).dv and SV.L(1).dw are set equal to zero
%

SV.L(1).dv = zeros(3,1);
SV.L(1).dw = zeros(3,1);

for iJ = 1:SP.n % iterate over all joints

  iL = iJ+1;         % joint iJ is the "input" joint for link iL
  iS = SP.C(iL);     % link iS is the link that "supports" link iL  
  
  k = SV.L(iL).R(:,3); % joint axis of rotation/translation expressed in the "world" frame
  d = SV.L(iL).R*SP.J(iJ).f; % vector from joint iJ to CoM of link iL expressed in the "world"
                             % frame  (in case of a revolute joint)
  
  kXd = cross(k,d);
  
  if SP.J(iJ).type == 'R' % Revolute joint
    
    SV.L(iL).w = SV.L(iS).w + k*SV.dq(iJ);
    SV.L(iL).v = SV.L(iS).v + cross(SV.L(iS).w, SV.L(iL).p-SV.L(iS).p) + kXd*SV.dq(iJ);
    
    SV.L(iL).dw = SV.L(iS).dw + cross(SV.L(iS).w,k)*SV.dq(iJ);
    SV.L(iL).dv = SV.L(iS).dv + cross(SV.L(iS).dw, SV.L(iL).p-SV.L(iS).p) + cross(SV.L(iS).w, SV.L(iL).v-SV.L(iS).v) + ...
        cross(SV.L(iL).w, kXd)*SV.dq(iJ);
    
  else % Prismatic joint
    
    SV.L(iL).w = SV.L(iS).w;
    SV.L(iL).v = SV.L(iS).v + cross(SV.L(iS).w, SV.L(iL).p-SV.L(iS).p) + k*SV.dq(iJ);
    
    SV.L(iL).dw = SV.L(iS).dw;
    SV.L(iL).dv = SV.L(iS).dv + cross(SV.L(iS).dw, SV.L(iL).p-SV.L(iS).p) + ... 
        cross(SV.L(iS).w, SV.L(iL).v-SV.L(iS).v + k*SV.dq(iJ));
    
  end
  
end

function [dJe,joints] = calc_dJe(SP,SV,bN,bP)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% calc_dJe
% 
% Calculates the derivative w.r.t. time of Je (see calc_Je.m) assuming a fixed base system
%
% Note: This function requires updated velocities SV = calc_vel(SP,SV);
%
% Note: The explicit computation of dJe is almost never performed. 
%       The implementation is very inefficient.
%       No other function in this toolbox depends on this function.       
%
% Syntax:
% -------
% [dJe,joints] = calc_dJe(SP,SV,bN,bP);
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
% dJe [6xSP.n]  - derivative of Je
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
n = length(joints);          % number of joints 

pJ = fk_j(SP,SV,joints);     % positions of joints in inertial frame
pE = fk_e(SP,SV,bN,bP);      % position of the end-effector in inertial frame

[Je,joints] = calc_Je(SP,SV,bN,bP);
vE = Je(1:3,joints)*SV.dq(joints); % velocity of end-effector due to joint velocity

dJe = zeros(6,SP.n); % initialization of the output

for i = 1:n % iterate over the components of vector joints
  
  iJ = joints(i);      % the i-th joint number is iJ
  iL = iJ+1;           % joint iJ is the "input" joint for link iL
  k = SV.L(iL).R(:,3); % axis of rotation/translation of joint iJ (expreessed in the "world" frame)  
  
  w = SV.L(iL).w - SV.L(1).w; % subtract the angular velocity of the base (assume that the base is fixed)
  if SP.J(iJ).type == 'R'     % Revolute joint
   
      % "-SP.J(iJ).f" is needed in order to go from the CoM of link iL to the joint iJ
      [Jj,jj] = calc_Je(SP,SV,iL,-SP.J(iJ).f);
      vJoint = Jj(1:3,jj)*SV.dq(jj); % velocity of joint iJ due to joint velocity
      
      dJe(1:3,iJ) = cross(cross(w,k),pE - pJ(:,i)) + cross(k,vE - vJoint);
      dJe(4:6,iJ) = cross(w,k);
  else % Prismatic joint
      dJe(1:3,iJ) = cross(w,k);
      dJe(4:6,iJ) = [ 0 0 0 ]';
  end
  
end

%%%EOF

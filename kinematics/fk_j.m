function p = fk_j(SP,SV,joints)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% fk_j
%
% Calculates the position of n manipulator joints
%
% Note: This function requires updated positions SV = calc_pos(SP,SV);
%
% Syntax:
% -------
% p=fk_j(SP,SV,joints);
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% SV    - structure containing system variables (see struct_def.m)
% joints [nx1] - vector of n joint numbers 
%
% Output:
% -------
% p  [3xn]   - positions of the n joints in the vector "joints"
%
% SV members changed:
% --------------------
%

n = length(joints); % number of joints
p = zeros(3,n);

for i = 1:n % iterate over the components of vector joints
  
  iJ = joints(i); % the i-th joint number is iJ
  iL = iJ+1;      % joint iJ is the "input" joint for link iL
  iS = SP.C(iL);  % link iS is the link that "supports" link iL  
   
  p(:,i) = SV.L(iS).p + SV.L(iS).R * SP.J(iJ).t;
  
end

%%%EOF
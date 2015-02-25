function Rg = calc_CoM(SP,SV)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Dynamics |
% ------------
%
% calc_CoM
%
% Computes the Center of Mass of the system
% 
% Note: This function requires updated positions SV = calc_pos(SP,SV);
%
% Syntax:
% -------
% Rg = calc_CoM(SP,SV)
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% SV    - structure containing system variables (see struct_def.m)
%
% Output:
% -------
%
% Rg [3x1]     - CoM of the system.
%
% SV members changed:
% --------------------
%

Rm = SP.L(1).m * SV.L(1).p;
mass = SP.L(1).m;
for i = 2:SP.n+1
  Rm = Rm + SP.L(i).m * SV.L(i).p;
  mass = mass + SP.L(i).m;
end
Rg = Rm/mass;

%%%EOF
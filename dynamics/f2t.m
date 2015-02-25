function T = f2t(SV,bN,bP,bF)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Dynamics |
% ------------
%
% f2h
%
% Computes the torque that should be applied to the
% CoM of link bN, that results from applying a force bF (expressed in the 
% inertial frame) in point bP (expressed in the local frame of link bN)
% 
% Note: This function requires updated positions SV = calc_pos(SP,SV);
%
% Syntax:
% -------
% T = f2t(SV,bN,bP,bF);
%
% Input:
% ------
% SV         - structure containing system variables (see struct_def.m) 
% bN  [1x1]  - link number
% bP  [3x1]  - position vector expressed in the frame fixed in link bN 
% bF  [3x1]  - external force (expressed in the inertial frame) applied to bP 
%
% Output:
% -------
% T   [3x1]  - external torque to be applied to the CoM of link bN as a result of bF        
%
% Note:
%       Requires updated matrix SV.L(i).R (see calc_pos.m)
%
% SV members changed:
% --------------------
%

T = cross( SV.L(bN).R*bP , bF );

%%%EOF

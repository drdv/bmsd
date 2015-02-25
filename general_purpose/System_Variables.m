function SV = System_Variables(SP)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | General purpose |
% -------------------
%
% System_Variables
%
% Manipulator system variables initialization
%
% Syntax:
% -------
% SV=System_Variables(SP);
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
% all
%

SV.q   = zeros(SP.n,1);
SV.dq  = zeros(SP.n,1);
SV.ddq = zeros(SP.n,1);
SV.tau = zeros(SP.n,1);

for iL = 1:SP.n+1
    SV.L(iL).R  = eye(3);
    SV.L(iL).Q  = R2q(SV.L(iL).R);
    SV.L(iL).p  = [ 0 0 0 ]';
    SV.L(iL).v  = [ 0 0 0 ]';
    SV.L(iL).dv = [ 0 0 0 ]';
    SV.L(iL).w  = [ 0 0 0 ]';
    SV.L(iL).dw = [ 0 0 0 ]';
    SV.L(iL).T  = [ 0 0 0 ]';
    SV.L(iL).F  = [ 0 0 0 ]';
end

%%% EOF
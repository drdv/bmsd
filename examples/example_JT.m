% T = J'*F

clear;clc

Gravity = [0;0;0];

% system definition
SP = model_ABB();
SV = System_Variables(SP);

SV.q  = rand(SP.n,1);

% updates positions of links
SV = calc_pos(SP,SV); 

% generate random force & torque
FT = rand(6,1)*10;

% Compute the Jacobiam 
J = calc_Je(SP,SV,SP.bN,SP.bP); 

% the minus sign is because we are trying to compensate the force
SV.tau = -J'*FT;

SV.L(7).F = FT(1:3);
SV.L(7).T = FT(4:6);
SV.L(7).T = SV.L(7).T + f2t(SV,SP.bN,SP.bP,FT(1:3));

% the second term above accounts for the torque as a result of FT(1:3)
% if SP.bP = [0;0;0] we dont need it

SV = f_dyn(SP,SV,Gravity);

FT
SV.ddq

%%%EOF
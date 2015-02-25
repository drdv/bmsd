%
% test calc_dJmm and calc_dJe
%
% Note: this is not the derivative of the Jacobian for the free-floating system 
% (but for the fixed-base one)
%

clear;clc

% define a system 
SP = model_test();
%SP = model_ABB();

SV = System_Variables(SP);

% set the state to something (misleading: essentially, in calc_dJmm.m I zero-out the base velocity)
SV.L(1).R = rx(randn)*ry(randn)*rz(randn);
SV.L(1).p = randn(3,1);
SV.L(1).v = randn(3,1);
SV.L(1).w = randn(3,1);

SV.q = randn(SP.n,1);
SV.dq = randn(SP.n,1);
SV.ddq = randn(SP.n,1);

% update positions and velocities
SV = calc_pos(SP,SV);
SV = calc_vel(SP,SV);

% Compute Jacobina matrices
Je = calc_Je(SP,SV,SP.bN,SP.bP);
Jmm = calc_Jmm(SP,SV);

% Compute derivative w.r.t time of the Jacobina matrices
dJe = calc_dJe(SP,SV,SP.bN,SP.bP);
dJmm = calc_dJmm(SP,SV);

% -------------------------------------------
% verify dJe using numerical differentiation
% -------------------------------------------

Je_eps = zeros(6,SP.n);
q = SV.q;
epsilon = sqrt(eps);
for i=1:SP.n
    
    SV.q = q;
    SV.q(i) = SV.q(i) + epsilon; % perturb i-th joint angle
    SV = calc_pos(SP,SV);
    tmp = calc_Je(SP,SV,SP.bN,SP.bP);
    Je_eps = Je_eps + SV.dq(i)*(tmp - Je)/epsilon;

end

norm(Je_eps - dJe)

% -------------------------------------------
% verify dJmm using numerical differentiation
% -------------------------------------------

J_eps = zeros(6*SP.n,SP.n);
for i=1:SP.n
    
    SV.q = q;
    SV.q(i) = SV.q(i) + epsilon; % perturb i-th joint angle
    SV = calc_pos(SP,SV);
    tmp = calc_Jmm(SP,SV);
    J_eps = J_eps + SV.dq(i)*(tmp - Jmm)/epsilon;

end

norm(J_eps - dJmm)

% ----------------------------------------

%%%EOF
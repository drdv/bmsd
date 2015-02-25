clear;clc

Gravity = [0;0;-9.81]; 

% system definition
SP = model_7dof();
SV = System_Variables(SP);

% ------------------------------
% generate random initial state
% ------------------------------
SV.q  = rand(SP.n,1);
SV.dq = rand(SP.n,1);
SV.L(1).R = rpy2R(rand(3,1));
SV.L(1).p = rand(3,1);
SV.L(1).v = rand(3,1);
SV.L(1).w = rand(3,1);
% ------------------------------

% compute forward kinematics  
SV = calc_pos(SP,SV);

% Compute the augmented Jacobian R
[H_bar,R] = calc_hh_a(SP,SV);

% ------------------------------
% specify desired accelerations
% ------------------------------
ddq=[1;2;3;4;5;6;7;8;9;0];
dv=[1;2;3];
dw=[4;5;6];
% ------------------------------

% ------------------------------
% solve the inverse dynamics 
% ------------------------------
F_id = r_ne1_a(SP,SV,R,Gravity,ddq,dv,dw);
% ------------------------------

% ------------------------------
% apply joint and base forces/torques
% ------------------------------
SV.L(1).F = F_id(1:3);
SV.L(1).T = F_id(4:6);
SV.tau = F_id(7:end);

% solve the forward dynamics
SV = f_dyn(SP,SV,Gravity);
% ------------------------------

% verify
SV.ddq - ddq
SV.L(1).dv - dv
SV.L(1).dw - dw

% -----------------------------------------------------
% Example for computing H using inverse dynamics
% (the procedure can be made to be much more efficient)
% -----------------------------------------------------

% set forces/torques equal to zero
SV.L(1).F = zeros(3,1);
SV.L(1).T = zeros(3,1);
SV.tau = zeros(3,1);
Gravity = [0;0;0];

% set velocities equal to zero
SV.L(1).v = zeros(3,1);
SV.L(1).w = zeros(3,1);
SV.dq = zeros(SP.n,1);

H_bar1 = zeros(SP.n+6,SP.n+6);

dv = zeros(3,1);
dw = zeros(3,1);
ddq = zeros(SP.n,1);

for i = 1:3  
  dv = zeros(3,1);
  dv(i) = 1;
  
  F_id = r_ne1_a(SP,SV,R,Gravity,ddq,dv,dw);
  H_bar1(:,i) = F_id;
end

dv = zeros(3,1);

for i = 1:3  
  dw = zeros(3,1);
  dw(i) = 1;
  
  F_id = r_ne1_a(SP,SV,R,Gravity,ddq,dv,dw);
  H_bar1(:,i+3) = F_id;
end

dw = zeros(3,1);

for i = 1:SP.n  
  ddq = zeros(SP.n,1);
  ddq(i) = 1;
  
  F_id = r_ne1_a(SP,SV,R,Gravity,ddq,dv,dw);
  H_bar1(:,i+6) = F_id;
end

H_bar - H_bar1

%%%EOF
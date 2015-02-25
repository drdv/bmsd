% uses function normalize_angles.m

clear;clc
% system definition
SP = model_ABB();
SV = System_Variables(SP);

% ----------------------------------------------------------------------
% joint limits for
% ----------------------------------------------------------------------
qL = [ -180.0  -90.0  -230.0  -200.0  -110.0  -400.0 ]'*pi/180;
qU = [  180.0  120.0    50.0   200.0   110.0   400.0 ]'*pi/180;
% ----------------------------------------------------------------------

% ----------------------------------------------------------------------
% restrictive joint limits
% ----------------------------------------------------------------------
qLr = [ -90.0  -90.0  -50.0  -100.0  -110.0  -200.0 ]'*pi/180;
qUr = [  90.0   90.0   50.0   100.0   110.0   200.0 ]'*pi/180;
% ----------------------------------------------------------------------

SV = calc_pos(SP,SV);
[pE,RE] = fk_e(SP,SV,SP.bN,SP.bP,SP.bR);

Draw_System(SP,SV,SP.bN,SP.bP,1:SP.n);
grid on;axis equal

% generate some desired position/orientation for the end-effector
dP = pE + randn(3,1)/10;
dR = rx(randn)*ry(randn)*rz(randn)*RE;

% Compute joint angles corresponding to dP and dR (within the joint limits)
q = ones(6,1)*1000; % set q to something "crazy"
count = 0;
count_limit = 100;
while any(q<qLr) || any(q>qUr)
    % each time solve the IK from different starting configuration
    % until we find q within the joint limits
    SV.q = randn(SP.n,1)/2;
    [q, iter] = ik_e(SP,SV,SP.bN,SP.bP,SP.bR,dP,dR);
    if iter < 100 % if ik_e.m converged
        q = normalize_angles(q,qL,qU);
    else
        q = ones(6,1)*1000;
    end
    
    if count == count_limit
        break
    end
    count = count+1;
end

if count == count_limit
    fprintf('Warning: q in [qLr, qUr] not found in %i iterations. \n',count_limit);
end

SV.q = q;
Draw_System(SP,SV,SP.bN,SP.bP,1:SP.n);
grid on;axis equal

SV = calc_pos(SP,SV);
[pE1,RE1] = fk_e(SP,SV,SP.bN,SP.bP,SP.bR);

disp('position error:')
dP-pE1

disp('orientation error:')
dR-RE1

%%%EOF
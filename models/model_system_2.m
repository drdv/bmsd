function SP = model_system_2()
%
% two links (planar system)
% 

% definition of system structure
% ----------------------------------------
SP.C = [0 1 2];
SP.n = length(SP.C)-1;
SP.mode = 1;

% definition of joints
% ----------------------------------------
SP.J(1).t    = [  0.5     0.0      0.0 ]';
SP.J(1).f    = [  0.5     0.0      0.0 ]';
SP.J(1).rpy  = [  0.0     0.0      0.0 ]';
SP.J(1).type = 'R';

SP.J(2).t    = [  0.5     0.0      0.0 ]';
SP.J(2).f    = [  0.5     0.0      0.0 ]';
SP.J(2).rpy  = [  0.0     0.0      0.0 ]';
SP.J(2).type = 'R';

% definition of links
% ----------------------------------------
SP.L(1).m = 1;
SP.L(1).I = eye(3);

SP.L(2).m = 1;
SP.L(2).I = eye(3);

SP.L(3).m = 1;
SP.L(3).I = eye(3);

% definition of end-effectors
% ----------------------------------------
SP.bN = 3;
SP.bP = [0.5 0.0 0.0]';

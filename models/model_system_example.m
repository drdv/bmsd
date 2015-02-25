function SP = model_system_example()
%
% system of five links (including base)
% 

% definition of system structure
% ----------------------------------------
SP.C = [0 1 2 1 1];
SP.n = length(SP.C)-1;
SP.mode = 0; % free-floating base

% definition of joints
% ----------------------------------------
SP.J(1).t    = [0.5 0.0 0.0]';
SP.J(1).f    = [0.5 0.0 0.0]';
SP.J(1).rpy  = [  pi/2     0.0      0.0 ]';
SP.J(1).type = 'R';

SP.J(2).t    = [0.5 0.0 0.0]';
SP.J(2).f    = [0.5 0.0 0.0]';
SP.J(2).rpy  = [  -pi/2     0.0      0.0 ]';
SP.J(2).type = 'R';

SP.J(3).t    = [0.0 -0.5 0.0]';
SP.J(3).f    = [0.0 -0.5 0.0]';
SP.J(3).rpy  = [  0.0   -pi/2   0.0 ]';
SP.J(3).type = 'P';

SP.J(4).t    = [0.0 0.5 0.0]';
SP.J(4).f    = [0.0 0.5 0.0]';
SP.J(4).rpy  = [  0.0     0.0      0.0 ]';
SP.J(4).type = 'R';

% definition of links
% ----------------------------------------
SP.L(1).m = 1;
SP.L(1).I = eye(3);

SP.L(2).m = 1;
SP.L(2).I = eye(3);

SP.L(3).m = 1;
SP.L(3).I = eye(3);

SP.L(4).m = 1;
SP.L(4).I = eye(3);

SP.L(5).m = 1;
SP.L(5).I = eye(3);

% definition of end-effectors
% ----------------------------------------
SP.bN = [3    4];
SP.bP = [0.5  0.4;
         0.0  0.0;
         0.0 -0.3];

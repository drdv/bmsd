function SP = model_system_3()
%
% three links (3D system)
% 

% definition of system structure
% ----------------------------------------
SP.C = [0 1 2 3];
SP.n = length(SP.C)-1;
SP.mode = 1;

% definition of joints
% ----------------------------------------
SP.J(1).t    = [  0.0     0.0      0.0 ]';
SP.J(1).f    = [  0.5     0.0      0.0 ]';
SP.J(1).rpy  = [  0.0     0.0      0.0 ]';
SP.J(1).type = 'R';

SP.J(2).t    = [  0.5     0.0      0.0 ]';
SP.J(2).f    = [  0.5     0.0      0.0 ]';
SP.J(2).rpy  = [  pi/2     0.0      0.0 ]';
SP.J(2).type = 'R';

SP.J(3).t    = [  0.5     0.0      0.0 ]';
SP.J(3).f    = [  0.5     0.0      0.0 ]';
SP.J(3).rpy  = [  -pi/2     0.0      0.0 ]';
SP.J(3).type = 'R';

% definition of links
% ----------------------------------------
SP.L(1).m = 1;
I = rand(3,3);
SP.L(1).I = I'*I;

SP.L(2).m = 2;
I = rand(3,3);
SP.L(2).I = I'*I;

SP.L(3).m = 3;
I = rand(3,3);
SP.L(3).I = I'*I;

SP.L(4).m = 4;
I = rand(3,3);
SP.L(4).I = I'*I;

% definition of end-effectors
% ----------------------------------------
SP.bN = 4;
SP.bP = [0.5 0.0 0.0]';

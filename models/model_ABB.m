function SP = model_ABB()
% 
% model of 
% ABB IRB140 manipulator (6 DOF RRRRRR)
%
% the mass and inertia parameters are arbitrary set
% the center of mass is assumed to co-inside with the input joint
%

% definition of system structure
% ----------------------------------------
SP.C = 0:6;
SP.n = length(SP.C)-1;
SP.mode = 1;

% definition of joints
% ----------------------------------------
SP.J(1).t    = [  0.0     0.0      0.0 ]';
SP.J(1).f    = [  0.0     0.0      0.0 ]';
SP.J(1).rpy  = [  0.0     0.0      0.0 ]';
SP.J(1).type = 'R';

SP.J(2).t    = [  0.07    0.0    0.352 ]';
SP.J(2).f    = [  0.0     0.0      0.0 ]';
SP.J(2).rpy  = [-pi/2     0.0      0.0 ]';
SP.J(2).type = 'R';

SP.J(3).t    = [  0.0    -0.36     0.0 ]';
SP.J(3).f    = [  0.0     0.0      0.0 ]';
SP.J(3).rpy  = [  0.0     0.0      0.0 ]';
SP.J(3).type = 'R';

SP.J(4).t    = [  0.24    0.0      0.0 ]';
SP.J(4).f    = [  0.0     0.0      0.0 ]';
SP.J(4).rpy  = [  0.0    pi/2      0.0 ]';
SP.J(4).type = 'R';

SP.J(5).t    = [  0.0     0.0     0.14 ]';
SP.J(5).f    = [  0.0     0.0      0.0 ]';
SP.J(5).rpy  = [  0.0   -pi/2      0.0 ]';
SP.J(5).type = 'R';

SP.J(6).t    = [  0.0     0.0      0.0 ]';
SP.J(6).f    = [  0.0     0.0      0.0 ]';
SP.J(6).rpy  = [  0.0    pi/2      0.0 ]';
SP.J(6).type = 'R';

% definition of end-effectors
% ----------------------------------------
SP.bN = 7;
SP.bP = [0;0;0.065];

% orientation offset for the end-effector
SP.bR = rz(pi/2);

%%%EOF
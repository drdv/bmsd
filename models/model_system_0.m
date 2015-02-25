function SP = model_system_0
%
% only a base
%

% definition of system structure
% ----------------------------------------
SP.C = [0];
SP.n = length(SP.C)-1;
SP.mode = 0; % free-floating base

% definition of links
% ----------------------------------------
SP.L(1).m = 1;
SP.L(1).I = diag([1,2,3]);


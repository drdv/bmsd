function SP = model_test()
%
% this model is for testing purposes
% 

% definition of system structure
% ----------------------------------------
SP.C = [0 1 2 3 3 2 5 7 1 1 10 11 7 13 12 12];
SP.n = length(SP.C)-1;
SP.mode = 0;

% definition of joints
% ----------------------------------------
for iJ = 1:SP.n
  SP.J(iJ).t    = rand(3,1);
  SP.J(iJ).f    = rand(3,1);
  SP.J(iJ).rpy  = rand(3,1);
  SP.J(iJ).type = 'R';
end

% definition of links
% ----------------------------------------
for iL = 1:SP.n+1
  SP.L(iL).m = rand;
  I = randn(3,3);
  SP.L(iL).I = I*I';
end

% definition of end-effectors
% ----------------------------------------
SP.bN = 13;
SP.bP = rand(3,1);

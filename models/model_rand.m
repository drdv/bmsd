function SP = model_rand(NL)
%
% random model with NL links (including base)
% 

if nargin < 1
  NL = 3;
end

% definition of system structure
% ----------------------------------------
SP.C = [0:NL];
SP.n = length(SP.C)-1;
SP.mode = 0; % free-floating base

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
  SP.L(iL).m = rand(1,1);
  I = rand(3,3);
  SP.L(iL).I = I*I';
end


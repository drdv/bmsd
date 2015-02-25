function joints = joint_tree(SP,bN)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | General purpose |
% -------------------
%
% joint_tree
%
% Returns joints between link 1 and link "bN"
%
% Syntax:
% -------
% joints = joint_tree(SP,bN);
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% bN  [1x1]  - body number
%
% Output:
% -------
% joints  [?x1]  -: joints between link 1 and link "bN" 
%                   (the dimension depend on the particular tree structure)
%
% SV members changed:
% --------------------
%

if bN <= 1 || bN > SP.n+1 
  if bN == 1 % there are no joints "supporting" the base
    joints = [];
    return
  end
  
  disp('ERROR: bN is out of range');
  return
end

iL = bN;
iJ = iL-1;
iS = SP.C(iL);

joints = iJ; % the "input" joint for body bN 

i = 1;
while iS ~= 1 % while iS is not "the base"
  iL = iS;
  iJ = iL-1;
  iS = SP.C(iL);

  i = i+1;
  joints(i) = iJ;
end
joints = joints(i:-1:1); % this is possible because SP.C(iL)<iL

%%%EOF
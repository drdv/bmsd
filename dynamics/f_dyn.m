function SV = f_dyn(SP,SV,Gravity)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Dynamics |
% ------------
%
% f_dyn
%
% forward dynamics
% 
% Syntax:
% -------
% SV = f_dyn(SP,SV,Gravity);
%
% Input:
% ------
% SP         - structure containing system parameters (see struct_def.m)
% SV         - structure containing system variables (see struct_def.m) 
% Gravity    - [3x1] gravity vector
%
% Output:
% -------
% SV         - structure containing system variables (see struct_def.m) 
%
% SV members changed:
% --------------------
% SV.L(1).dv, SV.L(1).dw, SV.ddq
% updates the positions SV.L(2:SP.n+1).R, SV.L(2:SP.n+1).p
%
  
SV = calc_pos(SP,SV);

%HH = calc_hh(SP,SV);
%c = r_ne(SP,SV,Gravity);

[HH,R] = calc_hh_a(SP,SV);
c = r_ne_a(SP,SV,R,Gravity);

Force = [zeros(6,1); SV.tau] - c;

if SP.mode ~= 0 % fixed-base manipulator

  SV.ddq = HH(7:end,7:end)\Force(7:end);

else % free-floating system

  Acc = HH\Force;
  
  SV.L(1).dv = Acc(1:3);
  SV.L(1).dw = Acc(4:6);
  SV.ddq = Acc(7:6+SP.n);
  
end

%%%EOF
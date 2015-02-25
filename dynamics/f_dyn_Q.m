function [SV,dy,y] = f_dyn_Q(SP,SV,Gravity,y)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Dynamics |
% ------------
%
% f_dyn
%
% forward dynamics (used by int_rk4.m)
% (updating the Quaternion of the base)
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

if nargin < 4
  y = [];
end

% update the state
if ~isempty(y)
  SV.L(1).p = y(1:3);
  SV.L(1).Q = y(4:7); SV.L(1).R = q2R(SV.L(1).Q);
  SV.L(1).v = y(8:10);
  SV.L(1).w = y(11:13);
  SV.q = y(14:13+SP.n);
  SV.dq = y(14+SP.n:13+2*SP.n);
else % for convenience I output y if the input y=[]
  y = [SV.L(1).p;
       SV.L(1).Q(:);
       SV.L(1).v;
       SV.L(1).w;
       SV.q;
       SV.dq];
end

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

dy = [SV.L(1).v;
      qw2dq(SV.L(1).Q,SV.L(1).w); % dQ
      SV.L(1).dv;
      SV.L(1).dw;
      SV.dq;
      SV.ddq];

%%%EOF
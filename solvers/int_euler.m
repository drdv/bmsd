function  SV = int_euler(SV,d_time,case_flag,case_flag1)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Solvers |
% -------------
%
% Euler integration (implicit or explicit)
%
% Syntax:
% -------
% SV = int_euler(SV,d_time)
% SV = int_euler(SV,d_time,1)
% SV = int_euler(SV,d_time,1,1)
%
% Input:
% ------
% SV          - structure containing system variables (see struct_def.m)
% d_time      - fixed integration step
%
% case_flag   - specify case_flag=0 for implicit Euler
%               specify case_flag=1 for explicit Euler
%
% case_flag1  - specify case_flag1=0 in order to use Rodrigues rotation formula
%               specify case_flag1=1 in order to use Quaternion
%
% by default it is assumed that case_flag = 0 and case_flag1 = 0 
%
% Output:
% -------
% SV
%
% SV members changed:
% --------------------
% L(1).p, L(1).R, L(1).Q, L(1).v, L(1).w, q, dq
%

tol = 1e-12;

if nargin < 4
  case_flag1 = 0; % use Rodrigues rotation formula by default 
end

if nargin < 3
  case_flag1 = 0; % use Rodrigues rotation formula by default 
  case_flag = 0; % use implicit Euler integration by default 
end

v_n = SV.L(1).v + SV.L(1).dv*d_time;
w_n = SV.L(1).w + SV.L(1).dw*d_time;
  
if case_flag == 1 %explicit

  SV.L(1).p = SV.L(1).p + SV.L(1).v*d_time;
  
  if case_flag1 % using quaternion
  
    dQ = qw2dq(SV.L(1).Q,SV.L(1).w)';
    SV.L(1).Q = SV.L(1).Q + dQ*d_time; 
    SV.L(1).Q = SV.L(1).Q/norm(SV.L(1).Q);
    SV.L(1).R = q2R(SV.L(1).Q);
  
  else % using Rodrigues rotation formula
  
    norm_w = norm(SV.L(1).w);
    if norm_w > tol
      SV.L(1).R = aa2R( norm_w*d_time , SV.L(1).w/norm_w )*SV.L(1).R;   
    end
  
  end
  
else % implicit
  
  SV.L(1).p = SV.L(1).p + v_n*d_time;
  
  if case_flag1 % using quaternion
  
    dQ = qw2dq(SV.L(1).Q,w_n)';
    SV.L(1).Q = SV.L(1).Q + dQ*d_time;
    SV.L(1).Q = SV.L(1).Q/norm(SV.L(1).Q);    
    SV.L(1).R = q2R(SV.L(1).Q);
  
  else % using Rodrigues rotation formula
   
    norm_w = norm(w_n);
    if norm_w > tol
      SV.L(1).R = aa2R( norm_w*d_time , w_n/norm_w )*SV.L(1).R;
    end
  
  end
  
end

SV.L(1).v = v_n;
SV.L(1).w = w_n;

if ~isempty(SV.q)

  dq_n = SV.dq + SV.ddq*d_time;

  if case_flag == 1
    SV.q  = SV.q + SV.dq*d_time; % explicit
  else
    SV.q  = SV.q + dq_n*d_time; % implicit
  end
  
  SV.dq = dq_n;
end

%%%EOF
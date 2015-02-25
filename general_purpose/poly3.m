function [q,dq,ddq] = poly3(qi,qf,vi,vf,t)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | General purpose |
% -------------------
%
% poly3
%
% Using 3rd order polynomial generate position, velocity and acceleration profiles  
% that satisfy the initial and final position and velocity constraints. 
% 
% Syntax:
% -------
% [q,dq,ddq] = poly3(qi,qf,vi,vf,t)
%
% Input:
% ------
% qi   - initial position
% qf   - final position
% vi   - initial velocity
% vf   - final velocity
% t    - [ti : time_step : tf]
%
% Output:
% -------
% q    - position profile
% dq   - velocity profile
% ddq  - acceleration profile
%
% Example: 
% --------
% [q, dq, ddq] = poly3(5, 30, 0, 0, [0:0.01:10])

t = t(:); ti = t(1); tf = t(end); 
N = length(t);
I = ones(N,1);
Z = zeros(N,1);

b = [qi;vi;qf;vf];
A = [  ti^3    ti^2   ti  1;   
     3*ti^2  2*ti     1   0;   
       tf^3    tf^2   tf  1;   
     3*tf^2  2*tf     1   0];  
p = A\b;                     % p = [a b c d]'

tt = [t.^3 t.^2 t I]; 
q = tt*p;                    % position

tt = [3*t.^2 2*t I Z];
dq = tt*p;                   % velocity

tt = [6*t 2*I Z Z];
ddq = tt*p;                  % acceleration

%%%EOF
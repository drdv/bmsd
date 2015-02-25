%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Examples |
% ------------
%
% test_slerp
%
% Test the slerp function
%

r_i = [0 0 0];     % initial frame orientation
r_e = rand(3,1);   % final frame orientation

% form quaternion from rpy (X-Y-Z rotation sequence around the new axis)
qi = rpy2q(r_i);
qe = rpy2q(r_e);

t_step = 0.05;
T = 0:t_step:1;
N = length(T);
q = poly3(0, 1, 0, 0, T);

i = 1;
for t = 0:t_step:1
  t = q(i);
  qm(i,:) = slerp(qi,qe,t,eps);
  i = i+1;
end

cc = {'r','b'};
hold on;
k = 1;
for j = 1:1:i-1
  draw_frame(qm(j,:), cc{k});
  drawnow
  if k == 2
    k = 1;
  else
    k = k+1;
  end
end

%%%EOF
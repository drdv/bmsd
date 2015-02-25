function [P1,dP1] = trajC(x,y,z,t_f,d_time,disp_flag)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | General purpose |
% -------------------
%
% trajC
%
% Generation of Cartesian trajectory 
%
% Syntax:
% -------
% [P1,dP1] = trajC(x,y,z,t_f,d_time)
%
% Input:
% ------
% x,y,z  - x,y and z coordinates of points in Cartesian space 
% t_f    - final time
% d_time - time step
% disp_flag  - if disp_flag=1 the result points are plotted
%
% Output:
% -------
% P1,    - position profile
% dP1    - velocity profile
% 
% For a example see below

% Example
if nargin == 0
  d_time = 0.1;
  t_f = 5;
  
  x = [ 1.3744   1.45   1.6      1.75    1.85    2       2.4];
  y = [-2.0193  -1.6   -1.5193  -1.43   -1.35   -1.20   -1.1];
  z = [ 2.2147   2      1.8      1.68    1.6     1.5     1.4];

  disp_flag = 1;
end

if nargin == 5
  disp_flag = 0;
end

k=0.5;

x = x(:)';
y = y(:)';
z = z(:)';

P = [x;y;z];

dx=diff(x)';
dy=diff(y)';
dz=diff(z)';

dt=(dx.^2+dy.^2+dz.^2).^(k/2);
t=cumsum([0;dt]);

tt = [0:0.01:t(end)];

pp.n = length(t)-1;
pp.t = t;
pp.t0 = pp.t(2:end)-pp.t(1:end-1);
pp.x = cspline(t,x,tt);
pp.y = cspline(t,y,tt);
pp.z = cspline(t,z,tt);

f = @(t,x,y,z) sqrt((3*x(1)*t.^2 + 2*x(2)*t + x(3)).^2 + ...
                    (3*y(1)*t.^2 + 2*y(2)*t + y(3)).^2 + ...
                    (3*z(1)*t.^2 + 2*z(2)*t + z(3)).^2 );

for i=1:pp.n 
  le(i) = quad(f,0,pp.t0(i),[],[],pp.x(i,:), pp.y(i,:), pp.z(i,:));
end

[q, dq] = poly3(0, sum(le), 0, 0, [0:d_time:t_f]);

ta(1) = 0;
for i=2:length(q)
  ta(i) = secant(@(x_up) curve_len(x_up, ta(i-1), q(i), q(i-1), pp, le),ta(i-1),ta(i-1)+q(i)-q(i-1));
end

for i = 1:1:length(ta)

  k = max(find(pp.t(1:end-1)<=ta(i)));  
  
  z = ta(i) - pp.t(k);
  P1(1,i) = pp.x(k,:)*[z^3 ; z^2 ; z ; 1];
  P1(2,i) = pp.y(k,:)*[z^3 ; z^2 ; z ; 1];
  P1(3,i) = pp.z(k,:)*[z^3 ; z^2 ; z ; 1];
  
end

dP1 = [zeros(3,1), num_diff(P1,d_time)];

if disp_flag
  hold on;
  plot3(P(1,:),P(2,:),P(3,:),'rs','MarkerSize',10)
  plot3(P1(1,:),P1(2,:),P1(3,:),'b*');
  grid on
  cameratoolbar
  
  % for i = 1:1:length(dP1)
  %   ndP1(i) =  norm(dP1(:,i));  
  % end
  %
  % figure(2);hold on;
  % plot(ndP1)
  % plot(dq,'r')
end

%-------------------
% Utility functions
%-------------------
 
function S = cspline(x,y,xi)

N = length(x);

E = zeros(N,N); e = zeros(N,1);
a = zeros(N-1,1); c = zeros(N-1,1); d = zeros(N-1,1);
b = zeros(N,1); 

k = 1:N-1;

h(k) = x(k+1)-x(k); 
dy(k) = (y(k+1)-y(k))./h(k);

% Boundary condition (2nd derivatives)
E(1,1) = 2; e(1) = 0; 
E(N,N) = 2; e(N) = 0;

for k = 2:N-1
  E(k,k-1:k+1) = [h(k-1) 2*(h(k-1)+h(k)) h(k)];
  e(k) = 3*(dy(k)-dy(k-1));
end
b = inv(E)*e;

for k = 1:N-1
  a(k) = (b(k+1)-b(k))/(3*h(k)); 
  c(k) = dy(k) - h(k)*(b(k+1)+2*b(k))/3;
  d(k) = y(k);
end

S = [a,b(1:N-1),c,d]; 


function obj_fun = curve_len(x_up, x_low, f_up, f_low, pp, le)
%
% Function for calculating the length of a curve
%

if x_up == x_low
  
  obj_fun = -(f_up-f_low);
  
else
  
  k_low = max(find(pp.t(1:end-1)<=x_low));  
  k_up = max(find(pp.t(1:end-1)<=x_up));

  f = @(t,x,y,z) sqrt((3*x(1)*t.^2 + 2*x(2)*t + x(3)).^2 + ...
                      (3*y(1)*t.^2 + 2*y(2)*t + y(3)).^2 + ...
                      (3*z(1)*t.^2 + 2*z(2)*t + z(3)).^2 );
  
  x = pp.x(k_low,:);
  y = pp.y(k_low,:);
  z = pp.z(k_low,:);

  if k_low == k_up
    
    L = quad(f,x_low-pp.t(k_low),x_up-pp.t(k_low),[],[],x,y,z);
    obj_fun = L - (f_up-f_low);
    
  else

    L  = quad(f,x_low - pp.t(k_low),pp.t(k_low+1) - pp.t(k_low),[],[],x,y,z);
    
    if ~isempty(k_low+1:k_up-1)
      L = L + sum(le(k_low+1:k_up-1));
    end
    
    x = pp.x(k_up,:);
    y = pp.y(k_up,:);
    z = pp.z(k_up,:);   
    L = L + quad(f,0,x_up-pp.t(k_up),[],[],x,y,z);
    
    obj_fun = L - (f_up-f_low);
    
  end
  
end


function dS = num_diff(S,d_time)
%
% numerical differentiaton
%

dS=zeros(3,length(S)-1);

j=1;
for i = 2:1:length(S)
  dS(:,j) = (S(:,i)-S(:,i-1))/d_time;
  j=j+1;
end

function [b,iter]=secant(f,a,b)
%
% see http://math.fullerton.edu/mathews/n2003/SecantMethodMod.html
%

max_iter = 20;
tol = 1e-14;
%tol = 1e-6;

fa = feval(f,a);
fb = feval(f,b);

for iter=1:max_iter

  c=b-fb*(b-a)/(fb-fa);
  
  err=abs(c-b);
  relerr=2*err/(abs(c)+tol);
  
  a=b; b=c; fa=fb;
  
  fb = feval(f,b);
  
  if (abs(fb)<tol), 
    return 
  end  
end

disp('MAX itarations reached')

%%%EOF
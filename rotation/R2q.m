function q = R2q(R)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% R2q
%
% Converts a rotation matrix to unit quaternion
%
% Syntax:
% -------
% q = R2q(R)
%
% Input:
% ------
% R  - 3x3 rotation matrix 
%
% Output:
% -------
% q = [s x y z] - unit quaternion 
%     [s]       - scalar part
%     [x y z]   - vector part
%

% Algorithm:
% -----------
% The rotation matrix that corresponrs to a unit quaternion is (see q2R.m) 
% 
% R = [1-2*z^2-2*y^2,   2*x*y-2*s*z,   2*x*z+2*s*y;
%        2*x*y+2*s*z, 1-2*z^2-2*x^2,   2*y*z-2*s*x;
%        2*x*z-2*s*y,   2*y*z+2*s*x, 1-2*y^2-2*x^2];
%
% Define vectors X, b, r and a matrix A in the following way:  
% 
% X = [x*x ; y*y ; z*z ; x*y ; x*z ; x*s ; y*z ; y*s ; z*s];
% b = [1 ; 0 ; 0 ; 0 ; 1 ; 0 ; 0 ; 0 ; 1];
% r = [R(1,1) ; R(1,2) ; R(1,3) ; R(2,1) ; R(2,2) ; R(2,3) ; R(3,1) ; R(3,2) ; R(3,3)];
%
% A = [ 0 -2 -2  0  0  0  0  0  0;      
%       0  0  0  2  0  0  0  0 -2;      
%       0  0  0  0  2  0  0  2  0;      
%       0  0  0  2  0  0  0  0  2;      
%      -2  0 -2  0  0  0  0  0  0;      
%       0  0  0  0  0 -2  2  0  0;      
%       0  0  0  0  2  0  0 -2  0;      
%       0  0  0  0  0  2  2  0  0;      
%      -2 -2  0  0  0  0  0  0  0];     
%
% It is straightforward to verify that 
% A*X + b = r
%
% Solving the above linear system of equation for X gives
% X = inv(A)*(r-b)  % note that matrix A is of full rank
%
% the result is
% X(1) = x*x =  (1 + R(1,1) - R(2,2) - R(3,3))/4;
% X(2) = y*y =  (1 - R(1,1) + R(2,2) - R(3,3))/4;
% X(3) = z*z =  (1 - R(1,1) - R(2,2) + R(3,3))/4;
% X(4) = x*y =  (R(1,2) + R(2,1))/4;
% X(5) = x*z =  (R(1,3) + R(3,1))/4;
% X(6) = x*s =  (R(3,2) - R(2,3))/4;
% X(7) = y*z =  (R(2,3) + R(3,2))/4;
% X(8) = y*s =  (R(1,3) - R(3,1))/4;
% X(9) = z*s =  (R(2,1) - R(1,2))/4;
% 
% Thee are many ways to solve the nine equations above in order to obtain the four parameters 
% x y z and s. Of course we need only frour equations, but it is not clear which four. Depending 
% on R we should chose four equations in a way that guarantees a numerically stable solution.
%
% Choosing four equations:
%
% 1. Note that X(1), X(2), X(3) can be only >=0 (X(1) = x^2 ...)
%
% 2. Chose the largest element among X(1), X(2) and X(3). This guarantees that we are not working
% with 0. If indeed the largest element is 0 then X(1) = X(2) = X(3) = 0, so the quaternion is
% q = [1 0 0 0].
%
% 3. If X(1) is the largest element we have
%       x = sqrt(X(1)) and from X(4:6) we can determine y, z and s
%    If X(2) is the largest element  we have 
%       y = sqrt(X(2)) and from X(4,7,8) we can determine x, z and s
%    If X(3) is the largest element  we have
%       z = sqrt(X(3)) and from X(5,7,9) we can determine x, y and s
%
% 4. With this algorithm depending on the choice X(1) or X(2) or X(3) we can
%    obtain a quaternion with positive or negative "s". For convenience we can fix 
%    it to be always positive by doing:
%  
%    if s < 0 sgn = -1; else sgn = 1; end
%    q = sgn * [s x y z];

X = zeros(9,1); % solution of the linear system 
X(1) =  (1 + R(1,1) - R(2,2) - R(3,3))/4; % xx
X(2) =  (1 - R(1,1) + R(2,2) - R(3,3))/4; % yy
X(3) =  (1 - R(1,1) - R(2,2) + R(3,3))/4; % zz
X(4) =  (R(1,2) + R(2,1))/4;              % xy
X(5) =  (R(1,3) + R(3,1))/4;              % xz
X(6) =  (R(3,2) - R(2,3))/4;              % xs
X(7) =  (R(2,3) + R(3,2))/4;              % yz
X(8) =  (R(1,3) - R(3,1))/4;              % ys
X(9) =  (R(2,1) - R(1,2))/4;              % zs

[mx, ix] = max(X(1:3));

if ix == 1 && mx > eps
  x = sqrt(X(1));
  y = X(4)/x;
  z = X(5)/x;
  s = X(6)/x;
elseif ix == 2
  y = sqrt(X(2));
  x = X(4)/y;
  z = X(7)/y;
  s = X(8)/y;
elseif ix == 3
  z = sqrt(X(3));
  x = X(5)/z;
  y = X(7)/z;
  s = X(9)/z;
else
  s = 1;
  x = 0;
  y = 0;
  z = 0;
end

% make sure that "s" is positive
% convenient when we interpolate (no jumps from q to -q)
if s < 0 sgn = -1; else sgn = 1; end

q = sgn * [s x y z];

%%%EOF
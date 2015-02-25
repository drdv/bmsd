function dq = qw2dq(q,w)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% qw2dq
%
% Time derivatine of a quaternion when we know the angular velocity w
%
% Note: Knowing dq is usefull when we want to determine the attitude of a 
% rigid body at time t+1 when we know q and w at time t. We can simply 
% integrate dq to obtain q.
% Other method would be to use Rodrigues rotation formula see (aa2R.m)
% For more information see Peter C. Hughes "Spacecraft Attitude Dynamics"
%
% Syntax:
% -------
% dq = qw2dq(q,w)
% 
% Input: 
% -------
% w - (3x1) body angular velocity (expressed in inertial frame) 
% q = [s x y z] - unit quaternion 
%     [s]       - scalar part
%     [x y z]   - vector part
%
% Output:
% --------
% dq  - the time derivative of q  
%

% Algorithm:
% ----------
% syms w1 w2 w3 s x y z 
% q = [s x y z];
% W = [0 w1 w2 w3]; angular velocity expressed as a quaternion
%
% dq = 0.5 * quat_mult(W,q); % dq is the time derivative of q
%
% if I reverse the product, I have to use w in the body frame 
% dQ = 0.5 * quat_mult(SV.L(1).Q,[0;SV.L(1).R'*SV.L(1).w]); 
%
% The derivation of dq = 0.5 * W * q follows
%
% X0 - initial vector expressed as a quaternion X0 = [0 X0x X0y X0z]
% X1 - rotated vector expressed as a quaternion X1 = [0 X1x X1y X1z]
% 
% X1 = q*X0*q1 ... (1) 
%
% q =[s x y z]; 
% q1 = [s -x -y -z];
%
% differentiating (1) with respect to time we get
% 
% dX1/dt = dq/dt*X0*q1 + q*X0*dq1/dt ...(2)
% 
% from (1) X0 = q1*X1*q substituting in (2) we get
%
% dX1/dt = dq/dt*q1*X1 + X1*q*dq1/dt ...(3)
%
% because q is a unit quaternion (s^2+x^2+y^2+z^2 = 1 ) we heave
%
% quat_mult(q,q1) = [ s^2+x^2+y^2+z^2, 0, 0, 0] = [1 0 0 0]
%
% differentiating the above equation we get
%
% dq/dt*q1 + q*dq1/dt = [0,0,0,0]
% 
% hence,
%
% -dq/dt*q1 = q*dq1/dt ... (4)
%
% combining (3) amd (4) we obtain:
%
% dX1/dt = dq/dt*q1*X1 - X1*dq/dt*q1 ...(5)
%
% let us denote G = dq/dt*q1 hence
% 
% dX1/dt = G*X1 - X1*G ...(6)
% 
% expanding the quaternion multiplications above we obtain 
%
% G*X1 = (Xs*Gs - Xv'*Gv) + (Xs*Gv + Gs*Xv + cross(Gv,Xv))
% X1*G = (Xs*Gs - Xv'*Gv) + (Xs*Gv + Gs*Xv + cross(Xv,Gv))
%
%               [scalar               vector]            [scalar         vector]
% G*X1 - X1*G =  (0)   + (cross(Gv,Xv) - cross(Xv,Gv)) =   (0)   +  (2*cross(Gv,Xv))
%
% where
% X1 = [Xs Xx Xy Xz] = [Xs Xv]
% G  = [Gs Gx Gy Gz] = [Gs Gv]
%
% Note that the scalar part of G*X1-X1*G is (0).
%
% dX1/dt = (0) + (2*cross(Gv,Xv))
%
% but we can express dX1/dt as
%
% dX1/dt = (0) + (cross(V(W),Xv))  % V(.) is the vector part of .
%
% where W is the angular velocity of the rigid body expressed in a quaternion form [0 w1 w2 w3], hence:
%
% cross(V(W),Xv) = 2*cross(Gv,Xv)
% 
% hence, 
% V(W) = 2*Gv =>
% W = 2*dq/dt*q1  
%
% rearanging leads to 
%
% dq/dt = 0.5*W*q

s = q(1);
x = q(2);
y = q(3);
z = q(4);

E = [-x -y -z;  
      s  z -y; 
     -z  s  x;  
      y -x  s];

dq = 0.5*E*w(:);

% Equivalent expressions
% -----------------------------------------------
%dq = 0.5*[-w(1)*x - w(2)*y - w(3)*z;  
%           w(1)*s + w(2)*z - w(3)*y; 
%          -w(1)*z + w(2)*s + w(3)*x;  
%           w(1)*y - w(2)*x + w(3)*s];

%dq = 0.5 * quat_mult([0;w(1);w(2);w(3)],q);
%dq = 0.5 * [0 w(1) w(2) w(3)]*qRM(q);
%dq = 0.5 * q'*qLM([0 w(1) w(2) w(3)])';

%wa = w/norm(w);
%dq = 0.5 * q'*qLM([0 wa(1) wa(2) wa(3)])*norm(w);
% -----------------------------------------------

%%%EOF
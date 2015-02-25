function w = qq2w(q1,q2,d_time)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Rotations |
% -------------
%
% qq2w
%
% Compute angular velocity w given two quaternions q1,q2 and 
% time d_time for the transition from q1 to q2 
%
% Syntax:
% -------
% w = qq2w(q1,q2,d_time)
% 
% Input: 
% -------
% q1,q2 = [s x y z] - unit quaternions 
%         [s]       - scalar part
%         [x y z]   - vector part
%
% Output:
% --------
% w - (3x1) body angular velocity (expressed in the world frame) 
%

% Algorithm:
% ----------
% 1. "transitional" quaternion q -> quat_mult(quat_inv(q1),q2);
% 2. determine the axis of rotation and angle of rotation
% 3. form w as (an/d_time)*wa
%

% determine the axis and angle for the "transition" quaternion q
q = quat_mult(q2,quat_inv(q1));
[an,wa] = q2aa(q);
w = (an/d_time)*wa;

% to verify the above transition quaternion
% --------------------------------------------
% R1 = q2R(q1);
% R2 = q2R(q2);
% R = R2*R1';
% q0 = R2q(R);
% norm(q0-q)
% --------------------------------------------      

% Note that 2*acos(q1'*q2) = an
% where acos(q1'*q2) gives the angle between the two quaternions

% an alternative way to obtain ||w|| is
% ---------------------------------------------------------------
% find derivative of q
if 0
  dq = (q2-q1)/d_time;
  dq = dq(:)';
  
  %dq = 0.5*q'*qLM([0 wa(1) wa(2) wa(3)])*norm(w)
  % using the above reltion, find the norm of the angular velocity
  wn = norm(2*dq*qLM([0 wa(1) wa(2) wa(3)])');
  
  w = wn*wa; 
end
% ---------------------------------------------------------------

%%%EOF
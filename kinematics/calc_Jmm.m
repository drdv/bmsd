function Jmm = calc_Jmm(SP,SV)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% calc_Jmm
%
% Calculates an augmented Jacobian matrix of the CoM of each link 
% (except for the base) with respect to the manipulator joints.
%
% Note: This function requires updated positions SV = calc_pos(SP,SV);
%
% Syntax:
% -------
% Jmm = calc_Jmm(SP,SV);
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% SV    - structure containing system variables (see struct_def.m)
%
% Output:
% -------
% Jmm [(6*SP.n)x(SP.n)]  - Jacobian matrix.
%
% SV members changed:
% --------------------
%

Jmm = zeros(6*SP.n,SP.n); % initialization of the output
pJ = fk_j(SP,SV,1:SP.n);  % positions of joints

for j = 2:SP.n+1 % iterate over all links (except for the base)

  iL = j;
  iJ = iL-1;
  
  while iL ~= 1 
    
    k = SV.L(iL).R(:,3); % axis of rotation/translation of joint iJ (expreessed in the "world" frame)  
    
    if SP.J(iJ).type == 'R' % Revolute joint
      Jmm((j-1)*6-5:(j-1)*6-3,iJ) = cross( k , ( SV.L(j).p - pJ(:,iJ) ) );
      Jmm((j-1)*6-2:(j-1)*6,iJ) = k;
    else % Prismatic joint
      Jmm((j-1)*6-5:(j-1)*6-3,iJ) = k;
      Jmm((j-1)*6-2:(j-1)*6,iJ) = [ 0 0 0 ]';
    end
        
    iL = SP.C(iL);
    iJ = iL-1;
  end

end

%%%EOF
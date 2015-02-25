function dJmm = calc_dJmm(SP,SV)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% calc_dJmm
%
% Calculates the derivative w.r.t. time of Jmm (see calc_Jmm.m) assuming a fixed base system. 
%
% Note: This function requires updated velocities SV = calc_vel(SP,SV);
%
% Note: The explicit computation of dJe is almost never performed. 
%       The implementation is very inefficient.
%       No other function in this toolbox depends on this function. 
%
% Syntax:
% -------
% dJmm = calc_dJmm(SP,SV);
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% SV    - structure containing system variables (see struct_def.m)
%
% Output:
% -------
% dJmm [(6*SP.n)x(SP.n)]  - derivative of Jmm
%
% SV members changed:
% --------------------
%

dJmm = zeros(6*SP.n,SP.n); % initialization of the output
pJ = fk_j(SP,SV,1:SP.n);   % positions of joints in inertial frame

for j = 2:SP.n+1 % iterate over all links (except for the base)
    
    iL = j;
    iJ = iL-1;

    [Jl,ll] = calc_Je(SP,SV,j,zeros(3,1));
    vLink = Jl(1:3,ll)*SV.dq(ll); % velocity of CoM of link j due to joint velocity
    
    while iL ~= 1 
        
        k = SV.L(iL).R(:,3); % axis of rotation/translation of joint iJ (expreessed in the "world" frame)  
        
        w = SV.L(iL).w - SV.L(1).w; % subtract the angular velocity of the base (assume that the base is fixed)
        if SP.J(iJ).type == 'R' % Revolute joint
            
            % "-SP.J(iJ).f" is needed in order to go from the CoM of link iL to the joint iJ
            [Jj,jj] = calc_Je(SP,SV,iL,-SP.J(iJ).f);
            vJoint = Jj(1:3,jj)*SV.dq(jj); % velocity of joint iJ due to joint velocity

            dJmm((j-1)*6-5:(j-1)*6-3,iJ) = cross(cross(w,k),SV.L(j).p - pJ(:,iJ)) + cross(k,vLink - vJoint);
            dJmm((j-1)*6-2:(j-1)*6,iJ) = cross(w,k);
            
        else % Prismatic joint
            dJmm((j-1)*6-5:(j-1)*6-3,iJ) = cross(w,k);
            dJmm((j-1)*6-2:(j-1)*6,iJ) = [ 0 0 0 ]';
        end
        
        iL = SP.C(iL);
        iJ = iL-1;
  end
  
end

%%%EOF
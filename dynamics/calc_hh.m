function HH = calc_hh(SP,SV)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Dynamics |
% ------------
%
% calc_hh
%
% Calculation of the inertia matrix HH
% (This function is simmilar to the one provided with the Spacedyn toolbox)
% 
% Note: This function requires updated positions SV = calc_pos(SP,SV);
%
% Syntax:
% -------
% HH = calc_hh(SP,SV);
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% SV    - structure containing system variables (see struct_def.m)
%
% Output:
% -------
%
% HH [(6+SP.n)x(6+SP.n)]     - Global inertia matrix.
%
% ("6" is the DOF of the base)
%
% SV members changed:
% --------------------
%

mass = sum([SP.L.m]); % total mass of the system
wE = mass*eye(3,3);

if SP.n == 0 % Single body
    
  HH_w = SV.L(1).R*SP.L(1).I*SV.L(1).R';
  HH = [         wE    zeros(3,3);
         zeros(3,3)        HH_w ];   
    
else % Multibody system

  % memory allocation
  JJ_tg = zeros(3,SP.n);
  HH_wq = zeros(3,SP.n);
  HH_q  = zeros(SP.n,SP.n);
  
  Jmm = calc_Jmm(SP,SV);
 
  HH_w = SV.L(1).R*SP.L(1).I*SV.L(1).R';
  
  for i = 2:SP.n+1

    % indexing
    s2 = 6*(i-1)-2:6*(i-1);
    s5 = 6*(i-1)-5:6*(i-1)-3;
    
    r0i = SV.L(i).p - SV.L(1).p;
    Ii = SV.L(i).R*SP.L(i).I*SV.L(i).R';
    t_r0i = tilde(r0i);
    
    JJ_tg = JJ_tg + SP.L(i).m*Jmm(s5,:);
    HH_w  = HH_w + Ii + SP.L(i).m*(t_r0i'*t_r0i);
    HH_wq = HH_wq + Ii*Jmm(s2,:) + SP.L(i).m*t_r0i*Jmm(s5,:);
    HH_q  = HH_q  + Jmm(s2,:)'*Ii*Jmm(s2,:) + SP.L(i).m*Jmm(s5,:)'*Jmm(s5,:);
    
  end
 
  % Position of the gravity centroid, Rg
  Rm = zeros(3,1);
  for iL = 1:SP.n+1
    Rm = Rm + SP.L(iL).m * SV.L(iL).p;
  end
  
  Rg = Rm/mass;
  wr0g = (Rg-SV.L(1).p)*mass;
  t_wr0g = tilde(wr0g);
  
  HH = [ wE       t_wr0g'     JJ_tg;
         t_wr0g     HH_w      HH_wq;
         JJ_tg'    HH_wq'     HH_q ];
  
end

%%%EOF
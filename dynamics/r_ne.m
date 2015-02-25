function c = r_ne(SP,SV,Gravity)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Dynamics |
% ------------
%
% r_ne
%
% recursive Newton-Euler method (inverse dynamics)
% 
% Syntax:
% -------
% c = r_ne_a(SP,SV,Gravity);
%
% Input:
% ------
% SP         - structure containing system parameters (see struct_def.m)
% SV         - structure containing system variables (see struct_def.m) 
% Gravity    - [3x1] gravity vector
%
% Output:
% -------
% c          - nonlinear term of the equations of motion (includes external forces + gravity)
%
% SV members changed:
% --------------------
%

SV = forward_recursion(SP,SV);
pJ = fk_j(SP,SV,1:SP.n); 

FF = zeros(3,SP.n+1);
NN = zeros(3,SP.n+1);

for iL = SP.n+1:-1:2
  
  iJ = iL-1;   % joint iJ is the "input" joint for link iL
  iS = SP.C(iL);  % link iS is the link that "supports" link iL
  iJ_S = iS-1; % The input joint of link iS  
   
  Ii = SV.L(iL).R*SP.L(iL).I*SV.L(iL).R';
  Q = [SP.L(iL).m*SV.L(iL).dv - (SV.L(iL).F + SP.L(iL).m*Gravity); 
       Ii*SV.L(iL).dw + cross(SV.L(iL).w,Ii*SV.L(iL).w) - SV.L(iL).T];
  
  NN(:,iL) = NN(:,iL) + Q(4:6) + cross(SV.L(iL).p - pJ(:,iJ), Q(1:3));
  FF(:,iL) = FF(:,iL) + Q(1:3);
    
  if SP.J(iJ).type == 'R'
    tau(iJ) = SV.L(iL).R(:,3)'*NN(:,iL);
  else
    tau(iJ) = SV.L(iL).R(:,3)'*FF(:,iL);
  end
  
  if iS == 1 
    add = cross(pJ(:,iJ)-SV.L(iS).p,FF(:,iL));
  else
    add = cross(pJ(:,iJ)-pJ(:,iJ_S),FF(:,iL));
  end
  NN(:,iS) = NN(:,iS) + NN(:,iL) + add;
  FF(:,iS) = FF(:,iS) + FF(:,iL);  
  
end

% handle the base separately
iL = 1;
Ii = SV.L(iL).R*SP.L(iL).I*SV.L(iL).R';

% SP.L(iL).m*SV.L(iL).dv and Ii*SV.L(iL).dw are not included, because 
% SV.L(iL).dv and SV.L(iL).dw are assumed equal to zero (see forward_recursion.m)
Q = [ -(SV.L(iL).F + SP.L(iL).m*Gravity); 
      cross(SV.L(iL).w,Ii*SV.L(iL).w) - SV.L(iL).T];

NN(:,iL) = NN(:,iL) + Q(4:6);
FF(:,iL) = FF(:,iL) + Q(1:3);

if SP.n > 1
  c = [FF(:,1);NN(:,1);tau(:)];
else
  c = [FF(:,1);NN(:,1)];
end

%%%EOF
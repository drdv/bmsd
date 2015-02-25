function SV = calc_pos_from(SP,SV,base_link)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
% | Kinematics |
% --------------
%
% calc_pos_from
%
% Updates SV.L(i).R and SV.L(i).p starting from SV.L(base_link).
% When base_link = 1 then this function produces the same results as
% calc_pos.m. In other words instead of using Link 1 as a "base" link
% we use Link base_link as a "base" link.
%
% Syntax:
% -------
% SV = calc_pos_from(SP,SV,base_link);
%
% Input:
% ------
% SP    - structure containing system parameters (see struct_def.m)
% SV    - structure containing system variables (see struct_def.m)
% base_link - the link from where to start the recursion. Usually it
%             would be Link 1, but in certain cases we would like to 
%             use a different link as a "base" link
%
% Output:
% -------
% SV
%
% SV members changed:
% --------------------
% SV.L(1:SP.n+1).R except for SV.L(base_link).R
% SV.L(1:SP.n+1).p except for SV.L(base_link).p
%

iS = SP.C(base_link);
links_from_base_link_to_L1 = base_link;
while iS ~= 0
    links_from_base_link_to_L1(end+1) = iS;
    iS = SP.C(iS);
end

% first go from the base_link to Link 1
for i = 2:length(links_from_base_link_to_L1)
    
    iL = links_from_base_link_to_L1(i-1);
    iS = links_from_base_link_to_L1(i);
    iJ = iL-1;
    
    % ------------------
    % update orientation
    % ------------------
    if SP.J(iJ).type == 'R'
        tmp = SP.J(iJ).rpy + [0;0;SV.q(iJ)];
    end
    SV.L(iS).R = SV.L(iL).R * rpy2R(tmp)';
    
    % ----------------------
    % update position of CoM
    % ----------------------
    SV.L(iS).p = SV.L(iL).p - SV.L(iS).R*SP.J(iJ).t - SV.L(iL).R*SP.J(iJ).f;   
    if SP.J(iJ).type == 'P' % Prismatic joint
        SV.L(iS).p = SV.L(iS).p - SV.L(iL).R(:,3)*SV.q(iJ);
    end
  
end

% and then proceed as usual (calc_pos.m)
for iJ = 1:SP.n % iterate over all joints
        
    iL = iJ+1;         % joint iJ is the "input" joint for link iL
    
    % update only unexplored links so far
    if ~ismember(iL, links_from_base_link_to_L1)
  
        iS = SP.C(iL);     % link iS is the link that "supports" link iL
        
        % ------------------
        % update orientation
        % ------------------
        
        % Account for the angle of revolute joints
        % Note that: the Z axis of the CF fixed in a joint is assumed to be the joint's axis of rotation
        if SP.J(iJ).type == 'R'
            SP.J(iJ).rpy(3) = SP.J(iJ).rpy(3) + SV.q(iJ); % SP is changed only locally
        end
        
        SV.L(iL).R = SV.L(iS).R * rpy2R(SP.J(iJ).rpy);
        
        % ----------------------
        % update position of CoM
        % ----------------------
        SV.L(iL).p = SV.L(iS).p + SV.L(iS).R*SP.J(iJ).t + SV.L(iL).R*SP.J(iJ).f;
        
        % Account for the length of prismatic joints
        % Note that: SV.L(iL).R(:,3) stands for SV.L(iL).R*[0;0;1] (where [0;0;1] is the joint axis of translation)
        if SP.J(iJ).type == 'P' % Prismatic joint
            SV.L(iL).p = SV.L(iL).p + SV.L(iL).R(:,3)*SV.q(iJ);
        end
        
    end
    
end

%%%EOF